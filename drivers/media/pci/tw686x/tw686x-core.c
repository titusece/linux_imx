/*
 * Copyright (C) 2015 VanguardiaSur - www.vanguardiasur.com.ar
 *
 * Based on original driver by Krzysztof Hałasa:
 * Copyright (C) 2015 Industrial Research Institute for Automation
 * and Measurements PIAP
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License
 * as published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci_ids.h>
#include <linux/slab.h>
#include <linux/timer.h>

#include "tw686x.h"
#include "tw686x-regs.h"

static u32 dma_interval = 0x00098968;
module_param(dma_interval, int, 0444);
MODULE_PARM_DESC(dma_interval, "Minimum time span for DMA interrupting host");

/*
 * The purpose of this awful hack is to avoid
 * enabling the DMA channels "too fast" which seems
 * to make some TW686x devices _very_ angry and freeze the CPU.
 */
static void tw686x_dma_delay(unsigned long data)
{
        struct tw686x_dev *dev = (struct tw686x_dev *) data;
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);

	reg_write(dev, DMA_CHANNEL_ENABLE, dev->pending_dma_en);
	reg_write(dev, DMA_CMD, dev->pending_dma_cmd);
	dev->pending_dma_en = 0;
	dev->pending_dma_cmd = 0;

	spin_unlock_irqrestore(&dev->lock, flags);
}

static void tw686x_reset_channels(struct tw686x_dev *dev, unsigned int ch_mask)
{
	u32 dma_en, dma_cmd;

	dma_en = reg_read(dev, DMA_CHANNEL_ENABLE);
	dma_cmd = reg_read(dev, DMA_CMD);

	/*
	 * Save pending register status, the timer will
	 * restore them.
	 */
	dev->pending_dma_en |= dma_en;
	dev->pending_dma_cmd |= dma_cmd;

	/* Disable the reset channels */
	reg_write(dev, DMA_CHANNEL_ENABLE, dma_en & ~ch_mask);

	if ((dma_en & ~ch_mask) == 0) {
		dev_dbg(&dev->pci_dev->dev, "reset: stopping DMA\n");
		dma_cmd &= ~BIT(31);
	}
	reg_write(dev, DMA_CMD, dma_cmd & ~ch_mask);
}

static irqreturn_t tw686x_irq(int irq, void *dev_id)
{
	struct tw686x_dev *dev = (struct tw686x_dev *)dev_id;
	unsigned int video_requests, audio_requests, reset_ch;
	u32 fifo_status, fifo_signal, fifo_ov, fifo_bad, fifo_errors;
	u32 int_status, dma_en, video_en, pb_status;
	unsigned long flags;

	int_status = reg_read(dev, INT_STATUS); /* cleared on read */
	fifo_status = reg_read(dev, VIDEO_FIFO_STATUS);

	/* INT_STATUS does not include FIFO_STATUS errors! */
	if (!int_status && !TW686X_FIFO_ERROR(fifo_status))
		return IRQ_NONE;

	if (int_status & BIT(17)) {
		dev_dbg(&dev->pci_dev->dev,
			"DMA timeout. Resetting DMA for all channels\n");
		reset_ch = ~0;
		goto reset_channels;
	}

	spin_lock_irqsave(&dev->lock, flags);
	dma_en = reg_read(dev, DMA_CHANNEL_ENABLE);
	spin_unlock_irqrestore(&dev->lock, flags);

	video_en = dma_en & 0xff;
	fifo_signal = ~(fifo_status & 0xff) & video_en;
	fifo_ov = fifo_status >> 24;
	fifo_bad = fifo_status >> 16;

	/* Mask of channels with signal and FIFO errors */
	fifo_errors = fifo_signal & (fifo_ov | fifo_bad);

	reset_ch = 0;
	pb_status = reg_read(dev, PB_STATUS);

	/* Coalesce video frame/error events */
	video_requests = (int_status & video_en) | fifo_errors;
	audio_requests = (int_status & dma_en) >> 8;

	if (video_requests)
		tw686x_video_irq(dev, video_requests, pb_status,
				 fifo_status, &reset_ch);
	if (audio_requests)
		tw686x_audio_irq(dev, audio_requests, pb_status);

reset_channels:
	if (reset_ch) {
		spin_lock_irqsave(&dev->lock, flags);
		tw686x_reset_channels(dev, reset_ch);
		spin_unlock_irqrestore(&dev->lock, flags);
		mod_timer(&dev->dma_delay_timer,
			  jiffies + msecs_to_jiffies(100));
	}

	return IRQ_HANDLED;
}

static void tw686x_dev_release(struct v4l2_device *v4l2_dev)
{
        struct tw686x_dev *dev = container_of(v4l2_dev, struct tw686x_dev,
					      v4l2_dev);
	unsigned int ch;

	for (ch = 0; ch < max_channels(dev); ch++)
		v4l2_ctrl_handler_free(&dev->video_channels[ch].ctrl_handler);

	v4l2_device_unregister(&dev->v4l2_dev);

	kfree(dev->audio_channels);
	kfree(dev->video_channels);
	kfree(dev);
}

static int tw686x_probe(struct pci_dev *pci_dev,
			const struct pci_device_id *pci_id)
{
	struct tw686x_dev *dev;
	int err;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	dev->type = pci_id->driver_data;
	sprintf(dev->name, "tw%04X", pci_dev->device);

	dev->video_channels = kcalloc(max_channels(dev),
		sizeof(*dev->video_channels), GFP_KERNEL);
	if (!dev->video_channels) {
		err = -ENOMEM;
		goto free_dev;
	}

	dev->audio_channels = kcalloc(max_channels(dev),
		sizeof(*dev->audio_channels), GFP_KERNEL);
	if (!dev->audio_channels) {
		err = -ENOMEM;
		goto free_video;
	}

	pr_info("%s: PCI %s, IRQ %d, MMIO 0x%lx\n", dev->name,
		pci_name(pci_dev), pci_dev->irq,
		(unsigned long)pci_resource_start(pci_dev, 0));

	dev->pci_dev = pci_dev;
	if (pci_enable_device(pci_dev)) {
		err = -EIO;
		goto free_audio;
	}

	pci_set_master(pci_dev);
	err = pci_set_dma_mask(pci_dev, DMA_BIT_MASK(32));
	if (err) {
		dev_err(&pci_dev->dev, "32-bit PCI DMA not supported\n");
		err = -EIO;
		goto disable_pci;
	}

	err = pci_request_regions(pci_dev, dev->name);
	if (err) {
		dev_err(&pci_dev->dev, "unable to request PCI region\n");
		goto disable_pci;
	}

	dev->mmio = pci_ioremap_bar(pci_dev, 0);
	if (!dev->mmio) {
		dev_err(&pci_dev->dev, "unable to remap PCI region\n");
		err = -ENOMEM;
		goto free_region;
	}

	/* Reset all subsystems */
	reg_write(dev, SYS_SOFT_RST, 0x0F);
	mdelay(1);

	reg_write(dev, SRST[0], 0x3F);
	if (max_channels(dev) > 4)
		reg_write(dev, SRST[1], 0x3F);

	/* Disable the DMA engine */
	reg_write(dev, DMA_CMD, 0);
	reg_write(dev, DMA_CHANNEL_ENABLE, 0);

	/* Enable DMA FIFO overflow and pointer check */
	reg_write(dev, DMA_CONFIG, 0xFFFFFF04);
	reg_write(dev, DMA_CHANNEL_TIMEOUT, 0x140C8584);
	reg_write(dev, DMA_TIMER_INTERVAL, dma_interval);

	spin_lock_init(&dev->lock);

	err = request_irq(pci_dev->irq, tw686x_irq, IRQF_SHARED,
			  dev->name, dev);
	if (err < 0) {
		dev_err(&pci_dev->dev, "unable to request interrupt\n");
		goto iounmap;
	}

	setup_timer(&dev->dma_delay_timer,
		    tw686x_dma_delay, (unsigned long) dev);

	/*
	 * This must be set right before initializing v4l2_dev.
	 * It's used to release resources after the last handle
	 * held is released.
	 */
	dev->v4l2_dev.release = tw686x_dev_release;
	err = tw686x_video_init(dev);
	if (err) {
		dev_err(&pci_dev->dev, "can't register video\n");
		goto free_irq;
	}

	err = tw686x_audio_init(dev);
	if (err) {
		dev_warn(&pci_dev->dev, "can't register audio\n");
	}

	pci_set_drvdata(pci_dev, dev);
	return 0;

free_irq:
	free_irq(pci_dev->irq, dev);
iounmap:
	pci_iounmap(pci_dev, dev->mmio);
free_region:
	pci_release_regions(pci_dev);
disable_pci:
	pci_disable_device(pci_dev);
free_audio:
	kfree(dev->audio_channels);
free_video:
	kfree(dev->video_channels);
free_dev:
	kfree(dev);
	return err;
}

static void tw686x_remove(struct pci_dev *pci_dev)
{
	struct tw686x_dev *dev = pci_get_drvdata(pci_dev);
	unsigned long flags;

	/* This guarantees the IRQ handler is no longer running,
	 * which means we can kiss good-bye some resources.
	 */
	free_irq(pci_dev->irq, dev);

	tw686x_video_free(dev);
	tw686x_audio_free(dev);
	del_timer_sync(&dev->dma_delay_timer);

	pci_iounmap(pci_dev, dev->mmio);
	pci_release_regions(pci_dev);
	pci_disable_device(pci_dev);

	/*
	 * This allows to detect device is not here,
	 * and will be used by vb2_ops. The lock is really
	 * important here.
	 */
	spin_lock_irqsave(&dev->lock, flags);
	dev->pci_dev = NULL;
	spin_unlock_irqrestore(&dev->lock, flags);

        /*
         * This calls tw686x_dev_release if it's the last reference.
         * therwise, release is posponed until there are no users left.
         */
        v4l2_device_put(&dev->v4l2_dev);
}

/*
 * On TW6864 and TW6868, all channels share the pair of video DMA SG tables,
 * with 10-bit start_idx and end_idx determining start and end of frame buffer
 * for particular channel.
 * TW6868 with all its 8 channels would be problematic (only 127 SG entries per
 * channel) but we support only 4 channels on this chip anyway (the first
 * 4 channels are driven with internal video decoder, the other 4 would require
 * an external TW286x part).
 *
 * On TW6865 and TW6869, each channel has its own DMA SG table, with indexes
 * starting with 0. Both chips have complete sets of internal video decoders
 * (respectively 4 or 8-channel).
 *
 * All chips have separate SG tables for two video frames.
 */

/* driver_data is number of A/V channels */
static const struct pci_device_id tw686x_pci_tbl[] = {
	{
		PCI_DEVICE(PCI_VENDOR_ID_TECHWELL, 0x6864),
		.driver_data = 4
	},
	{
		PCI_DEVICE(PCI_VENDOR_ID_TECHWELL, 0x6865), /* not tested */
		.driver_data = 4 | TYPE_SECOND_GEN
	},
	/*
	 * TW6868 supports 8 A/V channels with an external TW2865 chip;
	 * not supported by the driver.
	 */
	{
		PCI_DEVICE(PCI_VENDOR_ID_TECHWELL, 0x6868), /* not tested */
		.driver_data = 4
	},
	{
		PCI_DEVICE(PCI_VENDOR_ID_TECHWELL, 0x6869),
		.driver_data = 8 | TYPE_SECOND_GEN},
	{}
};
MODULE_DEVICE_TABLE(pci, tw686x_pci_tbl);

static struct pci_driver tw686x_pci_driver = {
	.name = "tw686x",
	.id_table = tw686x_pci_tbl,
	.probe = tw686x_probe,
	.remove = tw686x_remove,
};
module_pci_driver(tw686x_pci_driver);

MODULE_DESCRIPTION("Driver for video frame grabber cards based on Intersil/Techwell TW686[4589]");
MODULE_AUTHOR("Ezequiel Garcia <ezequiel@xxxxxxxxxxxxxxxxxxxx>");
MODULE_LICENSE("GPL v2");
