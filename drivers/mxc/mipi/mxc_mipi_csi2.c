/*
 * Copyright (C) 2011-2014 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/fsl_devices.h>
#include <linux/slab.h>
#include <linux/of.h>

#include <linux/mipi_csi2.h>

#include "mxc_mipi_csi2.h"

static struct mipi_csi2_info *gmipi_csi2;

void _mipi_csi2_lock(struct mipi_csi2_info *info)
{
	if (!in_irq() && !in_softirq())
		mutex_lock(&info->mutex_lock);
}

void _mipi_csi2_unlock(struct mipi_csi2_info *info)
{
	if (!in_irq() && !in_softirq())
		mutex_unlock(&info->mutex_lock);
}

static inline void mipi_csi2_write(struct mipi_csi2_info *info,
		unsigned value, unsigned offset)
{
	writel(value, info->mipi_csi2_base + offset);
}

/* LnT */
//static inline unsigned int mipi_csi2_read(struct mipi_csi2_info *info,
//		unsigned offset)
unsigned int mipi_csi2_read(struct mipi_csi2_info *info,
		unsigned offset)
{
	return readl(info->mipi_csi2_base + offset);
}

/* LnT */
EXPORT_SYMBOL(mipi_csi2_read);

/*!
 * This function is called to enable the mipi csi2 interface.
 *
 * @param	info		mipi csi2 hander
 * @return      Returns setted value
 */
bool mipi_csi2_enable(struct mipi_csi2_info *info)
{
	bool status;

	printk("mipi_csi2_enable : info->mipi_csi2_base %p \n",info->mipi_csi2_base);

	_mipi_csi2_lock(info);

	if (!info->mipi_en) {
		info->mipi_en = true;
		clk_prepare_enable(info->cfg_clk);
		clk_prepare_enable(info->dphy_clk);
	} else
		mipi_dbg("mipi csi2 already enabled!\n");

	status = info->mipi_en;

	_mipi_csi2_unlock(info);

	return status;
}
EXPORT_SYMBOL(mipi_csi2_enable);

/*!
 * This function is called to disable the mipi csi2 interface.
 *
 * @param	info		mipi csi2 hander
 * @return      Returns setted value
 */
bool mipi_csi2_disable(struct mipi_csi2_info *info)
{
	bool status;

	_mipi_csi2_lock(info);

	if (info->mipi_en) {
		info->mipi_en = false;
		clk_disable_unprepare(info->dphy_clk);
		clk_disable_unprepare(info->cfg_clk);
	} else
		mipi_dbg("mipi csi2 already disabled!\n");

	status = info->mipi_en;

	_mipi_csi2_unlock(info);

	return status;
}
EXPORT_SYMBOL(mipi_csi2_disable);

/*!
 * This function is called to get mipi csi2 disable/enable status.
 *
 * @param	info		mipi csi2 hander
 * @return      Returns mipi csi2 status
 */
bool mipi_csi2_get_status(struct mipi_csi2_info *info)
{
	bool status;

	_mipi_csi2_lock(info);
	status = info->mipi_en;
	_mipi_csi2_unlock(info);

	return status;
}
EXPORT_SYMBOL(mipi_csi2_get_status);

/*!
 * This function is called to set mipi lanes.
 *
 * @param	info		mipi csi2 hander
 * @return      Returns setted value
 */
int mipi_csi2_set_lanes(struct mipi_csi2_info *info, unsigned lanes)
{
	if (--lanes > 3)
		return -EINVAL;

	_mipi_csi2_lock(info);
	mipi_csi2_write(info, lanes, MIPI_CSI2_N_LANES);
	lanes = mipi_csi2_read(info, MIPI_CSI2_N_LANES);
	_mipi_csi2_unlock(info);
	return ++lanes;
}
EXPORT_SYMBOL(mipi_csi2_set_lanes);

/*!
 * This function is called to set mipi data type.
 *
 * @param	info		mipi csi2 hander
 * @return      Returns setted value
 */
unsigned int mipi_csi2_set_datatype(struct mipi_csi2_info *info,
					unsigned int datatype)
{
	unsigned int dtype;

	_mipi_csi2_lock(info);
	info->datatype = datatype;
	dtype = info->datatype;
	_mipi_csi2_unlock(info);

	return dtype;
}
EXPORT_SYMBOL(mipi_csi2_set_datatype);

/*!
 * This function is called to get mipi data type.
 *
 * @param	info		mipi csi2 hander
 * @return      Returns mipi data type
 */
unsigned int mipi_csi2_get_datatype(struct mipi_csi2_info *info)
{
	unsigned int dtype;

	_mipi_csi2_lock(info);
	dtype = info->datatype;
	_mipi_csi2_unlock(info);

	return dtype;
}
EXPORT_SYMBOL(mipi_csi2_get_datatype);

/*!
 * This function is called to get mipi csi2 dphy status.
 *
 * @param	info		mipi csi2 hander
 * @return      Returns dphy status
 */
unsigned int mipi_csi2_dphy_status(struct mipi_csi2_info *info)
{
	unsigned int status;

	_mipi_csi2_lock(info);
	status = mipi_csi2_read(info, MIPI_CSI2_PHY_STATE);
	_mipi_csi2_unlock(info);

	return status;
}
EXPORT_SYMBOL(mipi_csi2_dphy_status);

/*!
 * This function is called to get mipi csi2 error1 status.
 *
 * @param	info		mipi csi2 hander
 * @return      Returns error1 value
 */
unsigned int mipi_csi2_get_error1(struct mipi_csi2_info *info)
{
	unsigned int err1;

	_mipi_csi2_lock(info);
	err1 = mipi_csi2_read(info, MIPI_CSI2_ERR1);
	_mipi_csi2_unlock(info);

	return err1;
}
EXPORT_SYMBOL(mipi_csi2_get_error1);

/*!
 * This function is called to get mipi csi2 error1 status.
 *
 * @param	info		mipi csi2 hander
 * @return      Returns error1 value
 */
unsigned int mipi_csi2_get_error2(struct mipi_csi2_info *info)
{
	unsigned int err2;

	_mipi_csi2_lock(info);
	err2 = mipi_csi2_read(info, MIPI_CSI2_ERR2);
	_mipi_csi2_unlock(info);

	return err2;
}
EXPORT_SYMBOL(mipi_csi2_get_error2);

/* LnT */
void mipi_csi2_reg_dump(struct mipi_csi2_info *info)
{
	_mipi_csi2_lock(info);

	printk("MIPI_CSI2_VERSION %x \n",mipi_csi2_read(info, MIPI_CSI2_VERSION));
	printk("MIPI_CSI2_N_LANES %x \n",mipi_csi2_read(info, MIPI_CSI2_N_LANES));
	printk("MIPI_CSI2_PHY_SHUTDOWNZ %x \n",mipi_csi2_read(info, MIPI_CSI2_PHY_SHUTDOWNZ));
	printk("MIPI_CSI2_DPHY_RSTZ %x \n",mipi_csi2_read(info, MIPI_CSI2_DPHY_RSTZ));
	printk("MIPI_CSI2_CSI2_RESETN %x \n",mipi_csi2_read(info, MIPI_CSI2_CSI2_RESETN));
	printk("MIPI_CSI2_PHY_STATE %x \n",mipi_csi2_read(info, MIPI_CSI2_PHY_STATE));
	printk("MIPI_CSI2_DATA_IDS_1 %x \n",mipi_csi2_read(info, MIPI_CSI2_DATA_IDS_1));
	printk("MIPI_CSI2_DATA_IDS_2 %x \n",mipi_csi2_read(info, MIPI_CSI2_DATA_IDS_2));
	printk("MIPI_CSI2_ERR1 %x \n",mipi_csi2_read(info, MIPI_CSI2_ERR1));
	printk("MIPI_CSI2_ERR2 %x \n",mipi_csi2_read(info, MIPI_CSI2_ERR2));
	printk("MIPI_CSI2_MASK1 %x \n",mipi_csi2_read(info, MIPI_CSI2_MASK1));
	printk("MIPI_CSI2_MASK2 %x \n",mipi_csi2_read(info, MIPI_CSI2_MASK2));
	printk("MIPI_CSI2_PHY_TST_CTRL0 %x \n",mipi_csi2_read(info, MIPI_CSI2_PHY_TST_CTRL0));
	printk("MIPI_CSI2_PHY_TST_CTRL1 %x \n",mipi_csi2_read(info, MIPI_CSI2_PHY_TST_CTRL1));
	printk("MIPI_CSI2_SFT_RESET %x \n",mipi_csi2_read(info, MIPI_CSI2_SFT_RESET));

	_mipi_csi2_unlock(info);
}
EXPORT_SYMBOL(mipi_csi2_reg_dump);

/*!
 * This function is called to enable mipi to ipu pixel clock.
 *
 * @param	info		mipi csi2 hander
 * @return      Returns 0 on success or negative error code on fail
 */
int mipi_csi2_pixelclk_enable(struct mipi_csi2_info *info)
{
	return clk_prepare_enable(info->pixel_clk);
}
EXPORT_SYMBOL(mipi_csi2_pixelclk_enable);

/*!
 * This function is called to disable mipi to ipu pixel clock.
 *
 * @param	info		mipi csi2 hander
 * @return      Returns 0 on success or negative error code on fail
 */
void mipi_csi2_pixelclk_disable(struct mipi_csi2_info *info)
{
	clk_disable_unprepare(info->pixel_clk);
}
EXPORT_SYMBOL(mipi_csi2_pixelclk_disable);

/*!
 * This function is called to power on mipi csi2.
 *
 * @param	info		mipi csi2 hander
 * @return      Returns 0 on success or negative error code on fail
 */
int mipi_csi2_reset(struct mipi_csi2_info *info)
{
	_mipi_csi2_lock(info);

	printk("########### mipi_csi2_reset called\n");

	mipi_csi2_write(info, 0x0, MIPI_CSI2_PHY_SHUTDOWNZ);
	mipi_csi2_write(info, 0x0, MIPI_CSI2_DPHY_RSTZ);
	mipi_csi2_write(info, 0x0, MIPI_CSI2_CSI2_RESETN);

	mipi_csi2_write(info, 0x00000001, MIPI_CSI2_PHY_TST_CTRL0);
	mipi_csi2_write(info, 0x00000000, MIPI_CSI2_PHY_TST_CTRL1);
	mipi_csi2_write(info, 0x00000000, MIPI_CSI2_PHY_TST_CTRL0);
	mipi_csi2_write(info, 0x00000002, MIPI_CSI2_PHY_TST_CTRL0);
	mipi_csi2_write(info, 0x00010044, MIPI_CSI2_PHY_TST_CTRL1);
	mipi_csi2_write(info, 0x00000000, MIPI_CSI2_PHY_TST_CTRL0);

/* LnT */
	mipi_csi2_write(info, 0x00000014, MIPI_CSI2_PHY_TST_CTRL1);//Default for OV5640, if not,it would lead to ERR1 and ERR2
//	mipi_csi2_write(info, 0x00000044, MIPI_CSI2_PHY_TST_CTRL1); //187.5MHz, 187.5*2 = 360-400MHz, its for leonardo camera


	mipi_csi2_write(info, 0x00000002, MIPI_CSI2_PHY_TST_CTRL0);
	mipi_csi2_write(info, 0x00000000, MIPI_CSI2_PHY_TST_CTRL0);

	mipi_csi2_write(info, 0xffffffff, MIPI_CSI2_PHY_SHUTDOWNZ);
	mipi_csi2_write(info, 0xffffffff, MIPI_CSI2_DPHY_RSTZ);
	mipi_csi2_write(info, 0xffffffff, MIPI_CSI2_CSI2_RESETN);

	_mipi_csi2_unlock(info);

	return 0;
}
EXPORT_SYMBOL(mipi_csi2_reset);

/*!
 * This function is called to get mipi csi2 info.
 *
 * @return      Returns mipi csi2 info struct pointor
 */
struct mipi_csi2_info *mipi_csi2_get_info(void)
{
	return gmipi_csi2;
}
EXPORT_SYMBOL(mipi_csi2_get_info);

//LnT
int mipi_csi2_reset_with_dphy_freq(struct mipi_csi2_info *info, uint32_t value)
{
	_mipi_csi2_lock(info);

	printk("########### mipi_csi2_reset_with_dphy_freq called\n");

	mipi_csi2_write(info, 0x0, MIPI_CSI2_PHY_SHUTDOWNZ);
	mipi_csi2_write(info, 0x0, MIPI_CSI2_DPHY_RSTZ);
	mipi_csi2_write(info, 0x0, MIPI_CSI2_CSI2_RESETN);

	/* phy_testclr = 1 (0b01) test interface clear */
	mipi_csi2_write(info, 0x00000001, MIPI_CSI2_PHY_TST_CTRL0);
	/* phy_testdin (0) test interface input */
	mipi_csi2_write(info, 0x00000000, MIPI_CSI2_PHY_TST_CTRL1);
	/* 32'h00000000 this disables the testclr pin
	 * enabling the interface to write new values to the DPHY internal registers. */
	mipi_csi2_write(info, 0x00000000, MIPI_CSI2_PHY_TST_CTRL0);
	/* phy_testclk = 1 (0b10) test interface strobe signal
	 * use to clock TESTDIN bus into D-PHY */
	mipi_csi2_write(info, 0x00000002, MIPI_CSI2_PHY_TST_CTRL0);
	/* phy_testen (1) configure address write operation of TESTCLK
	 * phy_testdin (0x44) register address */
	mipi_csi2_write(info, 0x00010044, MIPI_CSI2_PHY_TST_CTRL1);
	mipi_csi2_write(info, 0x00000000, MIPI_CSI2_PHY_TST_CTRL0);
	/* change clock */
	mipi_csi2_write(info, value, MIPI_CSI2_PHY_TST_CTRL1);
	/* phy_testclk = 1 (0b10) test interface strobe signal
	 * use to clock TESTDIN bus into D-PHY */
	mipi_csi2_write(info, 0x00000002, MIPI_CSI2_PHY_TST_CTRL0);
	mipi_csi2_write(info, 0x00000000, MIPI_CSI2_PHY_TST_CTRL0);

	mipi_csi2_write(info, 0xffffffff, MIPI_CSI2_PHY_SHUTDOWNZ);
	mipi_csi2_write(info, 0xffffffff, MIPI_CSI2_DPHY_RSTZ);
	mipi_csi2_write(info, 0xffffffff, MIPI_CSI2_CSI2_RESETN);

	_mipi_csi2_unlock(info);

	return 0;
}
EXPORT_SYMBOL(mipi_csi2_reset_with_dphy_freq);


/**
 * This function is called by the driver framework to initialize the MIPI CSI2
 * device.
 *
 * @param	pdev	The device structure for the MIPI CSI2 passed in by the
 *			driver framework.
 *
 * @return      Returns 0 on success or negative error code on error
 */
static int mipi_csi2_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	u32 mipi_csi2_dphy_ver;
	int ret;

	gmipi_csi2 = kmalloc(sizeof(struct mipi_csi2_info), GFP_KERNEL);
	if (!gmipi_csi2) {
		ret = -ENOMEM;
		goto alloc_failed;
	}

	/* initialize mutex */
	mutex_init(&gmipi_csi2->mutex_lock);

	/* get mipi csi2 informaiton */
	gmipi_csi2->pdev = pdev;
	gmipi_csi2->mipi_en = false;

	gmipi_csi2->cfg_clk = devm_clk_get(dev, "cfg_clk");
	if (IS_ERR(gmipi_csi2->cfg_clk)) {
		dev_err(&pdev->dev, "failed to get cfg_clk\n");
		ret = PTR_ERR(gmipi_csi2->cfg_clk);
		goto err;
	}

	/* get mipi dphy clk */
	gmipi_csi2->dphy_clk = devm_clk_get(dev, "dphy_clk");
	if (IS_ERR(gmipi_csi2->dphy_clk)) {
		dev_err(&pdev->dev, "failed to get dphy pll_ref_clk\n");
		ret = PTR_ERR(gmipi_csi2->dphy_clk);
		goto err;
	}

	/* get mipi to ipu pixel clk */
	gmipi_csi2->pixel_clk = devm_clk_get(dev, "pixel_clk");
	if (IS_ERR(gmipi_csi2->pixel_clk)) {
		dev_err(&pdev->dev, "failed to get mipi pixel clk\n");
		ret = PTR_ERR(gmipi_csi2->pixel_clk);
		goto err;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -ENODEV;
		goto err;
	}

	/* mipi register mapping */
	gmipi_csi2->mipi_csi2_base = ioremap(res->start, PAGE_SIZE);
	if (!gmipi_csi2->mipi_csi2_base) {
		ret = -ENOMEM;
		goto err;
	}

	/* mipi dphy clk enable for register access */
	clk_prepare_enable(gmipi_csi2->dphy_clk);
	/* get mipi csi2 dphy version */
	mipi_csi2_dphy_ver = mipi_csi2_read(gmipi_csi2, MIPI_CSI2_VERSION);

	clk_disable_unprepare(gmipi_csi2->dphy_clk);

	platform_set_drvdata(pdev, gmipi_csi2);

	dev_info(&pdev->dev, "i.MX MIPI CSI2 driver probed\n");
	dev_info(&pdev->dev, "i.MX MIPI CSI2 dphy version is 0x%x\n",
						mipi_csi2_dphy_ver);

	return 0;

err:
	kfree(gmipi_csi2);
alloc_failed:
	dev_err(&pdev->dev, "i.MX MIPI CSI2 driver probed -  error\n");
	return ret;
}

static int mipi_csi2_remove(struct platform_device *pdev)
{
	/* unmapping mipi register */
	iounmap(gmipi_csi2->mipi_csi2_base);

	kfree(gmipi_csi2);

	dev_set_drvdata(&pdev->dev, NULL);

	return 0;
}

static const struct of_device_id imx_mipi_csi2_dt_ids[] = {
	{ .compatible = "fsl,imx6q-mipi-csi2", },
	{ /* sentinel */ }
};

static struct platform_driver mipi_csi2_driver = {
	.driver = {
		   .name = "mxc_mipi_csi2",
		   .of_match_table = imx_mipi_csi2_dt_ids,
	},
	.probe = mipi_csi2_probe,
	.remove = mipi_csi2_remove,
};

static int __init mipi_csi2_init(void)
{
	int err;

	err = platform_driver_register(&mipi_csi2_driver);
	if (err) {
		pr_err("mipi_csi2_driver register failed\n");
		return -ENODEV;
	}

	pr_info("MIPI CSI2 driver module loaded\n");

	return 0;
}

static void __exit mipi_csi2_cleanup(void)
{
	platform_driver_unregister(&mipi_csi2_driver);
}

subsys_initcall(mipi_csi2_init);
module_exit(mipi_csi2_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("i.MX MIPI CSI2 driver");
MODULE_LICENSE("GPL");
