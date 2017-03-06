/*
 * Copyright (C) 2016 Allied Vision Technologies , Inc. All Rights Reserved.
 */

/*
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
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/proc_fs.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <linux/mipi_csi2.h>
#include <media/v4l2-chip-ident.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include "v4l2-int-device.h"
#include "mxc_v4l2_capture.h"
#include "leonardo_mipi.h"
#include <linux/platform_data/i2c-imx.h>

#include <linux/byteorder/little_endian.h>

/* LnT */
/* #define DEBUG */

#define LNT_DEBUG

#ifdef LNT_DEBUG
	#define LNT_TRACE(fmt, args...) pr_debug("%s:%d " fmt "\n" ,__func__,__LINE__,##args)
#else
	#define LNT_TRACE(fmt, args...) do { } while(0);
#endif

#define MIN_FPS 15	// TODO
#define MAX_FPS 45	// TODO
#define DEFAULT_FPS 30	// TODO
#define CRC32MASKREV                            0xEDB88320u  
#define UINT32_MAXIMUM	                        0xffffffff

#define LEONARDO_XCLK_MIN 6000000
#define LEONARDO_XCLK_MAX 188000000

enum leonardo_mode {
	leonardo_mode_MIN = 0,
	leonardo_mode_SVGA_800_600 = 0,
//	leonardo_mode_VGA_640_480 = 0,
	leonardo_mode_QVGA_320_240 = 1,
	leonardo_mode_NTSC_720_480 = 2,
/*
	leonardo_mode_PAL_720_576 = 3,
	leonardo_mode_720P_1280_720 = 4,
	leonardo_mode_1080P_1920_1080 = 5,
	leonardo_mode_QSXGA_2592_1944 = 6,
	leonardo_mode_QCIF_176_144 = 7,
	leonardo_mode_XGA_1024_768 = 8,
	leonardo_mode_MAX = 8,
*/
	leonardo_mode_MAX = 2,
	leonardo_mode_INIT = 0xff, /*only for sensor init*/
};

/* list of image formats supported by Leonardo camera sensor */
static const struct v4l2_fmtdesc leonardo_formats[] = {
	{
		.description	= "RGB888 (RGB24)",
		.pixelformat	= V4L2_PIX_FMT_RGB24,		/* 24  RGB-8-8-8     */
		.flags		= MIPI_DT_RGB888		//	0x24
	},
	{
		.description	= "RAW12 (Y/CbCr 4:2:0)",
		.pixelformat	= V4L2_PIX_FMT_UYVY,		/* 12  Y/CbCr 4:2:0  */
		.flags		= MIPI_DT_RAW12			//	0x2c
	},
	{
		.description	= "YUV 4:2:2 8-bit",
		.pixelformat	= V4L2_PIX_FMT_YUYV, 		/*  8  8-bit color   */
		.flags		= MIPI_DT_YUV422		//	0x1e /* UYVY...		*/
	},
};

#define IO_LIMIT	1024

struct reg_value {
	u16 u16RegAddr;
	u8 u8Val;
	u8 u8Mask;
	u32 u32Delay_ms;
};

static int virtual_channel = 0;
static int is_type31 = 0;
static int lane_count = 0;
static bool gencp_mode = true;

/*!
 * Maintains the information on the current state of the sesor.
 */
static struct sensor_data leonardo_data;
static int leonardo_i2c_clock;
static int leonardo_init_mode(enum leonardo_mode mode);

static int leonardo_probe(struct i2c_client *adapter,
				const struct i2c_device_id *device_id);
static int leonardo_remove(struct i2c_client *client);

static s32 leonardo_read_reg(u16 reg, u8 *val);
static s32 leonardo_write_reg(u16 reg, u8 val);

int read_cci_registers( void);
int read_gencp_registers( void);
uint32_t Crc32(const void *pSrc, uint32_t nByteCount, const uint32_t * pnStartCrcValue);

char *swapbytes512(char *val);

static const struct i2c_device_id leonardo_id[] = {
	{"leonardo_mipi", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, leonardo_id);

static struct i2c_driver leonardo_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "leonardo_mipi",
		  },
	.probe  = leonardo_probe,
	.remove = leonardo_remove,
	.id_table = leonardo_id,
};

static void leonardo_reset(void)
{
	// TODO
}

static s32 leonardo_write_reg(u16 reg, u8 val)
{
	int ret = 0;
	u8 au8Buf[3] = {0};

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val;

	{
		ret = i2c_master_send(leonardo_data.i2c_client, au8Buf, 3);
	}

	if (ret < 0) {
		pr_err("%s, I2C Write failed reg=%x,val=%x error=%d\n",
			__func__, reg, val, ret);
		return ret;
	}
	return ret;
}

static s32 leonardo_read_reg(u16 reg, u8 *val)
{
	struct sensor_data *sensor = &leonardo_data;
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msgs[2];
	u8 buf[2];
	int ret = 0;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = buf;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = buf;

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0) {
		pr_err("%s : I2C Read failed reg=%x ret=%d\n", __func__, reg, ret);
		return ret;
	}
	*val = buf[0];
	pr_debug("%s(mipi):reg=%x,val=%x\n", __func__, reg, buf[0]);

	return 0;
}


static void leonardo_stream_on(void)
{
	leonardo_write_reg(0x4202, 0x00);
}

static void leonardo_stream_off(void)
{
	leonardo_write_reg(0x4202, 0x0f);
}


static int leonardo_get_gain16(void)
{
	 /* read gain, 16 = 1x */
	int gain16;
	//u8 temp;
	// TODO 

	return gain16;
}

static int leonardo_set_gain16(int gain16)
{
	/* write gain, 16 = 1x */
	// TODO
	return 0;
}

#define VIRTUAL_CHANNEL_REG 0x1
 
static void leonardo_set_virtual_channel(int channel)
{
	u8 channel_id;
	// TODO
	leonardo_read_reg(VIRTUAL_CHANNEL_REG, &channel_id);
	channel_id &= ~(3 << 6);
	leonardo_write_reg(VIRTUAL_CHANNEL_REG, channel_id | (channel << 6));
	pr_info("%s: virtual channel=%d\n", __func__, channel);
}

static int trigger_auto_focus(void){
	// TODO
	return 0;
}

/* LnT : */
static ssize_t i2c_read(struct v4l2_i2c *i2c_reg)
{
	printk("LEONARDO CAMERA : i2c_read executed! %s : %d \n",__func__,__LINE__);

	struct i2c_msg msg[2];
	u8 msgbuf[i2c_reg->reg_size];
	struct sensor_data *sensor = &leonardo_data;
	struct i2c_client *client = sensor->i2c_client;
	int status = 0, i = 0, j = 0, k = 0, reg_size_bkp;
/* LnT : Debug prints, 'temp_buffer' is used to print the CCI values from camera. */
/*	char *temp_buffer; */

	reg_size_bkp = i2c_reg->reg_size;
	printk("reg_size_bkp = %d i2c_reg->reg_size = %d\n",reg_size_bkp,i2c_reg->reg_size);
	
	/* clearing i2c msg with 0's */
	memset(msg, 0, sizeof(msg));

	
	if (i2c_reg->count > IO_LIMIT)
	{
		printk("Limit excedded! i2c_reg->count > IO_LIMIT\n");
		i2c_reg->count = IO_LIMIT;
	}	

/* LnT : Debug prints */
/*
	temp_buffer = kmalloc(i2c_reg->count * sizeof(char), GFP_KERNEL);
	if(temp_buffer == NULL){	
		printk("error allocating memory:\n"); 
	}
*/

	/* find start address of buffer */
	for(i = --i2c_reg->reg_size ; i >= 0 ; i--,j++)				
		msgbuf[i] = ((i2c_reg->reg >> (8*j)) & 0xFF);

/* LnT : Debug prints */
	for(k = 0; k < reg_size_bkp; k++)
		printk("I2C read message buffer msgbuf[%d] %x, i2c_reg->count %d , client->addr %x\n",k,msgbuf[k],i2c_reg->count,client->addr);

	msg[0].addr = client->addr;
	msg[1].flags = 0;
	msg[0].len = reg_size_bkp;
	msg[0].buf = msgbuf;

	msg[1].addr = client->addr;  //slave address
	msg[1].flags = I2C_M_RD;  // Read flag setting
	msg[1].len = i2c_reg->count;
	msg[1].buf = i2c_reg->buffer; //dest buffer

/* LnT : Debug prints */
/*	temp_buffer = i2c_reg->buffer; */

	status = i2c_transfer(client->adapter, msg, 2); 

	if (status < 0)
		pr_err("%s:I2C read failed, status = %d\n", __func__, status);

/* LnT : Debug prints */
/*
	for(i = 0; i < i2c_reg->count; i++)
	{	
		printk("count = %d,  temp_buffer data %p, temp_buffer addr %p\n",i,*temp_buffer,temp_buffer);
		temp_buffer++;
	}			
*/

/* LnT : Debug prints */
/*	kfree(temp_buffer); */
		
	return status;
		
}

static s32 ioctl_gencam_i2cread_reg(struct v4l2_int_device *s, struct v4l2_i2c *i2c_reg)
{
		ssize_t	status;
		printk("LEONARDO CAMERA : ioctl_gencam_i2cread_reg executed! %s:%d,  i2c_reg->count %d \n",__func__,__LINE__,i2c_reg->count);
		status = i2c_read(i2c_reg);

		if(status < 0)
		printk("%s:%d I2C Read failed, status = %d Bytes read = %d\n",__func__,__LINE__,status,i2c_reg->count);		
		else
		printk("status = %d Bytes read = %d\n",status,i2c_reg->count);

		return status;
}



/* LnT  */
static ssize_t i2c_write(struct v4l2_i2c *i2c_reg)
{
	printk("LEONARDO CAMERA : i2c_write executed! %s : %d\n",__func__,__LINE__);

	int j = 0, i = 0, ret = 0, reg_size_bkp = 0;
	int total_size = i2c_reg->count + i2c_reg->reg_size;	
	u8 i2c_w_buf [total_size];

	if (i2c_reg->count > IO_LIMIT)   // count exceeds writting IO_LIMIT characters
	{
		printk("***************************** Limit exceeded ***********************************\n");	
		i2c_reg->count = IO_LIMIT;
	}

	char *temp_buff;
	temp_buff = kmalloc(i2c_reg->count * sizeof(char), GFP_KERNEL);
	if(temp_buff == NULL){
		printk("failed to allocate memory:\n");
		return -1;	
	}


	if ( ret = (copy_from_user(temp_buff,i2c_reg->buffer,i2c_reg->count)) )
	{
	pr_err("Copy from user is failed... %s %d\n",__FILE__,__LINE__);
	kfree(temp_buff);
	return ret;	
	}


	/* Backup the size of register which needs to be read after filling the address buffer */
	reg_size_bkp = i2c_reg->reg_size;
	printk(" i2c_write: reg_size_bkp=%d i2c_reg->reg_size=%d  %s\n",reg_size_bkp,i2c_reg->reg_size, __FILE__);

// /****
	// Fill the address in buffer upto size of address want to write */
	for(i = --i2c_reg->reg_size; i >= 0 ; i--,j++)				
		i2c_w_buf[i] = ((i2c_reg->reg >> (8*j)) & 0xFF);	

	/* Append the data value in the same buffer  */
	for(i = reg_size_bkp; i < (i2c_reg->count + reg_size_bkp) ; i++,temp_buff++)
		i2c_w_buf[i] = *temp_buff;


	/* Debug purpose: Need to print the whole buffer and see correctly formed the address and data in i2c msg buffer */
	for(i = 0; i < total_size; i++)
		printk("I2C write: message buffer i2c_w_buf[%d] %x %s\n",i,i2c_w_buf[i], __FILE__);
// ****/
	
	ret = i2c_master_send(leonardo_data.i2c_client, i2c_w_buf, total_size );

	printk("i2c_write : ret %d\n",ret);

//TODO
	kfree(temp_buff);
	return ret;
}

static s32 bcrm_i2cwrite_reg(struct v4l2_i2c *i2c_reg)
{
	int j = 0, i = 0, ret = 0, reg_size_bkp = 0;
	int total_size = i2c_reg->count + i2c_reg->reg_size;	
	u8 i2c_w_buf [total_size];
	u32 temp_buffer_bkp;

	LNT_TRACE("\n");

	printk("LEONARDO CAMERA : i2c_write executed! %s : %d, i2c_reg->count %d, i2c_reg->reg_size %d, total_size %d \n",__func__,__LINE__,i2c_reg->count,i2c_reg->reg_size,total_size);

	if (i2c_reg->count > IO_LIMIT)   // count exceeds writting IO_LIMIT characters
	{
		printk("***************************** Limit exceeded ***********************************\n");	
		i2c_reg->count = IO_LIMIT;
	}

	char *temp_buff;
	temp_buff = kmalloc(i2c_reg->count * sizeof(char), GFP_KERNEL);
	if(temp_buff == NULL){
		printk("failed to allocate memory:\n");
		return -1;	
	}

	temp_buffer_bkp = temp_buff;
//	printk("############### temp_buff address 0x%x, temp_buffer_bkp address 0x%x ################\n",temp_buff,temp_buffer_bkp);

//	ret = copy_from_user(temp_buff,i2c_reg->buffer,i2c_reg->count);
	ret = memcpy(temp_buff,i2c_reg->buffer,i2c_reg->count);
	if (ret < 0)
	{
	pr_err("Copy from user is failed... %s:%d,  ret %d\n",__FILE__,__LINE__,ret);
	kfree(temp_buff);
	return ret;	
	}

	/* Backup the size of register which needs to be read after filling the address buffer */
	reg_size_bkp = i2c_reg->reg_size;
	printk(" i2c_write: reg_size_bkp=%d i2c_reg->reg_size=%d  %s\n",reg_size_bkp,i2c_reg->reg_size, __FILE__);

// /****
	// Fill the address in buffer upto size of address want to write */
	for(i = --i2c_reg->reg_size; i >= 0 ; i--,j++)				
		i2c_w_buf[i] = ((i2c_reg->reg >> (8*j)) & 0xFF);	

	/* Append the data value in the same buffer  */
	for(i = reg_size_bkp; i < (i2c_reg->count + reg_size_bkp) ; i++,temp_buff++)
		i2c_w_buf[i] = *temp_buff;


	/* Debug purpose: Need to print the whole buffer and see correctly formed the address and data in i2c msg buffer */
	for(i = 0; i < total_size; i++)
		printk("I2C write: message buffer i2c_w_buf[%d] %x %s\n",i,i2c_w_buf[i], __FILE__);
// ****/
	
	ret = i2c_master_send(leonardo_data.i2c_client, i2c_w_buf, total_size );

	printk("i2c_write : ret %d\n",ret);

	kfree(temp_buffer_bkp);

	return ret;
}


static s32 ioctl_gencam_i2cwrite_reg(struct v4l2_int_device *s, struct v4l2_i2c *i2c_reg)
{
	int j = 0, i = 0, ret = 0, reg_size_bkp = 0;
	int total_size = i2c_reg->count + i2c_reg->reg_size;	
	u8 i2c_w_buf [total_size];
	u32 temp_buffer_bkp;

	LNT_TRACE("\n");

	printk("LEONARDO CAMERA : i2c_write executed! %s : %d, i2c_reg->buffer %d \n",__func__,__LINE__,i2c_reg->buffer[0]);

	if (i2c_reg->count > IO_LIMIT)   // count exceeds writting IO_LIMIT characters
	{
		printk("***************************** Limit exceeded ***********************************\n");	
		i2c_reg->count = IO_LIMIT;
	}

	char *temp_buff;
	temp_buff = kmalloc(i2c_reg->count * sizeof(char), GFP_KERNEL);
	if(temp_buff == NULL){
		printk("failed to allocate memory:\n");
		return -1;	
	}

	temp_buffer_bkp = temp_buff;
//	printk("############### temp_buff address 0x%x, temp_buffer_bkp address 0x%x ################\n",temp_buff,temp_buffer_bkp);

	if (gencp_mode) {
		LNT_TRACE("GENCP i2c write transaction!\n");
		ret = copy_from_user(temp_buff,i2c_reg->buffer,i2c_reg->count);
		LNT_TRACE(" i2c_write: i2c_reg->buffer[0] %d, temp_buff[0] %d\n",i2c_reg->buffer[0],temp_buff[0]);
	}
	else {
		LNT_TRACE("BCRM i2c write transaction!\n");
		LNT_TRACE(" Before memcpy i2c_write: i2c_reg->buffer[0] %d, temp_buff[0] %d\n",i2c_reg->buffer[0],temp_buff[0]);
		memset(temp_buff,0,sizeof(i2c_reg->count));
		ret = memcpy(temp_buff,i2c_reg->buffer,i2c_reg->count);//i2c_reg->count//TODO Titus
		LNT_TRACE(" After memcpy i2c_write: i2c_reg->buffer[0] %d, temp_buff[0] %d, ret %d\n",i2c_reg->buffer[0],temp_buff[0],ret);
		ret = i2c_reg->count;	//TODO Titus
	}



	gencp_mode = true;

	if (ret < 0)
	{
	pr_err("Copy from user is failed... %s:%d,  ret %d\n",__FILE__,__LINE__,ret);
	kfree(temp_buff);
	return ret;	
	}


	printk("LEONARDO CAMERA : i2c_write executed! %s : %d, i2c_reg->buffer %d \n",__func__,__LINE__,i2c_reg->buffer[0]);

	/* Backup the size of register which needs to be read after filling the address buffer */
	reg_size_bkp = i2c_reg->reg_size;

	printk(" i2c_write: i2c_reg->buffer[0] %d, temp_buff[0] %d\n",i2c_reg->buffer[0],temp_buff[0]);

// /****
	// Fill the address in buffer upto size of address want to write */
	for(i = --i2c_reg->reg_size; i >= 0 ; i--,j++)				
		i2c_w_buf[i] = ((i2c_reg->reg >> (8*j)) & 0xFF);	

	/* Append the data value in the same buffer  */
	for(i = reg_size_bkp; i < (i2c_reg->count + reg_size_bkp) ; i++,temp_buff++)
		i2c_w_buf[i] = *temp_buff;


	/* Debug purpose: Need to print the whole buffer and see correctly formed the address and data in i2c msg buffer */
	for(i = 0; i < total_size; i++)
		printk("%s : I2C write: message buffer i2c_w_buf[%d] %x\n", __func__,i,i2c_w_buf[i]);
// ****/
	

	//TODO Titus
//	total_size = 4;
	ret = i2c_master_send(leonardo_data.i2c_client, i2c_w_buf, total_size );

	printk("i2c_write : ret %d\n",ret);

	kfree(temp_buffer_bkp);

	return ret;
}

uint32_t Crc32(const void *pSrc, uint32_t nByteCount, const uint32_t * pnStartCrcValue)
{
    register uint32_t   nBit;
    register uint32_t   nCrc32 = UINT32_MAXIMUM;
    const uint8_t *     pSrc8  = (const uint8_t *) pSrc;

    // start value provided ?
    if ( NULL != pnStartCrcValue )
    {
        nCrc32 = *pnStartCrcValue;
    }

    while ( 0u != nByteCount-- )
    {
        nCrc32 ^= *(pSrc8++);

        nBit = 8;
        while ( 0u != nBit-- )
        {
            if ( nCrc32 & 0x01u )
            {
                nCrc32 = (nCrc32 >> 1) ^ CRC32MASKREV;
            }
            else
            {
                nCrc32 = (nCrc32 >> 1);
            }
        }
    }
	
	pr_info("CRC Checksum -> nCrc32 %x\n",nCrc32);
    return nCrc32;
}

void swapbytes(void *_object, size_t _size)
{
unsigned char *start, *end;
	 
	printk("swapbytes function\n");

	if(!is_bigendian())//If the board is little endian, swap the CCI registers as its big endian
	{
		printk("swapbytes, SoC is little endian!, need to swap the CCI registers\n");

		for ( start = (unsigned char *)_object, end = start + _size - 1; start < end; ++start, --end )
		{
		unsigned char swap = *start;
		*start = *end;
		*end = swap;
		}
	}
}

int read_cci_registers( void)
{
	ssize_t status = 0;
	struct v4l2_i2c i2c_reg = {};
	uint32_t    CRC = 0;
	uint32_t    Crc_byte_count = 0;
	uint32_t    Crc_byte_count2 = 0;

	pr_info("read_cci_registers  %s\n", __FILE__);

        i2c_reg.reg = cci_cmd_tbl[CCI_REGISTER_LAYOUT_VERSION].address; 
        i2c_reg.reg_size = 2;
        i2c_reg.count = sizeof(cci_reg);    //32 bit Offset address

        i2c_reg.buffer = (char *)&cci_reg;
	// Calculate CRC byte count 
	Crc_byte_count = sizeof(cci_reg) - 7 ;
	Crc_byte_count2 = sizeof(cci_reg) - (sizeof(cci_reg.Checksum) + (sizeof(cci_reg.Current_Mode) + (sizeof(cci_reg.Current_Mode) + sizeof(cci_reg.Soft_Reset))));

	printk("Crc_byte_count --> %d, Crc_byte_count2 %d, i2c_reg.count %d\n",Crc_byte_count,Crc_byte_count2,i2c_reg.count);
	
	// Read  CCI registers
	status = i2c_read(&i2c_reg);

	if ( status < 0 )
	{
	printk("%s : I2C read failed, status %d\n",__func__,status);
	return status;
	}

	// CRC calculation
	CRC = Crc32(&cci_reg, Crc_byte_count, 0);

	// Swap bytes if neccessary
	swapbytes(&cci_reg.CCI_Register_Layout_Version, 4);
	swapbytes(&cci_reg.Device_Capabilities, 8);
	swapbytes(&cci_reg.GCPRM_Address, 2);
	swapbytes(&cci_reg.BCRM_Address, 2);
	swapbytes(&cci_reg.Checksum, 4);

#if 0
//	printk("ntohs OUTPUT : cci_reg.CCI_Register_Layout_Version %x\n", ntohl(cci_reg.CCI_Register_Layout_Version));
	Swap4Bytes(cci_reg.CCI_Register_Layout_Version);
	Swap8Bytes(cci_reg.Device_Capabilities);
	Swap2Bytes(cci_reg.GCPRM_Address);
	Swap2Bytes(cci_reg.BCRM_Address);
	Swap4Bytes(cci_reg.Checksum);
#endif

	//Check the checksum of received with calculated.
	if (CRC != cci_reg.Checksum)
	{
		printk("Wrong CRC value: exit CRC calculated = 0x%x CRC received = 0x%x \n",CRC,cci_reg.Checksum);
		return LEONARDO_ERR_CRC_FAIL; 
	}
	else
	{
		printk("Success!, CRC value: CRC calculated = 0x%x CRC received = 0x%x \n",CRC,cci_reg.Checksum);
	}

	pr_info("LnT : CCI_Register_Layout_Version  = %x\n",cci_reg.CCI_Register_Layout_Version);
	pr_info("LnT : Device_Capabilities  = %llx\n",cci_reg.Device_Capabilities);
	pr_info("LnT : cci_reg.Device_GUID  = %s\n",cci_reg.Device_GUID);

        return LEONARDO_ERR_SUCCESS;
}

int read_gencp_registers( void)
{
	ssize_t status =0;
	struct v4l2_i2c i2c_reg = {};

	pr_info("%s:%d, cci_reg.GCPRM_Address 0x%x, sizeof(gencp_reg) %d \n",__func__,__LINE__,cci_reg.GCPRM_Address,sizeof(gencp_reg));

        i2c_reg.reg = cci_reg.GCPRM_Address+0x0000; 
        i2c_reg.reg_size = 2;
        i2c_reg.count = sizeof(gencp_reg);    //32 bit Offset address

        i2c_reg.buffer = (char *)&gencp_reg;
	// Read  CCI registers
	status = i2c_read(&i2c_reg);

	if ( status < 0 )
	{
		printk("%s : I2C read failed, status %d\n",__func__,status);
		return status;
	}

	pr_info("%s:%d, gencp_reg.GenCP_Out_Buffer_Address 0x%x \n",__func__,__LINE__,gencp_reg.GenCP_Out_Buffer_Address);
	pr_info("%s:%d, gencp_reg.GenCP_In_Buffer_Address 0x%x \n",__func__,__LINE__,gencp_reg.GenCP_In_Buffer_Address);
	pr_info("%s:%d, gencp_reg.GenCP_Out_Buffer_Size 0x%x \n",__func__,__LINE__,gencp_reg.GenCP_Out_Buffer_Size);
	pr_info("%s:%d, gencp_reg.GenCP_In_Buffer_Size 0x%x \n",__func__,__LINE__,gencp_reg.GenCP_In_Buffer_Size);

        return LEONARDO_ERR_SUCCESS;
}


/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{

	struct sensor_data *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;

	int ret = 0;
	u32 tgt_fps;	/* target frames per secound */

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:

		/* Check that the new frame rate is allowed. */
		if ((timeperframe->numerator == 0) ||
		    (timeperframe->denominator == 0)) {
			timeperframe->denominator = DEFAULT_FPS;
			timeperframe->numerator = 1;
		}

		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps > MAX_FPS) {
			timeperframe->denominator = MAX_FPS;
			timeperframe->numerator = 1;
		} else if (tgt_fps < MIN_FPS) {
			timeperframe->denominator = MIN_FPS;
			timeperframe->numerator = 1;
		}

		/* Actual frame rate we use */
		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		pr_err(" The camera frame rate is tgt_fps %d \n",tgt_fps);

		leonardo_init_mode(leonardo_mode_MAX);

		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode =
				(u32)a->parm.capture.capturemode;

		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_debug("   type is not " \
			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			a->type);
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{

	struct sensor_data *sensor = s->priv;

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		f->fmt.pix = sensor->pix;
		pr_debug("%s: %dx%d\n", __func__, sensor->pix.width, sensor->pix.height);
		break;

	case V4L2_BUF_TYPE_SENSOR:
		pr_debug("%s: left=%d, top=%d, %dx%d\n", __func__,
			sensor->spix.left, sensor->spix.top,
			sensor->spix.swidth, sensor->spix.sheight);
		f->fmt.spix = sensor->spix;
		break;

	case V4L2_BUF_TYPE_PRIVATE:
		break;

	default:
		f->fmt.pix = sensor->pix;
		break;
	}

	return 0;
}

/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{

	int ret = 0;
	struct v4l2_i2c i2c_reg = {};
	ssize_t status = 0;
	unsigned int reg = 0;
	int length = 0;
	unsigned int value_hex;

	struct sensor_data *sensor = s->priv;//TODO


	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		vc->value = leonardo_data.brightness;
		break;
	case V4L2_CID_HUE:
		vc->value = leonardo_data.hue;
		break;
	case V4L2_CID_CONTRAST:
		vc->value = leonardo_data.contrast;
		break;
	case V4L2_CID_SATURATION:
		vc->value = leonardo_data.saturation;
		break;
	case V4L2_CID_RED_BALANCE:
		vc->value = leonardo_data.red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		vc->value = leonardo_data.blue;
		break;
//LNT
	case V4L2_CID_EXPOSURE:
		LNT_TRACE("V4L2_CID_EXPOSURE, sensor->ae_mode %d\n",sensor->ae_mode);
//		vc->value = leonardo_data.ae_mode;
		vc->value = leonardo_data.expos;
		reg = EXPOSURE_TIME_64RW; 
		length = 4;
		break;

	case V4L2_CID_EXPOSURE_AUTO:
		LNT_TRACE("V4L2_CID_EXPOSURE_AUTO\n");
		vc->value = leonardo_data.expos_auto;
		reg = EXPOSURE_AUTO_8RW; 
		length = 1;
		break;

	case V4L2_CID_GAIN:
		LNT_TRACE("V4L2_CID_GAIN\n");
		vc->value = leonardo_data.gain;
		reg = GAIN_64RW; 
		length = 4;
		break;

	case V4L2_CID_AUTOGAIN:
		LNT_TRACE("V4L2_CID_AUTOGAIN\n");
		vc->value = leonardo_data.gain_auto;
		reg = GAIN_AUTO_8RW;
		length = 1;
		break;

	default:
		ret = -EINVAL;
	}


       	i2c_reg.reg = reg + cci_reg.BCRM_Address;
       	i2c_reg.reg_size = 2;
       	i2c_reg.count = length;
       	i2c_reg.buffer = &vc->value;
	status = i2c_read(&i2c_reg);


	swapbytes(&vc->value, i2c_reg.count);

	if (status < 0)
		return LEONARDO_ERR_I2C_READ_FAIL;

	LNT_TRACE("vc->value %d, value 0x%x \n",vc->value,vc->value);

	return ret;
}



static int ioctl_bcrm_i2cwrite_reg( struct v4l2_control *vc, unsigned int reg, int length)
{
	struct v4l2_int_device s;
        struct v4l2_i2c i2c_reg = {};
        ssize_t status;

	LNT_TRACE("reg %x, length %d, vc->value %d, vc->value 0x%x\n",reg,length,vc->value,vc->value);


	swapbytes(&vc->value,length);//TODO Titus

        i2c_reg.reg = reg;
        i2c_reg.reg_size = 2;
        i2c_reg.count = length;
//        i2c_reg.buffer = &vc->value;
        i2c_reg.buffer = (const char*) &vc->value;

//	memcpy(i2c_reg.buffer, &vc->value, sizeof(vc->value) );

	gencp_mode = false;//BCRM write
	LNT_TRACE("Before i2c_reg.reg %x, i2c_reg.count %d, vc->value %d\n",i2c_reg.reg,i2c_reg.count,vc->value);

//        status = bcrm_i2cwrite_reg(&i2c_reg);
	ioctl_gencam_i2cwrite_reg(&s, &i2c_reg);


	LNT_TRACE("After i2c_reg.reg %x, i2c_reg.count %d, vc->value %d\n",i2c_reg.reg,i2c_reg.count,vc->value);

	return status;
}



/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int retval = 0;
	unsigned int reg = 0;
	int length = 0;

	LNT_TRACE("\n");
		
	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		reg = EXPOSURE_TIME_64RW;
		length = 2;
		break;
	case V4L2_CID_AUTO_FOCUS_START:
		retval = trigger_auto_focus();
		return retval;
		break;
	case V4L2_CID_CONTRAST:
		reg = HUE_32RW;
		length = 2;
		break;
	case V4L2_CID_SATURATION:
		reg = SATURATION_32RW; 
		length = 2;
		break;
	case V4L2_CID_HUE:
		reg = HUE_32RW;
		length = 2;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		reg = WHITE_BALANCE_AUTO_8RW; 
		length = 2;
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		reg = SATURATION_32RW; 
		length = 2;
		break;
	case V4L2_CID_RED_BALANCE:
		reg = RED_BALANCE_RATIO_64RW; 
		length = 2;
		break;
	case V4L2_CID_BLUE_BALANCE:
		reg = BLUE_BALANCE_RATIO_64RW; 
		length = 2;
		break;
	case V4L2_CID_GAMMA:
		reg = GAMMA_64RW; 
		length = 2;
		break;
//LnT
	case V4L2_CID_EXPOSURE:
		LNT_TRACE("V4L2_CID_EXPOSURE, cci_reg.BCRM_Address 0x%x\n",cci_reg.BCRM_Address);
		reg = EXPOSURE_TIME_64RW; 
		length = 4;
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		LNT_TRACE("V4L2_CID_EXPOSURE_AUTO\n");
		reg = EXPOSURE_AUTO_8RW; 
		length = 1;
		break;
	case V4L2_CID_AUTOGAIN:
		LNT_TRACE("V4L2_CID_AUTOGAIN\n");
		reg = GAIN_AUTO_8RW;
		length = 1;
	        break;
	case V4L2_CID_GAIN:
		LNT_TRACE("V4L2_CID_GAIN\n");
		reg = GAIN_64RW; 
		length = 4;
		break;

	case V4L2_CID_HFLIP:
		break;
	case V4L2_CID_VFLIP:
		break;
	default:
		retval = -EPERM;
		return retval;
	}

	return ioctl_bcrm_i2cwrite_reg(vc, reg + cci_reg.BCRM_Address, length);
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	// TODO
	return 0;
}

/*!
 * ioctl_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					 struct v4l2_frmivalenum *fival)
{
	// TODO
	return -EINVAL;
}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name,
		"leonardo_mipi_camera");

	return 0;
}



//LnT
static int ioctl_queryctrl(struct v4l2_int_device *s, struct v4l2_queryctrl *qctrl)
{
	struct v4l2_i2c i2c_reg;
	u64 value = 0;
	u32 value_hex;
	char buf[4];
	int status;
	float float_val;

	LNT_TRACE("\n");

	switch(qctrl->id) {


	case V4L2_CID_EXPOSURE: {


		/* Reading the Current Exposure time */
	        i2c_reg.reg = cci_reg.BCRM_Address + EXPOSURE_TIME_64RW;
        	i2c_reg.reg_size = 2;//2 Bytes address size
        	i2c_reg.count = 4;//4Bytes
        	i2c_reg.buffer = &value;
	
		status = i2c_read(&i2c_reg);//TODO
		swapbytes(&value, 4);
		value_hex = *(unsigned int *)&value;
//		float_val = *(float *)&value;//TODO Build error
//		LNT_TRACE("Current Exposure value : 0x%lx, 0x%d, %f\n",value,value_hex,float_val);//TODO Build error
		LNT_TRACE("Current Exposure value : 0x%lx, 0x%d\n",value,value_hex);
		qctrl->default_value = value;



		/* Reading the Exposure minimum time */
		value = 0;
	        i2c_reg.reg = cci_reg.BCRM_Address + EXPOSURE_TIME_MIN_64R;
        	i2c_reg.reg_size = 2;//2 Bytes address size
        	i2c_reg.count = 4;//4Bytes
        	i2c_reg.buffer = &value;
	
		status = i2c_read(&i2c_reg);//TODO
		swapbytes(&value, 4);
		qctrl->minimum = value;
		LNT_TRACE("Exposure Minimum value : 0x%lx, qctrl->minimum : 0x%lx\n",value,qctrl->minimum);



		/* Reading the Exposure maximum time */
		value = 0;
        	i2c_reg.reg = cci_reg.BCRM_Address + EXPOSURE_TIME_MAX_64R;
		i2c_reg.reg_size = 2;//2 Bytes address size
        	i2c_reg.count = 4;//4Bytes
        	i2c_reg.buffer = &value;
	
		status = i2c_read(&i2c_reg);//TODO
		swapbytes(&value, 4);
		qctrl->maximum = value;
		LNT_TRACE("Exposure Maximum value : 0x%lx, qctrl->maximum : 0x%lx\n",value,qctrl->maximum);



		/* Reading the Exposure step increment */
		value = 0;
	        i2c_reg.reg = cci_reg.BCRM_Address + EXPOSURE_TIME_INCREMENT_64R;
        	i2c_reg.reg_size = 2;//2 Bytes address size
        	i2c_reg.count = 4;//4Bytes
        	i2c_reg.buffer = &value;
	
		status = i2c_read(&i2c_reg);//TODO
		swapbytes(&value, 4);
		qctrl->step = value;
		LNT_TRACE("Exposure Step value : 0x%lx, qctrl->step : 0x%lx\n",value,qctrl->step);


//		qctrl->type = V4L2_CTRL_TYPE_INTEGER64;//64bit value
		qctrl->type = V4L2_CTRL_TYPE_INTEGER;//32bit value

		break;
	}

	case V4L2_CID_EXPOSURE_AUTO: {


		/* Reading the Current Exposure Auto */
	        i2c_reg.reg = cci_reg.BCRM_Address + EXPOSURE_TIME_64RW;
        	i2c_reg.reg_size = 2;//2 Bytes address size
        	i2c_reg.count = 1;//1Byte
        	i2c_reg.buffer = &value;
	
		status = i2c_read(&i2c_reg);//TODO
		swapbytes(&value, 4);
		LNT_TRACE("Current Exposure value : 0x%lx\n",value);
		qctrl->default_value = value;



		/* Reading the Exposure minimum time */
		value = 0;
	        i2c_reg.reg = cci_reg.BCRM_Address + EXPOSURE_TIME_MIN_64R;
        	i2c_reg.reg_size = 2;//2 Bytes address size
        	i2c_reg.count = 4;//4Bytes
        	i2c_reg.buffer = &value;
	
		status = i2c_read(&i2c_reg);//TODO
		swapbytes(&value, 4);
		qctrl->minimum = value;
		LNT_TRACE("Exposure Minimum value : 0x%lx, qctrl->minimum : 0x%lx\n",value,qctrl->minimum);



		/* Reading the Exposure maximum time */
		value = 0;
        	i2c_reg.reg = cci_reg.BCRM_Address + EXPOSURE_TIME_MAX_64R;
		i2c_reg.reg_size = 2;//2 Bytes address size
        	i2c_reg.count = 4;//4Bytes
        	i2c_reg.buffer = &value;
	
		status = i2c_read(&i2c_reg);//TODO
		swapbytes(&value, 4);
		qctrl->maximum = value;
		LNT_TRACE("Exposure Maximum value : 0x%lx, qctrl->maximum : 0x%lx\n",value,qctrl->maximum);



		/* Reading the Exposure step increment */
		value = 0;
	        i2c_reg.reg = cci_reg.BCRM_Address + EXPOSURE_TIME_INCREMENT_64R;
        	i2c_reg.reg_size = 2;//2 Bytes address size
        	i2c_reg.count = 4;//4Bytes
        	i2c_reg.buffer = &value;
	
		status = i2c_read(&i2c_reg);//TODO
		swapbytes(&value, 4);
		qctrl->step = value;
		LNT_TRACE("Exposure Step value : 0x%lx, qctrl->step : 0x%lx\n",value,qctrl->step);


		qctrl->type = V4L2_CTRL_TYPE_INTEGER;//32bit value

		break;
	}


	case V4L2_CID_GAIN: {

		/* Reading the Current Gain value */
	        i2c_reg.reg = cci_reg.BCRM_Address + GAIN_64RW;
        	i2c_reg.reg_size = 2;//2 Bytes address size
        	i2c_reg.count = 4;//4Bytes
        	i2c_reg.buffer = &value;
	
		status = i2c_read(&i2c_reg);//TODO
		swapbytes(&value, 4);
//		float_val = *(float *)&value;//TODO Build error
//		LNT_TRACE("Current Gain value : 0x%lx, %f\n",value,float_val);
		LNT_TRACE("Current Gain value : 0x%lx\n",value);
		qctrl->default_value = value;



		/* Reading the Gain minimum time */
		value = 0;
	        i2c_reg.reg = cci_reg.BCRM_Address + GAIN_MINIMUM_64R;
        	i2c_reg.reg_size = 2;//2 Bytes address size
        	i2c_reg.count = 4;//4Bytes
        	i2c_reg.buffer = &value;
	
		status = i2c_read(&i2c_reg);//TODO
		swapbytes(&value, 4);
//		value >>= 32;//TODO Its required if we use 64bits
//		LNT_TRACE("After shift value : %llx, %lx\n",value,value);
		qctrl->minimum = value;
		LNT_TRACE("qctrl->minimum : 0x%lx, Minimum Gain value 0x%lx\n",qctrl->minimum,value);


		/* Reading the Gain maximum time */
		value = 0;
        	i2c_reg.reg = cci_reg.BCRM_Address + GAIN_MAXIMUM_64R;
		i2c_reg.reg_size = 2;//2 Bytes address size
        	i2c_reg.count = 4;//4Bytes
        	i2c_reg.buffer = &value;
	
		status = i2c_read(&i2c_reg);
		swapbytes(&value, 4);
		qctrl->maximum = value;
		LNT_TRACE("qctrl->maximum : %lx, Maximum Gain value 0x%lx\n",qctrl->maximum,value);


		/* Reading the Gain step increment */
		value = 0;
	        i2c_reg.reg = cci_reg.BCRM_Address + GAIN_INCREMENT_64R;
        	i2c_reg.reg_size = 2;//2 Bytes address size
        	i2c_reg.count = 4;//4Bytes
        	i2c_reg.buffer = &value;
	
		status = i2c_read(&i2c_reg);//TODO
		swapbytes(&value, 4);
		qctrl->step = value;
		LNT_TRACE("qctrl->step : %lx, Gain Step value 0x%lx \n",qctrl->step,value);


//		qctrl->type = V4L2_CTRL_TYPE_INTEGER64;//64bit value
		qctrl->type = V4L2_CTRL_TYPE_INTEGER;//32bit value

		break;
	}


	default:
		LNT_TRACE("Default case or not supported\n");
		break;
	}



	if(status < 0) {
	printk("%s:%d I2C Read failed, status = %d Bytes read = %d\n",__func__,__LINE__,status,i2c_reg.count);		
	return -1;
	}
	else
	printk("status = %d Bytes read = %d\n",status,i2c_reg.count);

	return 0;
}


/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{

	return 0;
}

/*!
 * ioctl_enum_fmt_cap - V4L2 sensor interface handler for VIDIOC_ENUM_FMT
 * @s: pointer to standard V4L2 device structure
 * @fmt: pointer to standard V4L2 fmt description structure
 *
 * Return 0.
 */
static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
			      struct v4l2_fmtdesc *fmt)
{
	LNT_TRACE("\n");

	if (fmt->index > leonardo_mode_MAX)
		return -EINVAL;

//	fmt->pixelformat = leonardo_data.pix.pixelformat;
	fmt->pixelformat = leonardo_formats[fmt->index].pixelformat;
	strcpy(fmt->description, leonardo_formats[fmt->index].description);
	fmt->flags = leonardo_formats[fmt->index].flags;
//	strcpy(fmt->description, "RGB24 Mode");
	return 0;
}

static int leonardo_init_mode(enum leonardo_mode mode)
{
	int retval = 0;
	void *mipi_csi2_info;
	u32 mipi_reg = 0;

	printk("leonardo_init_mode called!\n");

	mipi_csi2_info = mipi_csi2_get_info();

	/* initial mipi dphy */
	if (mipi_csi2_info) {
		if (!mipi_csi2_get_status(mipi_csi2_info))
			mipi_csi2_enable(mipi_csi2_info);

		if (mipi_csi2_get_status(mipi_csi2_info)) {
			mipi_csi2_set_lanes(mipi_csi2_info, 1);

			/*Only reset MIPI CSI2 HW *once* at sensor initialize*/
				if (mode == leonardo_mode_INIT)
				mipi_csi2_reset_with_dphy_freq(mipi_csi2_info, 0x44);

//				if (mode == leonardo_mode_SVGA_800_600)
//				mipi_csi2_set_datatype(mipi_csi2_info, leonardo_formats[0].flags);//RGB888 80x600


			if (leonardo_data.pix.pixelformat == V4L2_PIX_FMT_UYVY)
				mipi_csi2_set_datatype(mipi_csi2_info, MIPI_DT_YUV422);

			else if (leonardo_data.pix.pixelformat == V4L2_PIX_FMT_RGB565)
				mipi_csi2_set_datatype(mipi_csi2_info, MIPI_DT_RGB565);

			else if (leonardo_data.pix.pixelformat == V4L2_PIX_FMT_RGB24)
				{
				pr_debug("leonardo_data.pix.pixelformat == V4L2_PIX_FMT_RGB24, leonardo_formats[0].flags 0x%x\n",leonardo_formats[0].flags);
//				mipi_csi2_set_datatype(mipi_csi2_info, MIPI_DT_RGB888);
				mipi_csi2_set_datatype(mipi_csi2_info, leonardo_formats[0].flags);
				}
			else if (leonardo_data.pix.pixelformat == V4L2_PIX_FMT_CUSTOM)
				{
				pr_debug("leonardo_data.pix.pixelformat == V4L2_PIX_FMT_CUSTOM\n");
				mipi_csi2_set_datatype(mipi_csi2_info, MIPI_DT_CUSTOM);
				}
			else
				pr_err("currently this sensor format can not be supported!\n");
		} else {
			pr_err("Can not enable mipi csi2 driver!\n");
			return -1;
		}
	} else {
		printk(KERN_ERR "Fail to get mipi_csi2_info!\n");
		return -1;
	}

//	set_virtual_channel(leonardo_data.csi);

	/* add delay to wait for sensor stable */
	if (mode != leonardo_mode_INIT)
	{
	pr_debug("Please connect the FPGA and load the CPU0 and CPU1 code!\n");
	//msleep(1000 * 5);//wait for some time the FPGA loading
	}

	if (mipi_csi2_info) {
		unsigned int i;

		i = 0;

		/* wait for mipi sensor ready */
		mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);

		/* 0x230 still pass this check but means mipi clock not ready */
		while ((mipi_reg == 0x200) && (i < 10)) {
			mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
			pr_debug("%s: mipi_csi2_dphy_status return 0x%x!\n", __func__, mipi_reg);
			i++;
			msleep(10);
		}

		if (i >= 10) {
			pr_err("mipi csi2 can not receive sensor clk!\n");
			return -1;
		}

		i = 0;

		/* wait for mipi stable */
		mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);
		while ((mipi_reg != 0x0) && (i < 10)) {
			mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);
			i++;
			msleep(10);
		}

		if (i >= 10) {
			pr_err("mipi csi2 can not reveive data correctly!\n");
			return -1;
		}
	}
	return retval;
}


/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{

	printk("ioctl_dev_init called!\n");

	int ret;

	struct sensor_data *sensor = s->priv;
	u32 tgt_xclk;	/* target xclk */
	u32 tgt_fps;	/* target frames per secound */

	void *mipi_csi2_info;

	leonardo_data.on = true;

	/* mclk */
	tgt_xclk = leonardo_data.mclk;
	tgt_xclk = min(tgt_xclk, (u32)LEONARDO_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)LEONARDO_XCLK_MIN);
	leonardo_data.mclk = tgt_xclk;

	printk("   Setting mclk to %d MHz\n", tgt_xclk / 1000000);

	/* Default camera frame rate is set in probe */
	tgt_fps = sensor->streamcap.timeperframe.denominator /
		  sensor->streamcap.timeperframe.numerator;

	mipi_csi2_info = mipi_csi2_get_info();

	/* enable mipi csi2 */
	if (mipi_csi2_info)
		mipi_csi2_enable(mipi_csi2_info);
	else {
		printk(KERN_ERR " %s() in %s: Fail to get mipi_csi2_info! \n",
		       __func__, __FILE__);
		return -EPERM;
	}

	ret = leonardo_init_mode(leonardo_mode_INIT);

	return ret;
}

/*!
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the device when slave detaches to the master.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{

	printk("ioctl_dev_exit called!\n");

	void *mipi_csi2_info;

	mipi_csi2_info = mipi_csi2_get_info();

	/* disable mipi csi2 */
	if (mipi_csi2_info)
		if (mipi_csi2_get_status(mipi_csi2_info))
			mipi_csi2_disable(mipi_csi2_info);
	return 0;
}

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{

	printk("ioctl_g_ifparm called!\n");

	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	memset(p, 0, sizeof(*p));
	p->u.bt656.clock_curr = leonardo_data.mclk;
	pr_debug("  clock_curr=mclk=%d\n", leonardo_data.mclk);
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.clock_min = LEONARDO_XCLK_MIN;
	p->u.bt656.clock_max = LEONARDO_XCLK_MAX;
	p->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync */

	return 0;
}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc leonardo_ioctl_desc[] = {
	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func *) ioctl_dev_init},
	{vidioc_int_dev_exit_num, ioctl_dev_exit},
	{vidioc_int_init_num, (v4l2_int_ioctl_func *) ioctl_init},

	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func *) ioctl_g_ifparm},

	{vidioc_int_enum_fmt_cap_num,
				(v4l2_int_ioctl_func *) ioctl_enum_fmt_cap},
	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_g_fmt_cap},

	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *) ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *) ioctl_s_parm},

	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *) ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *) ioctl_s_ctrl},
	{vidioc_int_enum_framesizes_num,
				(v4l2_int_ioctl_func *) ioctl_enum_framesizes},
	{vidioc_int_enum_frameintervals_num,
			(v4l2_int_ioctl_func *) ioctl_enum_frameintervals},
	{vidioc_int_g_chip_ident_num,
				(v4l2_int_ioctl_func *) ioctl_g_chip_ident},

//LnT
	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func *)ioctl_queryctrl},

/* LnT New IOCTL for GenCP Mode*/
	{vidioc_int_gencam_i2cread_reg_num,
				(v4l2_int_ioctl_func *) ioctl_gencam_i2cread_reg},
	{vidioc_int_gencam_i2cwrite_reg_num,
				(v4l2_int_ioctl_func *) ioctl_gencam_i2cwrite_reg},

};

static struct v4l2_int_slave leonardo_slave = {
	.ioctls = leonardo_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(leonardo_ioctl_desc),
};

static struct v4l2_int_device leonardo_int_device = {
	.module = THIS_MODULE,
	.name = "leonardo_mipi",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &leonardo_slave,
	},
};

static ssize_t show_reg(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u8 val;
	pr_info("show_reg\n");
	leonardo_read_reg(leonardo_data.last_reg, &val);
	return sprintf(buf, "leonardo[0x%04x]=0x%02x\n",leonardo_data.last_reg,val);
}

static ssize_t set_reg(struct device *dev,
			struct device_attribute *attr,
		       const char *buf, size_t count)
{
	pr_info("set_reg\n");
	int regnum, value;
	int num_parsed = sscanf(buf, "%04x=%02x", &regnum, &value);
	if (1 <= num_parsed) {
		if (0xffff < (unsigned)regnum){
			pr_err("%s:invalid regnum %x\n", __func__, regnum);
			return 0;
		}
		leonardo_data.last_reg = regnum;
	}
	if (2 == num_parsed) {
		if (0xff < (unsigned)value) {
			pr_err("%s:invalid value %x\n", __func__, value);
			return 0;
		}
		leonardo_write_reg(leonardo_data.last_reg, value);
	}
	return count;
}

static ssize_t get_CCI_Register_Layout_Version(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%d",cci_reg.CCI_Register_Layout_Version);
}

static ssize_t get_Device_Capabilities(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%llu",cci_reg.Device_Capabilities);
}

static ssize_t get_Device_GUID(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%s",cci_reg.Device_GUID);
}

static ssize_t get_manufacturer_name(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%s",cci_reg.Manufacturer_Name);
}

static ssize_t get_Model_Name(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%s",cci_reg.Model_Name); 
}

static ssize_t get_Family_Name(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%s",cci_reg.Family_Name);
}

static ssize_t get_Device_Version(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%s",cci_reg.Device_Version);
}

static ssize_t get_Manufacturer_Info(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%s",cci_reg.Manufacturer_Info);
}

static ssize_t get_Serial_Number(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%s",cci_reg.Serial_Number);
}


static ssize_t get_User_Defined_Name(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%s",cci_reg.User_Defined_Name);
}

static ssize_t get_leonardo_i2c_clock(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%d",leonardo_i2c_clock);
}

static ssize_t get_GenCP_Out_Buffer_Size(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%d",gencp_reg.GenCP_Out_Buffer_Size);
}

static ssize_t get_GenCP_In_Buffer_Size(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%d",gencp_reg.GenCP_In_Buffer_Size);
}

static ssize_t mipi_csi_phy_status_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u32 mipi_reg = 0;
	char string[60];
	void *mipi_csi2_info;

	mipi_csi2_info = mipi_csi2_get_info();
	mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
	sprintf(string, "MIPI_CSI_PHY_STATE: 0x%x\n", mipi_reg);

	strcpy(buf, string);
	return strlen(buf);
}

/*
unsigned int mipi_csi2_read_lnt(struct mipi_csi2_info *info,unsigned offset)
{
	return readl(info->mipi_csi2_base + offset);
}
*/

static ssize_t mipi_csi_err1(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u32 mipi_reg = 0;
	char string[60];
	void *mipi_csi2_info;

	mipi_csi2_info = mipi_csi2_get_info();
	mipi_reg = mipi_csi2_read(mipi_csi2_info, MIPI_CSI2_ERR1);
	sprintf(string, "MIPI_CSI_ERR1: 0x%x\n", mipi_reg);

	strcpy(buf, string);
	return strlen(buf);
}


static ssize_t mipi_csi_err2(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u32 mipi_reg = 0;
	char string[60];
	void *mipi_csi2_info;

	mipi_csi2_info = mipi_csi2_get_info();
	mipi_reg = mipi_csi2_read(mipi_csi2_info, MIPI_CSI2_ERR2);
	sprintf(string, "MIPI_CSI_ERR2: 0x%x\n", mipi_reg);

	strcpy(buf, string);
	return strlen(buf);
}


static ssize_t leonardo_video_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
//        return sprintf(buf, "%s",leonardo_camera_details());
        return sprintf(buf, "%s\n","1");
}

static ssize_t leonardo_virtual_channel(struct device *dev,
		struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%d\n",virtual_channel);
}

static ssize_t leonardo_lane_count(struct device *dev,
		struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%d\n",lane_count);
}

static ssize_t leonardo_is_type31(struct device *dev,
		struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%d\n",is_type31);
}


static DEVICE_ATTR(lane_count, (S_IRUGO), leonardo_lane_count, NULL);
static DEVICE_ATTR(virtual_channel, (S_IRUGO), leonardo_virtual_channel, NULL);
static DEVICE_ATTR(is_type31, (S_IRUGO), leonardo_is_type31, NULL);

static DEVICE_ATTR(leonardo_video, (S_IRUGO), leonardo_video_read, NULL);
static DEVICE_ATTR(MIPI_CSI_PHY_STATE, (S_IRUGO), mipi_csi_phy_status_read, NULL);
static DEVICE_ATTR(MIPI_CSI_ERR1, (S_IRUGO), mipi_csi_err1, NULL);
static DEVICE_ATTR(MIPI_CSI_ERR2, (S_IRUGO), mipi_csi_err2, NULL);
static DEVICE_ATTR(leonardo_reg, S_IRUGO|S_IWUGO, show_reg, set_reg);
static DEVICE_ATTR(CCI_Register_Layout_Version, S_IRUGO, get_CCI_Register_Layout_Version, NULL);
static DEVICE_ATTR(Device_Capabilities, S_IRUGO, get_Device_Capabilities, NULL);
static DEVICE_ATTR(Device_GUID, S_IRUGO, get_Device_GUID, NULL);
static DEVICE_ATTR(Manufacturer_Name, S_IRUGO, get_manufacturer_name, NULL);
static DEVICE_ATTR(Model_Name, S_IRUGO, get_Model_Name, NULL);
static DEVICE_ATTR(Family_Name, S_IRUGO, get_Family_Name, NULL);
static DEVICE_ATTR(Device_Version, S_IRUGO, get_Device_Version, NULL);
static DEVICE_ATTR(Manufacturer_Info, S_IRUGO, get_Manufacturer_Info, NULL);
static DEVICE_ATTR(Serial_Number, S_IRUGO, get_Serial_Number, NULL);
static DEVICE_ATTR(User_Defined_Name, S_IRUGO, get_User_Defined_Name, NULL);
static DEVICE_ATTR(leonardo_i2c_clock, S_IRUGO, get_leonardo_i2c_clock, NULL);
static DEVICE_ATTR(GenCP_In_Buffer_Size, S_IRUGO, get_GenCP_In_Buffer_Size, NULL);
static DEVICE_ATTR(GenCP_Out_Buffer_Size, S_IRUGO, get_GenCP_Out_Buffer_Size, NULL);

/*!
 * I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int leonardo_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{

	printk(KERN_INFO "leonardo_probe called! %s: %d\n",__func__, __LINE__);

	struct device *dev = &client->dev;
	int retval;

	struct sensor_data *sensor = &leonardo_data;

	/* Set initial values for the sensor struct. */
	memset(&leonardo_data, 0, sizeof(leonardo_data));

	sensor->mipi_camera = 1;
	leonardo_data.sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(leonardo_data.sensor_clk)) {
		/* assuming clock enabled by default */
		leonardo_data.sensor_clk = NULL;
		dev_err(dev, "clock-frequency missing or invalid\n");
		return PTR_ERR(leonardo_data.sensor_clk);
	}


	retval = of_property_read_u32(dev->of_node, "leo_i2c_clk",
                                        &(leonardo_i2c_clock));
	dev_err(dev, "leonardo_i2c_clock =  %d\n",leonardo_i2c_clock);
        if (retval) {
                dev_err(dev, "leonardo_i2c_clock missing or invalid\n");
                return retval;
        }

	retval = of_property_read_u32(dev->of_node, "mclk",
					&(leonardo_data.mclk));
	if (retval) {
		dev_err(dev, "mclk missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "mclk_source",
					(u32 *) &(leonardo_data.mclk_source));
	if (retval) {
		dev_err(dev, "mclk_source missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "ipu_id",
					&sensor->ipu_id);
	if (retval) {
		dev_err(dev, "ipu_id missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "csi_id",
					&(leonardo_data.csi));
	if (retval) {
		dev_err(dev, "csi id missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "lanes",
					&(lane_count));
	if (retval) {
		dev_err(dev, "lanes missing or invalid\n");
		return retval;
	}

	clk_prepare_enable(leonardo_data.sensor_clk);



        leonardo_data.io_init = leonardo_reset;
        leonardo_data.i2c_client = client;

//        leonardo_data.pix.pixelformat = V4L2_PIX_FMT_UYVY;
//        leonardo_data.pix.pixelformat = V4L2_PIX_FMT_RGB565;
//        leonardo_data.pix.pixelformat = V4L2_PIX_FMT_RGB24;//RGB888
//        leonardo_data.pix.pixelformat = V4L2_PIX_FMT_CUSTOM;//Type 31 format


        leonardo_data.pix.pixelformat = leonardo_formats[0].pixelformat;//RGB24


        leonardo_data.pix.width = 800;
        leonardo_data.pix.height = 600;
        leonardo_data.streamcap.capability = V4L2_MODE_HIGHQUALITY |
                                           V4L2_CAP_TIMEPERFRAME;
        leonardo_data.streamcap.capturemode = 0 ;
        leonardo_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
        leonardo_data.streamcap.timeperframe.numerator = 1;


	sensor->virtual_channel = sensor->csi | (sensor->ipu_id << 1);
        leonardo_int_device.priv = &leonardo_data;

	virtual_channel = sensor->virtual_channel;

	if(leonardo_data.pix.pixelformat == V4L2_PIX_FMT_CUSTOM)
	is_type31 = 1;
	else
	is_type31 = 0;

	printk("CSI ID %d, IPU ID %d\n", sensor->csi, sensor->ipu_id);
	printk("Virtual channel %d, %d\n",sensor->virtual_channel, leonardo_data.virtual_channel );

	retval = read_cci_registers();

	if ( retval < 0 ) // TODO Change to != in condition 
	{
		printk("Failed to read_cci_registers %d, %s\n",retval,__func__);
		goto err;
	}
	else
	{
                printk("Success to read_cci_registers %d, %s\n",retval,__func__);
	}

	retval = read_gencp_registers();
        if ( retval < 0 ) // TODO Change to != in condition 
	{
		printk("Failed to read_gencp_registers %d, %s\n",retval,__func__);
                goto err;
	}
        else
	{
                printk("Success to read_gencp_registers %d, %s\n",retval,__func__);
	}


	if( ( (__u16) cci_reg.CCI_Register_Layout_Version ) == CCI_REG_LAYOUT_MINOR_VER)
		pr_info("%s: Success, got the Minor version  %d, CCI_Register_Layout_Version %x\n",__func__, (__u16) cci_reg.CCI_Register_Layout_Version, cci_reg.CCI_Register_Layout_Version);
	else
		{
		pr_info("%s: Failed, Minor version didn't match. Minor version -> %d, CCI_Register_Layout_Version %x \n",__func__,(__u16) cci_reg.CCI_Register_Layout_Version, cci_reg.CCI_Register_Layout_Version);
		goto err;
		}

	if( (cci_reg.CCI_Register_Layout_Version >> 16) == CCI_REG_LAYOUT_MAJOR_VER)
		pr_info("%s: Success, got the Major version  %d, CCI_Register_Layout_Version %x\n",__func__,(cci_reg.CCI_Register_Layout_Version >> 16), cci_reg.CCI_Register_Layout_Version);
	else
		{
		pr_info("%s: Failed, Major version didn't match. Major version -> %d, CCI_Register_Layout_Version %x \n",__func__,(cci_reg.CCI_Register_Layout_Version >> 16), cci_reg.CCI_Register_Layout_Version);
		goto err;
		}

	retval = v4l2_int_device_register(&leonardo_int_device);

	if ( retval < 0)
	printk("Unable to register the device %d\n",retval);
	else
	printk("leonardo driver is registered to capture driver %d\n",retval);


	if (device_create_file(dev, &dev_attr_leonardo_reg))
		dev_err(dev, "%s: error creating leonardo_reg entry\n", __func__);
	if (device_create_file(dev, &dev_attr_CCI_Register_Layout_Version))
                dev_err(dev, "%s: error creating Manufacturer_Name  entry\n", __func__);
        if (device_create_file(dev, &dev_attr_Device_Capabilities))
                dev_err(dev, "%s: error creating Device_Capabilities  entry\n", __func__);
        if (device_create_file(dev, &dev_attr_Device_GUID))
                dev_err(dev, "%s: error creating Device_GUID entry\n", __func__);
        if (device_create_file(dev, &dev_attr_Manufacturer_Name))
                dev_err(dev, "%s: error creating Manufacturer_Name  entry\n", __func__);
        if (device_create_file(dev, &dev_attr_Model_Name))
                dev_err(dev, "%s: error creating Model_Name  entry\n", __func__);
        if (device_create_file(dev, &dev_attr_Family_Name))
                dev_err(dev, "%s: error creating Family_Name entry\n", __func__);
        if (device_create_file(dev, &dev_attr_Device_Version))
                dev_err(dev, "%s: error creating Device_Version  entry\n", __func__);
        if (device_create_file(dev, &dev_attr_Manufacturer_Info))
                dev_err(dev, "%s: error creating Manufacturer_Info entry\n", __func__);
        if (device_create_file(dev, &dev_attr_Serial_Number))
                dev_err(dev, "%s: error creating Serial_Number entry\n", __func__);
        if (device_create_file(dev, &dev_attr_User_Defined_Name))
                dev_err(dev, "%s: error creating Manufacturer_Info entry\n", __func__);
        if (device_create_file(dev, &dev_attr_leonardo_i2c_clock))
                dev_err(dev, "%s: error creating leonardo_i2c_clock entry\n", __func__);
        if (device_create_file(dev, &dev_attr_GenCP_In_Buffer_Size))
                dev_err(dev, "%s: error creating GenCP_Out_Buffer_Address entry\n", __func__);
        if (device_create_file(dev, &dev_attr_GenCP_Out_Buffer_Size))
                dev_err(dev, "%s: error creating GenCP_Out_Buffer_Size entry\n", __func__);
        if (device_create_file(dev, &dev_attr_lane_count))
                dev_err(dev, "%s: error creating lane count entry\n", __func__);
        if (device_create_file(dev, &dev_attr_virtual_channel))
                dev_err(dev, "%s: error creating Virtual Channel entry\n", __func__);
        if (device_create_file(dev, &dev_attr_is_type31))
                dev_err(dev, "%s: error creating is_type31 entry\n", __func__);



	/* LnT : for debug */
	retval = device_create_file(&client->dev, &dev_attr_MIPI_CSI_PHY_STATE);
	if (retval < 0)
		pr_err("%s : could not create sys node for MIPI_CSI_PHY_STATE\n", __func__);

	retval = device_create_file(&client->dev, &dev_attr_MIPI_CSI_ERR1);
	if (retval < 0)
		pr_err("%s : could not create sys node for MIPI_CSI_ERR1\n", __func__);

	retval = device_create_file(&client->dev, &dev_attr_MIPI_CSI_ERR2);
	if (retval < 0)
		pr_err("%s : could not create sys node for MIPI_CSI_ERR2\n", __func__);

	retval = device_create_file(&client->dev, &dev_attr_leonardo_video);
	if (retval < 0)
		pr_err("%s : could not create sys node for leonardo_video\n", __func__);

	pr_info("camera leonardo_mipi is found\n");
	return retval;

err:
	pr_info("camera leonardo_mipi is not found\n");
	return retval;
}

/*!
 *  leonardo I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int leonardo_remove(struct i2c_client *client)
{
	v4l2_int_device_unregister(&leonardo_int_device);

	return 0;
}

/*!
 * leonardo init function
 * Called by insmod leonardo_camera.ko.
 *
 * @return  Error code indicating success or failure 
 */
static __init int leonardo_init(void)
{
	u8 err;
	err = i2c_add_driver(&leonardo_i2c_driver);
	printk("leonardo_init called! %s: %d, ret %d\n",__func__, __LINE__,err);

	if (err != 0)
		pr_err("%s:driver registration failed, error=%d\n",
			__func__, err);

	return err;
}

/*!
 * leonardo cleanup function
 * Called on rmmod leonardo_camera.ko
 *
 * @return  Error code indicating success or failure
 */
static void __exit leonardo_clean(void)
{
	i2c_del_driver(&leonardo_i2c_driver);
}

module_init(leonardo_init);
module_exit(leonardo_clean);

MODULE_AUTHOR("Allied Vision Inc.");
MODULE_DESCRIPTION("LEONARDO MIPI Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
