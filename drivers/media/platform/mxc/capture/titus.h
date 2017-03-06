#ifndef _LEONARDO_MIPI_H
#define _LEONARDO_MIPI_H

/* General Control Registers */

#define BCRM_VERSION_REG_32R  0x0000
#define FEATURE_INQUIRY_REG_64R 0x0008

/* Streaming Control Registers */

#define SUPPORTED_CSI2_LANE_COUNTS_8R 0x0040
#define CSI2_LANE_COUNT_8RW 0x0044
#define CSI2_CLOCK_MIN_32R 0x0048
#define CSI2_CLOCK_MAX_32R 0x004C
#define CSI2_CLOCK_32RW 0x0050
#define BUFFER_SIZE_32R 0x0054


/* Acquisition Control Registers */

#define ACQUISITION_START_8RW  0x0080
#define ACQUISITION_STOP_8RW 0x0084
#define ACQUISITION_ABORT_8RW 0x0088
#define ACQUISITION_FRAME_RATE_64RW 0x0090

#define FRAME_START_TRIGGER_MODE_8RW 0x0098
#define FRAME_START_TRIGGER_SOURCE_8RW 0x009C
#define FRAME_START_TRIGGER_ACTIVATION_8RW 0x00A0
#define TRIGGER_SOFTWARE_8W 0x00A4

/* Image Format Control Registers */

#define IMG_WIDTH_32RW  0x0100
#define IMG_WIDTH_MIN_32R 0x0104
#define IMG_WIDTH_MAX_32R 0x0108
#define IMG_WIDTH_INCREMENT_32R 0x010C

#define IMG_HEIGHT_32RW 0x0110
#define IMG_HEIGHT_MIN_32R 0x0114
#define IMG_HEIGHT_MAX_32R 0x0118
#define IMG_HEIGHT_INCREMENT_32R 0x011C

#define IMG_OFFSET_X_32RW  0x0120
#define IMG_OFFSET_X_MIN_32R 0x0124
#define IMG_OFFSET_X_MAX_32R 0x0128
#define IMG_OFFSET_X_INCREMENT_32R 0x012C

#define IMG_OFFSET_Y_32RW  0x0130
#define IMG_OFFSET_Y_MIN_32R 0x0134
#define IMG_OFFSET_Y_MAX_32R 0x0138
#define IMG_OFFSET_Y_INCREMENT_32R 0x013C

#define IMG_MIPI_DATA_FORMAT_32RW 0x0140
#define IMG_AVAILABLE_MIPI_DATA_FORMATS_64R  0x0148

#define IMG_BAYER_PATTERN_INQUIRY_8R 0x150
#define IMG_BAYER_PATTERN_8RW 0x154

#define IMG_REVERSE_X_8RW 0x0158
#define IMG_REVERSE_Y_8RW 0x015C

/* Brightness Control Registers */

#define EXPOSURE_TIME_64RW 0x0180
#define EXPOSURE_TIME_MIN_64R 0x0188
#define EXPOSURE_TIME_MAX_64R 0x0190
#define EXPOSURE_TIME_INCREMENT_64R 0x0198
#define EXPOSURE_AUTO_8RW 0x01A0


#define INTENSITY_AUTO_PRECEDENCE_8RW 0x01A4
#define INTENSITY_AUTO_PRECEDENCE_VALUE_32RW 0x01A8
#define INTENSITY_AUTO_PRECEDENCE_MIN_32R 0x01AC
#define INTENSITY_AUTO_PRECEDENCE_MAX_32R 0x01B0
#define INTENSITY_AUTO_PRECEDENCE_INCREMENT_32R 0x01B4

#define BLACK_LEVEL_32RW 0x01B8
#define BLACK_LEVEL_MIN_32R 0x01BC
#define BLACK_LEVEL_MAX_32R 0x01C0
#define BLACK_LEVEL_INCREMENT_32R 0x01C4


#define GAIN_64RW 0x01C8
#define GAIN_MINIMUM_64R 0x01D0
#define GAIN_MAXIMUM_64R 0x01D8
#define GAIN_INCREMENT_64R 0x01E0
#define GAIN_AUTO_8RW 0x01E8


#define GAMMA_64RW 0x01F0
#define GAMMA_GAIN_MINIMUM_64R 0x01F8
#define GAMMA_GAIN_MAXIMUM_64R 0x0200
#define GAMMA_GAIN_INCREMENT_64R 0x0208

#define CONTRAST_ENABLE_8RW 0x0210
#define CONTRAST_VALUE_32RW 0x0214
#define CONTRAST_VALUE_MIN_32R 0x0218
#define CONTRAST_VALUE_MAX_32R 0x021C
#define CONTRAST_VALUE_INCREMENT_32R 0x0220

/* Color Management Registers */

#define SATURATION_32RW 0x0240
#define SATURATION_MIN_32R 0x0244
#define SATURATION_MAX_32R 0x0248
#define SATURATION_INCREMENT_32R 0x024C

#define HUE_32RW 0x0250
#define HUE_MIN_32R 0x0254
#define HUE_MAX_32R 0x0258
#define HUE_INCREMENT_32R 0x025C

#define ALL_BALANCE_RATIO_64RW 0x0260
#define ALL_BALANCE_RATION_MIN_64R 0x0268
#define ALL_BALANCE_RATION_MAX_64R 0x0270
#define ALL_BALANCE_RATIO_INCREMENT_64R 0x0278

#define RED_BALANCE_RATIO_64RW 0x0280
#define RED_BALANCE_RATION_MIN_64R 0x0288
#define RED_BALANCE_RATION_MAX_64R 0x0290
#define RED_BALANCE_RATIO_INCREMENT_64R 0x0298

#define GREEN_BALANCE_RATIO_64RW 0x02A0
#define GREEN_BALANCE_RATION_MIN_64R 0x02A8
#define GREEN_BALANCE_RATION_MAX_64R 0x02B0
#define GREEN_BALANCE_RATIO_INCREMENT_64R 0x02B8

#define BLUE_BALANCE_RATIO_64RW 0x02C0
#define BLUE_BALANCE_RATION_MIN_64R 0x02C8
#define BLUE_BALANCE_RATION_MAX_64R 0x02D0
#define BLUE_BALANCE_RATIO_INCREMENT_64R 0x02D8

#define WHITE_BALANCE_AUTO_8RW 0x02E0

/* Other Registers */
#define SHARPNESS_32RW 0x0300
#define SHARPNESS_MIN_32R 0x0304
#define SHARPNESS_MAX_32R 0x0308
#define SHARPNESS_INCREMENT_32R 0x030C


#define LEONARDO_DEBUG 0
#define LEONARDO_ERR_SUCCESS  0
#define LEONARDO_ERR_CRC_FAIL  -1
#define LEONARDO_ERR_I2C_READ_FAIL -2

#define CCI_REG_LAYOUT_MINOR_VER	0
#define CCI_REG_LAYOUT_MAJOR_VER	1

#define Swap2Bytes(val) \
	( val = ( (((val) >> 8) & 0x00FF) | (((val) << 8) & 0xFF00) ) )
	 
	// Swap 4 byte, 32 bit values:
	 
#define Swap4Bytes(val) \
	( val = ( (((val) >> 24) & 0x000000FF) | (((val) >>  8) & 0x0000FF00) | \
	   (((val) <<  8) & 0x00FF0000) | (((val) << 24) & 0xFF000000) ) )
 
	// Swap 8 byte, 64 bit values:
	 
#define Swap8Bytes(val) \
	( val = ( (((val) >> 56) & 0x00000000000000FF) | (((val) >> 40) & 0x000000000000FF00) | \
	   (((val) >> 24) & 0x0000000000FF0000) | (((val) >>  8) & 0x00000000FF000000) | \
	   (((val) <<  8) & 0x000000FF00000000) | (((val) << 24) & 0x0000FF0000000000) | \
	   (((val) << 40) & 0x00FF000000000000) | (((val) << 56) & 0xFF00000000000000) ) )

const int bsti = 1;  // Byte swap test integer
#define is_bigendian() ( (*(char*)&bsti) == 0 )

enum CCI_REG_INFO {
        CCI_REGISTER_LAYOUT_VERSION = 0,
	RESERVED4BIT,
        DEVICE_CAPABILITIES,
        GCPRM_ADDRESS,
	RESERVED2BIT,
        BCRM_ADDRESS,
	RESERVED2BIT_2,
        DEVICE_GUID,
        MANUFACTURER_NAME,
        MODEL_NAME,
        FAMILY_NAME,
        DEVICE_VERSION,
        MANUFACTURER_INFO,
        SERIAL_NUMBER,
        USER_DEFINED_NAME,
        CHECKSUM,
        CHANGE_MODE,
        CURRENT_MODE,
        SOFT_RESET,
        MAX_CMD = SOFT_RESET
};

typedef struct cci_cmd_t {
    __u8    command_index; /* diagnostc test name */
    const __u32 address; /* NULL for no alias name */
    __u32  byte_count;
} cci_cmd_t;

static cci_cmd_t cci_cmd_tbl[MAX_CMD] = {
        // command index                address 
        { CCI_REGISTER_LAYOUT_VERSION,  0x0000, 4 },
        { DEVICE_CAPABILITIES,          0x0008, 8 },
        { GCPRM_ADDRESS,                0x0010, 2 },
        { BCRM_ADDRESS,                 0x0014, 2 },
        { DEVICE_GUID,                  0x0018, 64 },
        { MANUFACTURER_NAME,            0x0058, 64 },
        { MODEL_NAME,                   0x0098, 64 },
        { FAMILY_NAME,                  0x00D8, 64 },
        { DEVICE_VERSION,               0x0118, 64 },
        { MANUFACTURER_INFO,            0x0158, 64 },
        { SERIAL_NUMBER,                0x0198, 64 },
        { USER_DEFINED_NAME,            0x01D8, 64 },
        { CHECKSUM,                     0x0218, 4 },
        { CHANGE_MODE,                  0x021C, 1 },
        { CURRENT_MODE,                 0x021D, 1 },
        { SOFT_RESET,                   0x021E, 1 },
};



struct   __attribute__((__packed__))  cci_reg_t {
        __u32   CCI_Register_Layout_Version;
        __u32   reserved_4bit;
        __u64   Device_Capabilities;
        __u16   GCPRM_Address;
        __u16   reserved_2bit;
        __u16   BCRM_Address;
        __u16   reserved_2bit_2;
        char    Device_GUID[64];
        char    Manufacturer_Name[64];
        char    Model_Name[64];
        char    Family_Name[64];
        char    Device_Version[64];
        char    Manufacturer_Info[64];
        char    Serial_Number[64];
        char    User_Defined_Name[64];
        __u32   Checksum;
        __u8    Change_Mode;
        __u8    Current_Mode;
        __u8    Soft_Reset;
} cci_reg;


struct   __attribute__((__packed__))  gencp_reg_t {
        __u16   GenCP_Out_Buffer_Address;
        __u16   reserved_2byte;
        __u16   GenCP_Out_Buffer_Size;
        __u16   reserved_2byte_1;
	__u16 	GenCP_In_Buffer_Address;
        __u16   reserved_2byte_2;
	__u16   GenCP_In_Buffer_Size;
} gencp_reg;


#endif
