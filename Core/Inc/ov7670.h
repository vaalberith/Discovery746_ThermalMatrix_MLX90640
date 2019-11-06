#ifndef __OV7670_H
#define __OV7670_H

#include "stm32f7xx_hal.h"
//#include "video.h"

#define OV7670_I2C_ADDRESS              0x42

/* OV7670 Registers definition */
#define OV7670_AGC        0x00
#define OV7670_BLUE       0x01
#define OV7670_RED        0x02
#define OV7670_VREF       0x03  
#define OV7670_COM1       0x04  
#define OV7670_BAVE       0x05
#define OV7670_GbAVE      0x06
#define OV7670_AECHH      0x07
#define OV7670_RAVE       0x08
#define OV7670_COM2       0x09  
#define OV7670_PID        0x0A  
#define OV7670_VER        0x0B  
#define OV7670_COM3       0x0C 
#define OV7670_COM4       0x0D  
#define OV7670_COM5       0x0E  
#define OV7670_COM6       0x0F  
#define OV7670_AECH       0x10
#define OV7670_CLKRC      0x11 
#define OV7670_COM7       0x12 
#define OV7670_COM8       0x13 
#define OV7670_COM9       0x14  
#define OV7670_COM10      0x15 
//#define OV7670_RSVD       0x16 
#define OV7670_HSTART     0x17 
#define OV7670_HSTOP      0x18 
#define OV7670_VSTART     0x19 
#define OV7670_VSTOP      0x1A 
#define OV7670_PSHFT      0x1B
#define OV7670_MIDH       0x1C 
#define OV7670_MIDL       0x1D 
#define OV7670_MVFP       0x1E
#define OV7670_LAEC		  0x1F
#define OV7670_ADCCTR0    0x20
#define OV7670_ADCCTR1    0x21
#define OV7670_ADCCTR2    0x22
#define OV7670_ADCCTR3    0x23
#define OV7670_AEW        0x24
#define OV7670_AEB        0x25
#define OV7670_VPT        0x26
#define OV7670_BBIAS      0x27
#define OV7670_GbBIAS     0x28
//#define OV7670_RSVD    	  0x29
#define OV7670_EXHCH      0x2A 
#define OV7670_EXHCL      0x2B 
#define OV7670_RBIAS      0x2C
#define OV7670_ADVFL      0x2D
#define OV7670_ADVFH      0x2E
#define OV7670_YAVE       0x2F
#define OV7670_HSYST      0x30
#define OV7670_HSYEN      0x31
#define OV7670_HREF       0x32  
#define OV7670_CHLF       0x33
#define OV7670_ARBLM      0x34
//#define OV7670_RSVD       0x35
//#define OV7670_RSVD       0x36
#define OV7670_ADC        0x37
#define OV7670_ACOM       0x38
#define OV7670_OFON       0x39
#define OV7670_TSLB       0x3A
#define OV7670_COM11      0x3B  
#define OV7670_COM12      0x3C  
#define OV7670_COM13      0x3D  
#define OV7670_COM14      0x3E  
#define OV7670_EDGE       0x3F
#define OV7670_COM15      0x40 
#define OV7670_COM16      0x41 
#define OV7670_COM17      0x42 
#define OV7670_AWBC1      0x43 
#define OV7670_AWBC2      0x44 
#define OV7670_AWBC3      0x45 
#define OV7670_AWBC4      0x46 
#define OV7670_AWBC5      0x47 
#define OV7670_AWBC6      0x48 
//#define OV7670_RSVD       0x49 
//#define OV7670_RSVD       0x4A 
#define OV7670_REG4B      0x4B 
#define OV7670_DNSTH      0x4C 
//#define OV7670_RSVD       0x4D 
//#define OV7670_RSVD       0x4E 
#define OV7670_MTX1       0x4F 
#define OV7670_MTX2       0x50
#define OV7670_MTX3       0x51
#define OV7670_MTX4       0x52
#define OV7670_MTX5       0x53
#define OV7670_MTX6       0x54 
#define OV7670_BRTN       0x55
#define OV7670_CONTRAS    0x56
#define OV7670_CONTRASCENTER    0x57
#define OV7670_MTXS       0x58
//#define OV7670_RSVD       0x59
//#define OV7670_RSVD       0x5A
//#define OV7670_RSVD       0x5B
//#define OV7670_RSVD       0x5C
//#define OV7670_RSVD       0x5D
//#define OV7670_RSVD       0x5E
//#define OV7670_RSVD       0x5F
//#define OV7670_RSVD       0x60
//#define OV7670_RSVD       0x61
#define OV7670_LCC1       0x62
#define OV7670_LCC2       0x63
#define OV7670_LCC3       0x64
#define OV7670_LCC4       0x65
#define OV7670_LCC5       0x66
#define OV7670_MANU       0x67
#define OV7670_MANV       0x68
#define OV7670_GFIX       0x69
#define OV7670_GGAIN      0x6A
#define OV7670_DBLV       0x6B
#define OV7670_AWBCTR3    0x6C
#define OV7670_AWBCTR2    0x6D
#define OV7670_AWBCTR1    0x6E
#define OV7670_AWBCTR0    0x6F
#define OV7670_SCALING_XSC      0x70
#define OV7670_SCALING_YSC      0x71
#define OV7670_SCALING_DCWCTR   0x72 
#define OV7670_SCALING_PC       0x73 
#define OV7670_REG74      0x74
#define OV7670_REG75      0x75
#define OV7670_REG76      0x76
#define OV7670_REG77      0x77
//#define OV7670_RSVD       0x78
//#define OV7670_RSVD       0x79
#define OV7670_SLOP       0x7A
#define OV7670_GAM1       0x7B
#define OV7670_GAM2       0x7C
#define OV7670_GAM3       0x7D
#define OV7670_GAM4       0x7E
#define OV7670_GAM5       0x7F
#define OV7670_GAM6       0x80
#define OV7670_GAM7       0x81
#define OV7670_GAM8       0x82
#define OV7670_GAM9       0x83
#define OV7670_GAM10      0x84
#define OV7670_GAM11      0x85
#define OV7670_GAM12      0x86
#define OV7670_GAM13      0x87
#define OV7670_GAM14      0x88
#define OV7670_GAM15      0x89
//#define OV7670_RSVD       0x8A
//#define OV7670_RSVD       0x8B
#define OV7670_RGB444     0x8C
//#define OV7670_RSVD       0x8D
//#define OV7670_RSVD       0x8E
//#define OV7670_RSVD       0x8F
//#define OV7670_RSVD       0x90
//#define OV7670_RSVD       0x91
#define OV7670_DM_LNL     0x92
#define OV7670_DM_LNH     0x93
#define OV7670_LCC6       0x94
#define OV7670_LCC7       0x95
//#define OV7670_RSVD       0x96
//#define OV7670_RSVD       0x97
//#define OV7670_RSVD       0x98
//#define OV7670_RSVD       0x99
//#define OV7670_RSVD       0x9A
//#define OV7670_RSVD       0x9B
//#define OV7670_RSVD       0x9C
#define OV7670_BD50ST     0x9D
#define OV7670_BD60ST     0x9E
#define OV7670_HAECC1     0x9F
#define OV7670_HAECC2     0xA0
//#define OV7670_RSVD       0xA1
#define OV7670_SCALING_PCLK_DELAY       0xA2
//#define OV7670_RSVD       0xA3
#define OV7670_NT_CTRL    0xA4 
#define OV7670_BD50MAX    0xA5 
#define OV7670_HAECC3     0xA6
#define OV7670_HAECC4     0xA7
#define OV7670_HAECC5     0xA8
#define OV7670_HAECC6     0xA9
#define OV7670_HAECC7     0xAA
#define OV7670_BD60MAX    0xAB
#define OV7670_STR_OPT    0xAC
#define OV7670_STR_R      0xAD
#define OV7670_STR_G      0xAE
#define OV7670_STR_B      0xAF
//#define OV7670_RSVD       0xB0
#define OV7670_ABLC1      0xB1
//#define OV7670_RSVD       0xB2
#define OV7670_THL_DLT    0xB3
//#define OV7670_RSVD       0xB4
#define OV7670_THL_DLT_AREA    0xB5
//#define OV7670_RSVD       0xB6
//#define OV7670_RSVD       0xB7
//#define OV7670_RSVD       0xB8
//#define OV7670_RSVD       0xBC
//#define OV7670_RSVD       0xBD
#define OV7670_AD_CHB     0xBE
#define OV7670_AD_CHR     0xBF
#define OV7670_AD_CHGb    0xC0
#define OV7670_AD_CHGr    0xC1
//#define OV7670_RSVD       0xC2
//#define OV7670_RSVD       0xC3
//#define OV7670_RSVD       0xC4 
//#define OV7670_RSVD       0xC5
//#define OV7670_RSVD       0xC6
//#define OV7670_RSVD       0xC7 
//#define OV7670_RSVD       0xC8
#define OV7670_SATCTR     0xC9

/* Registers bit definition */
/* COM1 Register */
#define CCIR656_FORMAT  0x40
#define HREF_SKIP_0     0x00
#define HREF_SKIP_1     0x04
#define HREF_SKIP_3     0x08

/* COM2 Register */
#define SOFT_SLEEP_MODE  0x10	
#define ODCAP_1x         0x00	
#define ODCAP_2x         0x01	
#define ODCAP_3x         0x02	
#define ODCAP_4x         0x03
	
/* COM3 Register */
#define COLOR_BAR_OUTPUT         0x80
#define OUTPUT_MSB_LAS_SWAP      0x40
#define PIN_REMAP_RESETB_EXPST   0x08 
#define RGB565_FORMAT            0x00 
#define RGB_OUTPUT_AVERAGE       0x04 
#define SINGLE_FRAME             0x01

/* COM5 Register */
#define SLAM_MODE_ENABLE      0x40
#define EXPOSURE_NORMAL_MODE  0x01

/* COM7 Register */
#define SCCB_REG_RESET                       0x80
#define FORMAT_CTRL_15fpsVGA                 0x00
#define FORMAT_CTRL_30fpsVGA_NoVArioPixel    0x50
#define FORMAT_CTRL_30fpsVGA_VArioPixel      0x60
#define OUTPUT_FORMAT_RAWRGB                 0x00
#define OUTPUT_FORMAT_RAWRGB_DATA            0x00
#define OUTPUT_FORMAT_RAWRGB_INTERP          0x01
#define OUTPUT_FORMAT_YUV                    0x02
#define OUTPUT_FORMAT_RGB                    0x03

/* COM9 Register */
#define GAIN_2x         0x00	
#define GAIN_4x         0x10	
#define GAIN_8x         0x20	
#define GAIN_16x        0x30	
#define GAIN_32x        0x40	
#define GAIN_64x        0x50	
#define GAIN_128x       0x60	
#define DROP_VSYNC      0x04	
#define DROP_HREF       0x02
	
/* COM10 Register */
#define RESETb_REMAP_SLHS    0x80
#define HREF_CHANGE_HSYNC    0x40
#define PCLK_ON              0x00
#define PCLK_OFF             0x20
#define PCLK_POLARITY_REV    0x10
#define HREF_POLARITY_REV    0x08
#define RESET_ENDPOINT       0x04
#define VSYNC_NEG            0x02
#define HSYNC_NEG            0x01

/* TSLB Register */
#define PCLK_DELAY_0         0x00
#define PCLK_DELAY_2         0x40
#define PCLK_DELAY_4         0x80
#define PCLK_DELAY_6         0xC0
#define OUTPUT_BITWISE_REV   0x20
#define UV_NORMAL            0x00
#define UV_FIXED             0x10
#define YUV_SEQ_YUYV         0x00
#define YUV_SEQ_YVYU         0x02
#define YUV_SEQ_VYUY         0x04
#define YUV_SEQ_UYVY         0x06
#define BANDING_FREQ_50      0x02

#define RGB_NORMAL   0x00 
#define RGB_565      0x10 
#define RGB_555      0x30 

#define PLL_FACTOR	4








/* Registers */
#define REG_GAIN	0x00	/* Gain lower 8 bits (rest in vref) */
#define REG_BLUE	0x01	/* blue gain */
#define REG_RED		0x02	/* red gain */
#define REG_VREF	0x03	/* Pieces of GAIN, VSTART, VSTOP */
#define REG_COM1	0x04	/* Control 1 */
#define  COM1_CCIR656	  0x40  /* CCIR656 enable */
#define REG_BAVE	0x05	/* U/B Average level */
#define REG_GbAVE	0x06	/* Y/Gb Average level */
#define REG_AECHH	0x07	/* AEC MS 5 bits */
#define REG_RAVE	0x08	/* V/R Average level */
#define REG_COM2	0x09	/* Control 2 */
#define  COM2_SSLEEP	  0x10	/* Soft sleep mode */
#define REG_PID		0x0a	/* Product ID MSB */
#define REG_VER		0x0b	/* Product ID LSB */
#define REG_COM3	0x0c	/* Control 3 */
#define  COM3_SWAP	  0x40	  /* Byte swap */
#define  COM3_SCALEEN	  0x08	  /* Enable scaling */
#define  COM3_DCWEN	  0x04	  /* Enable downsamp/crop/window */
#define REG_COM4	0x0d	/* Control 4 */
#define REG_COM5	0x0e	/* All "reserved" */
#define REG_COM6	0x0f	/* Control 6 */
#define REG_AECH	0x10	/* More bits of AEC value */
#define REG_CLKRC	0x11	/* Clocl control */
#define   CLK_EXT	  0x40	  /* Use external clock directly */
#define   CLK_SCALE	  0x3f	  /* Mask for internal clock scale */
#define REG_COM7	0x12	/* Control 7 */
#define   COM7_RESET	  0x80	  /* Register reset */
#define   COM7_FMT_MASK	  0x38
#define   COM7_FMT_VGA	  0x00
#define	  COM7_FMT_CIF	  0x20	  /* CIF format */
#define   COM7_FMT_QVGA	  0x10	  /* QVGA format */
#define   COM7_FMT_QCIF	  0x08	  /* QCIF format */
#define	  COM7_RGB	  0x04	  /* bits 0 and 2 - RGB format */
#define	  COM7_YUV	  0x00	  /* YUV */
#define	  COM7_BAYER	  0x01	  /* Bayer format */
#define	  COM7_PBAYER	  0x05	  /* "Processed bayer" */
#define REG_COM8	0x13	/* Control 8 */
#define   COM8_FASTAEC	  0x80	  /* Enable fast AGC/AEC */
#define   COM8_AECSTEP	  0x40	  /* Unlimited AEC step size */
#define   COM8_BFILT	  0x20	  /* Band filter enable */
#define   COM8_AGC	  0x04	  /* Auto gain enable */
#define   COM8_AWB	  0x02	  /* White balance enable */
#define   COM8_AEC	  0x01	  /* Auto exposure enable */
#define REG_COM9	0x14	/* Control 9  - gain ceiling */
#define REG_COM10	0x15	/* Control 10 */
#define   COM10_HSYNC	  0x40	  /* HSYNC instead of HREF */
#define   COM10_PCLK_HB	  0x20	  /* Suppress PCLK on horiz blank */
#define   COM10_HREF_REV  0x08	  /* Reverse HREF */
#define   COM10_VS_LEAD	  0x04	  /* VSYNC on clock leading edge */
#define   COM10_VS_NEG	  0x02	  /* VSYNC negative */
#define   COM10_HS_NEG	  0x01	  /* HSYNC negative */
#define REG_HSTART	0x17	/* Horiz start high bits */
#define REG_HSTOP	0x18	/* Horiz stop high bits */
#define REG_VSTART	0x19	/* Vert start high bits */
#define REG_VSTOP	0x1a	/* Vert stop high bits */
#define REG_PSHFT	0x1b	/* Pixel delay after HREF */
#define REG_MIDH	0x1c	/* Manuf. ID high */
#define REG_MIDL	0x1d	/* Manuf. ID low */
#define REG_MVFP	0x1e	/* Mirror / vflip */
#define   MVFP_MIRROR	  0x20	  /* Mirror image */
#define   MVFP_FLIP	  0x10	  /* Vertical flip */

#define REG_AEW		0x24	/* AGC upper limit */
#define REG_AEB		0x25	/* AGC lower limit */
#define REG_VPT		0x26	/* AGC/AEC fast mode op region */
#define REG_HSYST	0x30	/* HSYNC rising edge delay */
#define REG_HSYEN	0x31	/* HSYNC falling edge delay */
#define REG_HREF	0x32	/* HREF pieces */
#define REG_TSLB	0x3a	/* lots of stuff */
#define   TSLB_YLAST	  0x04	  /* UYVY or VYUY - see com13 */
#define REG_COM11	0x3b	/* Control 11 */
#define   COM11_NIGHT	  0x80	  /* NIght mode enable */
#define   COM11_NMFR	  0x60	  /* Two bit NM frame rate */
#define   COM11_HZAUTO	  0x10	  /* Auto detect 50/60 Hz */
#define	  COM11_50HZ	  0x08	  /* Manual 50Hz select */
#define   COM11_EXP	  0x02
#define REG_COM12	0x3c	/* Control 12 */
#define   COM12_HREF	  0x80	  /* HREF always */
#define REG_COM13	0x3d	/* Control 13 */
#define   COM13_GAMMA	  0x80	  /* Gamma enable */
#define	  COM13_UVSAT	  0x40	  /* UV saturation auto adjustment */
#define   COM13_UVSWAP	  0x01	  /* V before U - w/TSLB */
#define REG_COM14	0x3e	/* Control 14 */
#define   COM14_DCWEN	  0x10	  /* DCW/PCLK-scale enable */
#define REG_EDGE	0x3f	/* Edge enhancement factor */
#define REG_COM15	0x40	/* Control 15 */
#define   COM15_R10F0	  0x00	  /* Data range 10 to F0 */
#define	  COM15_R01FE	  0x80	  /*            01 to FE */
#define   COM15_R00FF	  0xc0	  /*            00 to FF */
#define   COM15_RGB565	  0x10	  /* RGB565 output */
#define   COM15_RGB555	  0x30	  /* RGB555 output */
#define REG_COM16	0x41	/* Control 16 */
#define   COM16_AWBGAIN   0x08	  /* AWB gain enable */
#define REG_COM17	0x42	/* Control 17 */
#define   COM17_AECWIN	  0xc0	  /* AEC window - must match COM4 */
#define   COM17_CBAR	  0x08	  /* DSP Color bar */

/*
 * This matrix defines how the colors are generated, must be
 * tweaked to adjust hue and saturation.
 *
 * Order: v-red, v-green, v-blue, u-red, u-green, u-blue
 *
 * They are nine-bit signed quantities, with the sign bit
 * stored in 0x58.  Sign for v-red is bit 0, and up from there.
 */
#define	REG_CMATRIX_BASE 0x4f
#define   CMATRIX_LEN 6
#define REG_CMATRIX_SIGN 0x58


#define REG_BRIGHT	0x55	/* Brightness */
#define REG_CONTRAS	0x56	/* Contrast control */

#define REG_GFIX	0x69	/* Fix gain control */

#define REG_DBLV	0x6b	/* PLL control an debugging */
#define   DBLV_BYPASS	  0x00	  /* Bypass PLL */
#define   DBLV_X4	  0x01	  /* clock x4 */
#define   DBLV_X6	  0x10	  /* clock x6 */
#define   DBLV_X8	  0x11	  /* clock x8 */

#define REG_REG76	0x76	/* OV's name */
#define   R76_BLKPCOR	  0x80	  /* Black pixel correction enable */
#define   R76_WHTPCOR	  0x40	  /* White pixel correction enable */

#define REG_RGB444	0x8c	/* RGB 444 control */
#define   R444_ENABLE	  0x02	  /* Turn on RGB444, overrides 5x5 */
#define   R444_RGBX	  0x01	  /* Empty nibble at end */

#define REG_HAECC1	0x9f	/* Hist AEC/AGC control 1 */
#define REG_HAECC2	0xa0	/* Hist AEC/AGC control 2 */

#define REG_BD50MAX	0xa5	/* 50hz banding step limit */
#define REG_HAECC3	0xa6	/* Hist AEC/AGC control 3 */
#define REG_HAECC4	0xa7	/* Hist AEC/AGC control 4 */
#define REG_HAECC5	0xa8	/* Hist AEC/AGC control 5 */
#define REG_HAECC6	0xa9	/* Hist AEC/AGC control 6 */
#define REG_HAECC7	0xaa	/* Hist AEC/AGC control 7 */
#define REG_BD60MAX	0xab	/* 60hz banding step limit */

struct regval_list {
	uint8_t reg_num;
	uint8_t value;
};

uint8_t ov7670_Init(uint32_t video_adr, DCMI_HandleTypeDef *phdcmi);
uint8_t ov7670_Start(void);
uint8_t ov7670_Stop(void);  
uint8_t ov7670_LineProcessing(DMA_HandleTypeDef *hdma);
uint8_t ov7670_FrameProcessing(DCMI_HandleTypeDef *phdcmi);  

#endif //__OV7670_H
