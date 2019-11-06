#include "ov7670.h"

//static void ConvertLineToRGB888(void *pSrc, void *pDst);
static uint8_t ov7670_write_array(struct regval_list *vals);
static uint8_t ov7670_set_hw(int hstart, int hstop, int vstart, int vstop);
//static void DMA_Mem2MemConfig(void);


extern I2C_HandleTypeDef hi2c1;
extern LTDC_HandleTypeDef hltdc;
//DCMI_HandleTypeDef *hdcmi_ptr;
extern uint8_t VideoLineBuffer0[];
extern uint8_t VideoLineBuffer1[];
extern DMA_HandleTypeDef  M2MDmaHandle;

extern DMA2D_HandleTypeDef hdma2d;  

static struct regval_list ov7670_default_regs[] = {
	{ REG_COM7, COM7_RESET },
/*
 * Clock scale: 3 = 15fps
 *              2 = 20fps
 *              1 = 30fps
 */
	{ REG_CLKRC, 0x1 },	/* OV: clock scale (30 fps) */
	{ REG_TSLB,  0x04 },	/* OV */
	{ REG_COM7, 0 },	/* qcif */
	/*
	 * Set the hardware window.  These values from OV don't entirely
	 * make sense - hstop is less than hstart.  But they work...
	 */
	/*{ REG_HSTART, 0x13 },	{ REG_HSTOP, 0x01 },
	{ REG_HREF, 0xb6 },	{ REG_VSTART, 0x02 },
	{ REG_VSTOP, 0x7a },	{ REG_VREF, 0x0a },*/

	{ REG_COM3, 0 },	{ REG_COM14, 0 },
	/* Mystery scaling numbers */
	/*{ 0x70, 0x3a },		{ 0x71, 0x35 },
	{ 0x72, 0x11 },		{ 0x73, 0xf0 },
	{ 0xa2, 0x02 },		{ REG_COM10, 0x0 },*/

	/* Gamma curve values */
	/*{ 0x7a, 0x20 },		{ 0x7b, 0x10 },
	{ 0x7c, 0x1e },		{ 0x7d, 0x35 },
	{ 0x7e, 0x5a },		{ 0x7f, 0x69 },
	{ 0x80, 0x76 },		{ 0x81, 0x80 },
	{ 0x82, 0x88 },		{ 0x83, 0x8f },
	{ 0x84, 0x96 },		{ 0x85, 0xa3 },
	{ 0x86, 0xaf },		{ 0x87, 0xc4 },
	{ 0x88, 0xd7 },		{ 0x89, 0xe8 },*/

	/* AGC and AEC parameters.  Note we start by disabling those features,
	   then turn them only after tweaking the values. */
	{ REG_COM8, COM8_FASTAEC | COM8_AECSTEP | COM8_BFILT },
	{ REG_GAIN, 0x0F },	{ REG_AECH, 0 }, //0 0
	{ REG_COM4, 0x40 }, /* magic reserved bit */ // 0x040
	{ REG_COM9, 0x38 }, /* 4x gain + magic rsvd bit */ //0x14
	{ REG_BD50MAX, 0x05 },	{ REG_BD60MAX, 0x07 },
	{ REG_AEW, 0x95 },	{ REG_AEB, 0x33 },
	{ REG_VPT, 0xe3 },	{ REG_HAECC1, 0x78 },
	{ REG_HAECC2, 0x68 },	{ 0xa1, 0x03 }, /* magic */
	{ REG_HAECC3, 0xd8 },	{ REG_HAECC4, 0xd8 },
	{ REG_HAECC5, 0xf0 },	{ REG_HAECC6, 0x90 },
	{ REG_HAECC7, 0x94 },
	{ REG_COM8, COM8_FASTAEC|COM8_AECSTEP|COM8_BFILT|COM8_AGC|COM8_AEC },

	/* Almost all of these are magic "reserved" values.  */
	{ REG_COM5, 0x61 },	{ REG_COM6, 0x4b },
	{ 0x16, 0x02 },		{ REG_MVFP, 0x07 },
	{ 0x21, 0x02 },		{ 0x22, 0x91 },
	{ 0x29, 0x07 },		{ 0x33, 0x0b },
	{ 0x35, 0x0b },		{ 0x37, 0x1d },
	{ 0x38, 0x71 },		{ 0x39, 0x2a },
	{ REG_COM12, 0x78 },	{ 0x4d, 0x40 },
	{ 0x4e, 0x20 },		{ REG_GFIX, 0 },
	{ 0x6b, 0x4a },		{ 0x74, 0x10 },
	{ 0x8d, 0x4f },		{ 0x8e, 0 },
	{ 0x8f, 0 },		{ 0x90, 0 },
	{ 0x91, 0 },		{ 0x96, 0 },
	{ 0x9a, 0 },		{ 0xb0, 0x84 },
	{ 0xb1, 0x0c },		{ 0xb2, 0x0e },
	{ 0xb3, 0x82 },		{ 0xb8, 0x0a },

	/* More reserved magic, some of which tweaks white balance */
	{ 0x43, 0x0a },		{ 0x44, 0xf0 },
	{ 0x45, 0x34 },		{ 0x46, 0x58 },
	{ 0x47, 0x28 },		{ 0x48, 0x3a },
	{ 0x59, 0x88 },		{ 0x5a, 0x88 },
	{ 0x5b, 0x44 },		{ 0x5c, 0x67 },
	{ 0x5d, 0x49 },		{ 0x5e, 0x0e },
	{ 0x6c, 0x0a },		{ 0x6d, 0x55 },
	{ 0x6e, 0x11 },		{ 0x6f, 0x9f }, /* "9e for advance AWB" */
	{ 0x6a, 0x40 },		{ REG_BLUE, 0x40 },
	{ REG_RED, 0x60 },
	{ REG_COM8, COM8_FASTAEC|COM8_AECSTEP|COM8_BFILT|COM8_AGC|COM8_AEC|COM8_AWB },

	/* Matrix coefficients */
	{ 0x4f, 0x80 },		{ 0x50, 0x80 },
	{ 0x51, 0 },		{ 0x52, 0x22 },
	{ 0x53, 0x5e },		{ 0x54, 0x80 },
	{ 0x58, 0x9e },

	{ REG_COM16, COM16_AWBGAIN },	{ REG_EDGE, 0 },
	{ 0x75, 0x05 },		{ 0x76, 0xe1 },
	{ 0x4c, 0 },		{ 0x77, 0x01 },
	{ REG_COM13, 0xc3 },	{ 0x4b, 0x09 },
	{ 0xc9, 0x60 },		{ REG_COM16, 0x38 },
	{ 0x56, 0x40 },

	{ 0x34, 0x11 },		{ REG_COM11, COM11_EXP|COM11_HZAUTO },
	{ 0xa4, 0x88 },		{ 0x96, 0 },
	{ 0x97, 0x30 },		{ 0x98, 0x20 },
	{ 0x99, 0x30 },		{ 0x9a, 0x84 },
	{ 0x9b, 0x29 },		{ 0x9c, 0x03 },
	{ 0x9d, 0x4c },		{ 0x9e, 0x3f },
	{ 0x78, 0x04 },

	/* Extra-weird stuff.  Some sort of multiplexor register */
	{ 0x79, 0x01 },		{ 0xc8, 0xf0 },
	{ 0x79, 0x0f },		{ 0xc8, 0x00 },
	{ 0x79, 0x10 },		{ 0xc8, 0x7e },
	{ 0x79, 0x0a },		{ 0xc8, 0x80 },
	{ 0x79, 0x0b },		{ 0xc8, 0x01 },
	{ 0x79, 0x0c },		{ 0xc8, 0x0f },
	{ 0x79, 0x0d },		{ 0xc8, 0x20 },
	{ 0x79, 0x09 },		{ 0xc8, 0x80 },
	{ 0x79, 0x02 },		{ 0xc8, 0xc0 },
	{ 0x79, 0x03 },		{ 0xc8, 0x40 },
	{ 0x79, 0x05 },		{ 0xc8, 0x30 },
	{ 0x79, 0x26 },

	{ 0xff, 0xff },	/* END MARKER */
};

static struct regval_list ov7670_fmt_rgb565[] = {
  { REG_COM3, COM3_SCALEEN},
	{ REG_COM7, COM7_RGB | COM7_FMT_QCIF },	/* Selects RGB mode */
	{ REG_RGB444, 0 },	/* No RGB444 please */
	{ REG_COM1, 0x0 },	/* CCIR601 */
	{ REG_COM15, COM15_RGB565 },
	{ REG_COM9, 0x38 }, 	/* 16x gain ceiling; 0x8 is reserved bit */
	{ 0x4f, 0xb3 }, 	/* "matrix coefficient 1" */
	{ 0x50, 0xb3 }, 	/* "matrix coefficient 2" */
	{ 0x51, 0    },		/* vb */
	{ 0x52, 0x3d }, 	/* "matrix coefficient 4" */
	{ 0x53, 0xa7 }, 	/* "matrix coefficient 5" */
	{ 0x54, 0xe4 }, 	/* "matrix coefficient 6" */
	{ REG_COM13, COM13_GAMMA|COM13_UVSAT },
	{ 0xff, 0xff },
};


DCMI_HandleTypeDef *hdcmi_ptr1;
HAL_StatusTypeDef statusI2C;

HAL_StatusTypeDef HAL_I2C_Mem_Read_OV7670(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint32_t Timeout)
{
  return HAL_I2C_Mem_Read(hi2c, DevAddress, MemAddress, 1, pData, 1, Timeout);
}

void OV7670_config_window(uint16_t startx, uint16_t starty, uint16_t width, uint16_t height)
{
	uint16_t endx=(startx+width);
	uint16_t endy=(starty+height*2);  // must be "height*2"
	uint8_t temp_reg1, temp_reg2;
	uint8_t temp;
	
	//state = state;	   //Prevent report warning

	statusI2C = HAL_I2C_Mem_Read_OV7670(&hi2c1, OV7670_I2C_ADDRESS, 0x03, &temp_reg1, 1000); 
  //DCMI_SingleRandomRead(0x03, &temp_reg1 );
	temp_reg1 &= 0xC0;
	statusI2C = HAL_I2C_Mem_Read_OV7670(&hi2c1, OV7670_I2C_ADDRESS, 0x32, &temp_reg2, 1000); 
  //DCMI_SingleRandomRead(0x32, &temp_reg2 );
	temp_reg2 &= 0xC0;
	
	// Horizontal
	temp = temp_reg2|((endx&0x7)<<3)|(startx&0x7);
	statusI2C = HAL_I2C_Mem_Write(&hi2c1, OV7670_I2C_ADDRESS, 0x32, I2C_MEMADD_SIZE_8BIT, &temp, 1, 100);  
  //state = DCMI_SingleRandomWrite(0x32, temp );
	temp = (startx&0x7F8)>>3;
	statusI2C = HAL_I2C_Mem_Write(&hi2c1, OV7670_I2C_ADDRESS, 0x17, I2C_MEMADD_SIZE_8BIT, &temp, 1, 100);  
  //state = DCMI_SingleRandomWrite(0x17, temp );
	temp = (endx&0x7F8)>>3;
	statusI2C = HAL_I2C_Mem_Write(&hi2c1, OV7670_I2C_ADDRESS, 0x18, I2C_MEMADD_SIZE_8BIT, &temp, 1, 100);  
	//state = DCMI_SingleRandomWrite(0x18, temp );
	
	// Vertical
	temp = temp_reg1|((endy&0x7)<<3)|(starty&0x7);
	//state = DCMI_SingleRandomWrite(0x03, temp );
	statusI2C = HAL_I2C_Mem_Write(&hi2c1, OV7670_I2C_ADDRESS, 0x03, I2C_MEMADD_SIZE_8BIT, &temp, 1, 100);  
	temp = (starty&0x7F8)>>3;
	//state = DCMI_SingleRandomWrite(0x19, temp );
	statusI2C = HAL_I2C_Mem_Write(&hi2c1, OV7670_I2C_ADDRESS, 0x19, I2C_MEMADD_SIZE_8BIT, &temp, 1, 100);  
	temp = (endy&0x7F8)>>3;
	//state = DCMI_SingleRandomWrite(0x1A, temp );
	statusI2C = HAL_I2C_Mem_Write(&hi2c1, OV7670_I2C_ADDRESS, 0x1A, I2C_MEMADD_SIZE_8BIT, &temp, 1, 100);  
}


uint8_t temp1;
uint8_t temp2;
uint8_t temp3;
uint8_t temp4;
uint8_t temp5;
  
uint8_t ov7670_Init(uint32_t video_adr, DCMI_HandleTypeDef *phdcmi)
{  
  //HAL_StatusTypeDef status = HAL_OK;

  hdcmi_ptr1 = phdcmi;
//  temp1 = 0xD0;
//  status = HAL_I2C_Mem_Write(&hi2c1, OV7670_I2C_ADDRESS, 0x40, I2C_MEMADD_SIZE_8BIT, &temp1, 1, 100);  
//  
//  status = HAL_I2C_Mem_Read_OV7670(&hi2c1, OV7670_I2C_ADDRESS, 0x40, &temp1, 1000); 
//  status = HAL_I2C_Mem_Read_OV7670(&hi2c1, OV7670_I2C_ADDRESS, 0x1D, &temp2, 1000); 
//  status = HAL_I2C_Mem_Read_OV7670(&hi2c1, OV7670_I2C_ADDRESS, 0x0A, &temp3, 1000); 
//  status = HAL_I2C_Mem_Read_OV7670(&hi2c1, OV7670_I2C_ADDRESS, 0xFF, &temp4, 1000); 
//  status = HAL_I2C_Mem_Read(&hi2c1, 0x20, 0xFF, I2C_MEMADD_SIZE_8BIT, &temp5, 1, 1000);

  temp1 = SCCB_REG_RESET;
  statusI2C = HAL_I2C_Mem_Write(&hi2c1, OV7670_I2C_ADDRESS, OV7670_COM7, I2C_MEMADD_SIZE_8BIT, &temp1, 1, 1000);  
  
  HAL_Delay(50);  
  ov7670_write_array(ov7670_default_regs);
  ov7670_write_array(ov7670_fmt_rgb565);
  
  ov7670_set_hw(158, 14, 14, 494);
  //OV7670_config_window(140,16,640,480);
  temp1 = 0x01;
  statusI2C = HAL_I2C_Mem_Write(&hi2c1, OV7670_I2C_ADDRESS, REG_CLKRC, I2C_MEMADD_SIZE_8BIT, &temp1, 1, 100);  
  
	/*for( uint8_t i=0; i<CHANGE_REG_NUM; i++ )
	{
          temp1 = change_reg[i][1];
          HAL_I2C_Mem_Write(&hi2c1, OV7670_I2C_ADDRESS, change_reg[i][0], I2C_MEMADD_SIZE_8BIT, &temp1, 1, 100);  
	}*/

  /* Enable DMA2D clock */
  //__HAL_DMA2D_CLK_ENABLE();
  
  /* Configure the DMA2D Mode, Color Mode and output offset */
  /*hdma2d.Init.Mode         = DMA2D_M2M_PFC;
  hdma2d.Init.ColorMode    = DMA2D_RGB888;
  hdma2d.Init.OutputOffset = 0;  */   
  
  /* Foreground Configuration */
  /*hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0xFF;
  hdma2d.LayerCfg[1].InputColorMode = CM_RGB565;
  hdma2d.LayerCfg[1].InputOffset = 0;*/
  
  //hdma2d.Instance = DMA2D; 
  
  /* DMA2D Initialization */
  /*if(HAL_DMA2D_Init(&hdma2d) == HAL_OK) 
  {
    if(HAL_DMA2D_ConfigLayer(&hdma2d, 1) == HAL_OK) 
    {
    }
  }*/
        
  return 0;
}

uint8_t ov7670_Start(void)
{      
  //HAL_DCMI_Start_DoubleBuffDMA(hdcmi_ptr1, DCMI_MODE_CONTINUOUS, (uint32_t)VideoLineBuffer0, (uint32_t)VideoLineBuffer1, 640*2/4);
  __HAL_DCMI_CLEAR_FLAG(hdcmi_ptr1, DCMI_FLAG_OVFRI);
  __HAL_DCMI_ENABLE(hdcmi_ptr1);  
  DCMI->CR |= DCMI_CR_CAPTURE;
  HAL_LTDC_SetPixelFormat(&hltdc, LTDC_PIXEL_FORMAT_RGB565, 0);
  return 0;
}

uint32_t frame_counter=0;

uint8_t ov7670_Stop(void)
{
  HAL_DMA_Abort(hdcmi_ptr1->DMA_Handle);
  __HAL_DCMI_DISABLE(hdcmi_ptr1);
  __HAL_RCC_DMA2D_CLK_ENABLE();
  
  hdma2d.Init.Mode         = DMA2D_M2M_PFC;
  hdma2d.Init.ColorMode    = DMA2D_RGB888;
  hdma2d.Init.OutputOffset = 0;     
  
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0xFF;
  hdma2d.LayerCfg[1].InputColorMode = CM_RGB565;
  hdma2d.LayerCfg[1].InputOffset = 0;
  
  hdma2d.Instance = DMA2D; 
  
  if(HAL_DMA2D_Init(&hdma2d) == HAL_OK) 
  {
    if(HAL_DMA2D_ConfigLayer(&hdma2d, 1) == HAL_OK) 
    {
      HAL_DMA2D_Start(&hdma2d, 0xD0000000 /*+ 640*480*3*(frame_counter%2)*/, 0xC0000000, 640, 480*2);
      HAL_DMA2D_PollForTransfer(&hdma2d, 1000);
    }
  }
  return 0;
}

uint32_t current1_line=0;

uint8_t ov7670_LineProcessing(DMA_HandleTypeDef *hdma)
{  
  current1_line++;
  if((hdma->Instance->CR & DMA_SxCR_CT) == 0)
  {
  //pBuff = VideoLineBuffer1;
    hdma->Instance->M1AR = 0xD0000000 + 640*480*2*(frame_counter%2) + 640*2*current1_line;
  }
  else 
  {
  //pBuff = VideoLineBuffer0;
    hdma->Instance->M0AR = 0xD0000000 + 640*480*2*(frame_counter%2) + 640*2*current1_line;
  }
  //if ( current1_line >= 480 ) return 1;
  //HAL_DMA2D_Start_IT(&hdma2d, (uint32_t)pBuff, 0xD0000000 + 640*480*3*(frame_counter%2) + /*640*480*3*+*/640*3*current1_line, 640, 1);
    
  return 0;
}

uint8_t ov7670_FrameProcessing(DCMI_HandleTypeDef *phdcmi)
{
  current1_line = 0;
  frame_counter++;
  HAL_DMA_Abort(phdcmi->DMA_Handle);
  //HAL_DCMI_Start_DoubleBuffDMA(phdcmi, DCMI_MODE_CONTINUOUS, (uint32_t)VideoLineBuffer0, (uint32_t)VideoLineBuffer1, 640*2/4);
  //HAL_DCMI_Start_DoubleBuffDMA(phdcmi, DCMI_MODE_CONTINUOUS, 0xD0000000 + 640*480*2*(frame_counter%2), 0xD0000000 + 640*480*2*(frame_counter%2) + 640*2, 640*2/4);
  HAL_LTDC_SetAddress(&hltdc, 0xD0000000 + 640*480*2*((frame_counter+1)%2), 0);

  return 0;
}


/*static void ConvertLineToRGB888(void *pSrc, void *pDst)
{ 
  if (HAL_DMA2D_Start_IT(&hdma2d, (uint32_t)pSrc, (uint32_t)pDst, 640, 1) == HAL_OK)
  {
    //HAL_DMA2D_PollForTransfer(&hdma2d_eval, 10);
  }
}*/


// Write a list of register settings; ff/ff stops the process.

static uint8_t ov7670_write_array(struct regval_list *vals)
{
  while (vals->reg_num != 0xff || vals->value != 0xff) 
  {
    statusI2C = HAL_I2C_Mem_Write(&hi2c1, OV7670_I2C_ADDRESS, vals->reg_num, I2C_MEMADD_SIZE_8BIT, &vals->value, 1, 100);  
    vals++;
  }
  return 0;
}

static uint8_t ov7670_set_hw(int hstart, int hstop, int vstart, int vstop)
{
	uint8_t v;
  uint8_t temp;

// * Horizontal: 11 bits, top 8 live in hstart and hstop.  Bottom 3 of
// * hstart are in href[2:0], bottom 3 of hstop in href[5:3].  There is
// * a mystery "edge offset" value in the top two bits of href.
   
  temp= (hstart >> 3) & 0xff;
  statusI2C = HAL_I2C_Mem_Write(&hi2c1, OV7670_I2C_ADDRESS, REG_HSTART, I2C_MEMADD_SIZE_8BIT, &temp, 1, 100);  
  temp= (hstop >> 3) & 0xff;
  statusI2C = HAL_I2C_Mem_Write(&hi2c1, OV7670_I2C_ADDRESS, REG_HSTOP, I2C_MEMADD_SIZE_8BIT, &temp, 1, 100);  

  statusI2C = HAL_I2C_Mem_Read_OV7670(&hi2c1, OV7670_I2C_ADDRESS, REG_HREF, &v, 1000);
  v = (v & 0xc0) | ((hstop & 0x7) << 3) | (hstart & 0x7);
  HAL_Delay(10);
  statusI2C = HAL_I2C_Mem_Write(&hi2c1, OV7670_I2C_ADDRESS, REG_HREF, I2C_MEMADD_SIZE_8BIT, &v, 1, 100);  
  
// * Vertical: similar arrangement, but only 10 bits.
  
  temp= (vstart >> 2) & 0xff;
  statusI2C = HAL_I2C_Mem_Write(&hi2c1, OV7670_I2C_ADDRESS, REG_VSTART, I2C_MEMADD_SIZE_8BIT, &temp, 1, 100);  
  temp= (vstop >> 2) & 0xff;
  statusI2C = HAL_I2C_Mem_Write(&hi2c1, OV7670_I2C_ADDRESS, REG_VSTOP, I2C_MEMADD_SIZE_8BIT, &temp, 1, 100);  
  statusI2C = HAL_I2C_Mem_Read_OV7670(&hi2c1, OV7670_I2C_ADDRESS, REG_VREF, &v, 1000);
  v = (v & 0xf0) | ((vstop & 0x3) << 2) | (vstart & 0x3);
  HAL_Delay(10);
  statusI2C = HAL_I2C_Mem_Write(&hi2c1, OV7670_I2C_ADDRESS, REG_VREF, I2C_MEMADD_SIZE_8BIT, &v, 1, 100);  
	
  return 0;
}

