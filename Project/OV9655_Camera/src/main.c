/**
  ******************************************************************************
  * @file    OV9655_Camera/src/main.c  
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-April-2011
  * @brief   Main program body.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; Portions COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 
/**
  ******************************************************************************
  * <h2><center>&copy; Portions COPYRIGHT 2012 Embest Tech. Co., Ltd.</center></h2>
  * @file    main.c 
  * @author  CMP Team
  * @version V1.0.0
  * @date    28-December-2012
  * @brief   Main program body.                        
  *          Modified to support the STM32F4DISCOVERY, STM32F4DIS-BB, STM32F4DIS-CAM
  *          and STM32F4DIS-LCD modules. 
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, Embest SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT
  * OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
  * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
  * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_lcd.h"
#include "stm32f4_discovery_lis302dl.h"
#include "main.h"
#include "bmp.h"
#include "dcmi_ov9655.h"


/** @addtogroup STM32F4xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup DCMI_OV9655_Camera
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DCMI_DR_ADDRESS     0x50050028
#define FSMC_LCD_ADDRESS    0x60100000

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t KeyPressFlg = 0;
__IO uint32_t TimingDelay;
__IO uint32_t SysTickLimit;
__IO uint64_t CycleCounter;
__IO int newFrame;
RCC_ClocksTypeDef RCC_Clocks;
EXTI_InitTypeDef   EXTI_InitStructure;
uint8_t capture_Flag = ENABLE;

#define CAMERA_WIDTH    640
#define CAMERA_HEIGHT   480
#define HORIZONTAL_SCALE_FACTOR 2
#define VERTICAL_SCALE_FACTOR   2
uint8_t pbuffer1[CAMERA_WIDTH * 8 * 2];
uint8_t pbuffer2[CAMERA_WIDTH * 8 * 2];
static int blockRowsWritten;

// TODO: put these in a proper place
extern void cppInit(int width, int height);
extern void cppResetBinarizer(void);
extern void cppProcessBlockRow(uint8_t *blockRow);
extern uint8_t *cppGetResults(void);
extern int read_image(void);

/* Private function prototypes -----------------------------------------------*/
uint8_t DCMI_OV9655Config(void);
void DCMI_Config(void);
void I2C1_Config(void);
void EXTILine0_Config(void);
void LIS302DL_Reset(void);
void DisplayBinarizedFrame(void);

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
     */
  /* SysTick end of count event each 10ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTickLimit = RCC_Clocks.HCLK_Frequency / 1000;
  SysTick_Config(SysTickLimit);

  cppInit(CAMERA_WIDTH, CAMERA_HEIGHT);

  LIS302DL_Reset();

  /* SET USER Key */
  /* Configure EXTI Line0 (connected to PA0 pin) in interrupt mode */
  EXTILine0_Config();

  /* Initialize the LCD */
  STM32f4_Discovery_LCD_Init();
  LCD_Clear(LCD_COLOR_WHITE);
  LCD_SetTextColor(LCD_COLOR_BLUE);

  DCMI_Control_IO_Init();

  LCD_DisplayStringLine(LINE(2), "   Camera Init..");
		   
  /* OV9655 Camera Module configuration */
  if (DCMI_OV9655Config() == 0x00)
  {
    LCD_DisplayStringLine(LINE(2), "                ");
    LCD_SetDisplayWindow(0, 0, 320, 240);
    LCD_WriteRAM_Prepare();

    /* Start Image capture and Display on the LCD *****************************/
    /* Enable DMA transfer */
    DMA_Cmd(DMA2_Stream1, ENABLE);

    /* Enable DCMI interface */
    DCMI_Cmd(ENABLE); 

    /* Start Image capture */ 
    DCMI_CaptureCmd(ENABLE);   

    /*init the picture count*/
    init_picture_count();

    KeyPressFlg = 0;
    while (1)
    {
      /* Insert 100ms delay */
      //Delay(100);

      if (newFrame) {
        /* Processing of the frame will probably require more than the Vblank
         * interval, in which case the binarized frame will be partially
         * overwritten by the next one. That doesn't really matter as long
         * as there is not much motion. */
        DisplayBinarizedFrame();
        read_image();
        newFrame = 0;
      }
      if (KeyPressFlg) {
        KeyPressFlg = 0;
        /* press user KEY take a photo */
        if (capture_Flag == ENABLE) {
          DCMI_CaptureCmd(DISABLE);
          capture_Flag = DISABLE;
          //Capture_Image_TO_Bmp();
          Delay(100);
          LCD_SetDisplayWindow(0, 0, 320, 240);
          LCD_WriteRAM_Prepare();
          DCMI_CaptureCmd(ENABLE);
          capture_Flag = ENABLE;
        }			
      }
    }
  } else {
    LCD_SetTextColor(LCD_COLOR_RED);

    LCD_DisplayStringLine(LINE(2), "Camera Init.. fails");    
    LCD_DisplayStringLine(LINE(4), "Check the Camera HW ");    
    LCD_DisplayStringLine(LINE(5), "  and try again ");

    /* Go to infinite loop */
    while (1);      
  }
}

extern void fastDisplayBinarizedRow(uint16_t *out, uint8_t *in, uint32_t count);

#define SCALE_DOWN_FACTOR       2

void DisplayBinarizedFrame(void) {
  uint8_t *results = cppGetResults();
  /* It's not worth trying to optimise this loop. Profiling has revealed that
   * the majority of time is spent waiting for the LCD. */

  // Uncomment this block to rescale image
  for (int y = 0; y < CAMERA_HEIGHT; y += SCALE_DOWN_FACTOR) {
    for (int x = 0; x < CAMERA_WIDTH; x += 8) {
      uint8_t el = results[(y * CAMERA_WIDTH + x) >> 3];
      for (int j = 0; j < 8; j += SCALE_DOWN_FACTOR) {
        *(uint16_t *)FSMC_LCD_ADDRESS = el & 1 ? 0: 0xffff;
        el >>= SCALE_DOWN_FACTOR;
      }
    }
  }
  // Uncomment this block to zoom in on middle
  /*for (int y = 120; y < 360; y++) {
    for (int x = 160; x < 480; x += 8) {
      uint8_t el = results[(y * CAMERA_WIDTH + x) >> 3];
      for (int j = 0; j < 8; j++) {
        *(uint16_t *)FSMC_LCD_ADDRESS = el & 1 ? 0: 0xffff;
        el >>= 1;
      }
    }
  }*/
}

/**
  * @brief  Configures all needed resources (I2C, DCMI and DMA) to interface with
  *         the OV9655 camera module
  * @param  None
  * @retval 0x00 Camera module configured correctly 
  *         0xFF Camera module configuration failed
  */
uint8_t DCMI_OV9655Config(void)
{
  /* I2C1 will be used for OV9655 camera configuration */
  I2C1_Config();

  /* Reset and check the presence of the OV9655 camera module */
  if (DCMI_SingleRandomWrite(OV9655_DEVICE_WRITE_ADDRESS,0x12, 0x80))
  {
     return (0xFF);
  }

  ///* OV9655 Camera size setup */    
  DCMI_OV9655_VGASizeSetup();

  ///* Set the RGB565 mode */
  //DCMI_SingleRandomWrite(OV9655_DEVICE_WRITE_ADDRESS, OV9655_COM7, 0x63);
  //DCMI_SingleRandomWrite(OV9655_DEVICE_WRITE_ADDRESS, OV9655_COM15, 0x10);
  ///* Set the Raw RGB mode */
  //DCMI_SingleRandomWrite(OV9655_DEVICE_WRITE_ADDRESS, OV9655_COM7, 0x60);
  //DCMI_SingleRandomWrite(OV9655_DEVICE_WRITE_ADDRESS, OV9655_COM15, 0xc0);
  /* Set the YUV mode */
  DCMI_SingleRandomWrite(OV9655_DEVICE_WRITE_ADDRESS, OV9655_COM7, 0x62);
  DCMI_SingleRandomWrite(OV9655_DEVICE_WRITE_ADDRESS, OV9655_TSLB, 0xc0);
  DCMI_SingleRandomWrite(OV9655_DEVICE_WRITE_ADDRESS, OV9655_COM15, 0xc0);

  ///* Enable colour bar test mode */
  //DCMI_SingleRandomWrite(OV9655_DEVICE_WRITE_ADDRESS, OV9655_COM3, 0x80);
  //DCMI_SingleRandomWrite(OV9655_DEVICE_WRITE_ADDRESS, OV9655_COM20, 0x10);

  /* Enable night mode */
  // Actually, night mode seems to be a bad idea; it makes motion blur
  // much worse while reducing noise only a little bit.
  //DCMI_SingleRandomWrite(OV9655_DEVICE_WRITE_ADDRESS, OV9655_COM11, 0xe4);

  /* Invert the HRef signal*/
  DCMI_SingleRandomWrite(OV9655_DEVICE_WRITE_ADDRESS, OV9655_COM10, 0x08);

  /* Configure the DCMI to interface with the OV9655 camera module */
  DCMI_Config();
  
  return (0x00);
}

volatile static uint64_t last;
//static int ccc;

uint64_t getCycleCounter(void) {
  return CycleCounter + (uint64_t)(SysTickLimit - 1 - SysTick->VAL);
}

extern void extractLuminance(uint8_t *out, uint8_t *in, uint32_t count);

void DMA2_Stream1_IRQHandler(void)
{
  if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1) != RESET) {
    uint8_t *p;
    uint8_t luminances[CAMERA_WIDTH * 8];
    if (DMA_GetCurrentMemoryTarget(DMA2_Stream1) == DMA_Memory_0) {
      p = pbuffer2;
    } else {
      p = pbuffer1;
    }
    /*for (int y = 0; y < 8; y += VERTICAL_SCALE_FACTOR) {
      //if (y & 1) continue;
      for (int x = 0; x < CAMERA_WIDTH; x += HORIZONTAL_SCALE_FACTOR) {
        // extract Y component and convert to RGB565 greyscale
        uint32_t yuv = *(uint8_t *)&(p[x * 2]);
        uint16_t t1 = yuv >> 2;
        uint16_t t2 = yuv >> 3;
        uint16_t rgb = (t2 << 11) | (t1 << 5) | (t2);
        //uint16_t rgb = yuv > 0x80 ? 0xffff: 0;
        *(uint16_t *)FSMC_LCD_ADDRESS = rgb;
  //      luminances[i] = p[i * 2];
      }
      p += (CAMERA_WIDTH * 2 * VERTICAL_SCALE_FACTOR);
    }*/
    /*if ((blockRowsWritten >= 20) && (blockRowsWritten < 50))
    {
      for (int y = 0; y < 8; y++) {
        //if (y & 1) continue;
        for (int x = 160; x < 480; x ++) {
          // extract Y component and convert to RGB565 greyscale
          uint32_t yuv = *(uint8_t *)&(p[x * 2]);
          uint16_t t1 = yuv >> 2;
          uint16_t t2 = yuv >> 3;
          uint16_t rgb = (t2 << 11) | (t1 << 5) | (t2);
          *(uint16_t *)FSMC_LCD_ADDRESS = rgb;
        }
        p += (CAMERA_WIDTH * 2);
      }
    }*/
    extractLuminance(luminances, p, sizeof(luminances));
    //uint64_t current = getCycleCounter();
    //uint64_t diff = current - last;
    //last = current;
    //char s[32];
    //sprintf(s, "%u       ", (unsigned int)diff);
    //LCD_DisplayStringLine(LINE(blockRowsWritten), s);
    //blockRowsWritten = (blockRowsWritten + 1) % 6;
    //sprintf(s, "%d %d ", (int)ccc / 30, (int)ccc++);
    
    cppProcessBlockRow(luminances);
    blockRowsWritten++;
    if (blockRowsWritten >= (CAMERA_HEIGHT >> 3)) {
      newFrame = 1;
      cppResetBinarizer();
      blockRowsWritten = 0;
    }
    //LCD_DisplayStringLine(LINE(7), s);
    
    DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);
  }
}

/**
  * @brief  Configures the I2C1 used for OV9655 camera module configuration.
  * @param  None
  * @retval None
  */
void I2C1_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef  I2C_InitStruct;

 /* I2C1 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  /* GPIOB clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 

  /* Connect I2C1 pins to AF4 ************************************************/
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);  
  
  /* Configure I2C1 GPIOs *****************************************************/  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;   
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Configure I2C1 ***********************************************************/  
  /* I2C DeInit */   
  I2C_DeInit(I2C1);
    
  /* Enable the I2C peripheral */
  I2C_Cmd(I2C1, ENABLE);
 
  /* Set the I2C structure parameters */
  I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStruct.I2C_OwnAddress1 = 0xFE;
  I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStruct.I2C_ClockSpeed = 30000;
  
  /* Initialize the I2C peripheral w/ selected parameters */
  I2C_Init(I2C1, &I2C_InitStruct);
}

/**
  * @brief  Configures the DCMI to interface with the OV9655 camera module.
  * @param  None
  * @retval None
  */
void DCMI_Config(void)
{
  DCMI_InitTypeDef DCMI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  DMA_InitTypeDef  DMA_InitStructure;
  DCMI_CROPInitTypeDef DCMI_CropInitStructure;
  
  /* Enable DCMI GPIOs clocks */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOE | 
                         RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOA, ENABLE); 

  /* Enable DCMI clock */
  RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI, ENABLE);

  /* Connect DCMI pins to AF13 ************************************************/
  /* PCLK */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_DCMI);
  /* D0-D7 */
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_DCMI);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_DCMI);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource0, GPIO_AF_DCMI);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource1, GPIO_AF_DCMI);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource4, GPIO_AF_DCMI);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_DCMI);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_DCMI);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_DCMI);
  /* VSYNC */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_DCMI);
  /* HSYNC */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_DCMI);
  
  /* DCMI GPIO configuration **************************************************/
  /* D0 D1(PC6/7) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;  
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* D2..D4(PE0/1/4) D6/D7(PE5/6) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 
	                              | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  /* D5(PB6), VSYNC(PB7) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* PCLK(PA6) HSYNC(PA4)*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* DCMI configuration *******************************************************/ 
  DCMI_InitStructure.DCMI_CaptureMode = DCMI_CaptureMode_Continuous;
  DCMI_InitStructure.DCMI_SynchroMode = DCMI_SynchroMode_Hardware;
  DCMI_InitStructure.DCMI_PCKPolarity = DCMI_PCKPolarity_Falling;
  DCMI_InitStructure.DCMI_VSPolarity = DCMI_VSPolarity_High;
  DCMI_InitStructure.DCMI_HSPolarity = DCMI_HSPolarity_High;
  DCMI_InitStructure.DCMI_CaptureRate = DCMI_CaptureRate_All_Frame;
  DCMI_InitStructure.DCMI_ExtendedDataMode = DCMI_ExtendedDataMode_8b;

  // All horizontal crop values are multiplied by 2 because there are 2
  // pixel clocks per pixel.
  DCMI_CropInitStructure.DCMI_VerticalStartLine = 0;
  DCMI_CropInitStructure.DCMI_HorizontalOffsetCount = 0;
#ifdef CROP_LAST_BLOCK_ROW
  DCMI_CropInitStructure.DCMI_VerticalLineCount = CAMERA_HEIGHT - 8;
#else
  DCMI_CropInitStructure.DCMI_VerticalLineCount = CAMERA_HEIGHT;
#endif
  DCMI_CropInitStructure.DCMI_CaptureCount = CAMERA_WIDTH * 2;
  // According to the STM32F4 reference manual, the counts in the registers
  // are 1 less than the actual count.
  DCMI_CropInitStructure.DCMI_CaptureCount--;
  DCMI_CropInitStructure.DCMI_VerticalLineCount--;
  DCMI_Init(&DCMI_InitStructure);
  DCMI_CROPConfig(&DCMI_CropInitStructure);
  DCMI_CROPCmd(ENABLE);

  /* DMA2 interrupt configuration */
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configures the DMA2 to transfer Data from DCMI to the LCD ****************/
  /* Enable DMA2 clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);  
  
  /* DMA2 Stream1 Configuration */  
  DMA_DeInit(DMA2_Stream1);

  DMA_InitStructure.DMA_Channel = DMA_Channel_1;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = DCMI_DR_ADDRESS;	
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)pbuffer1;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = sizeof(pbuffer1) / sizeof(uint32_t);
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

  DMA_Init(DMA2_Stream1, &DMA_InitStructure);
  DMA_DoubleBufferModeConfig(DMA2_Stream1, (uint32_t)pbuffer2, DMA_Memory_0);
  DMA_DoubleBufferModeCmd(DMA2_Stream1, ENABLE);
  DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);
}

/**

  * @brief  
  * @param  None
  * @retval None
  */
void LIS302DL_Reset(void)
{
  uint8_t ctrl = 0;
  
  LIS302DL_InitTypeDef  LIS302DL_InitStruct;
  LIS302DL_InterruptConfigTypeDef LIS302DL_InterruptStruct;  
  
  /* Set configuration of LIS302DL*/
  LIS302DL_InitStruct.Power_Mode = LIS302DL_LOWPOWERMODE_ACTIVE;
  LIS302DL_InitStruct.Output_DataRate = LIS302DL_DATARATE_100;
  LIS302DL_InitStruct.Axes_Enable = LIS302DL_X_ENABLE | LIS302DL_Y_ENABLE | LIS302DL_Z_ENABLE;
  LIS302DL_InitStruct.Full_Scale = LIS302DL_FULLSCALE_2_3;
  LIS302DL_InitStruct.Self_Test = LIS302DL_SELFTEST_NORMAL;
  LIS302DL_Init(&LIS302DL_InitStruct);
    
  /* Set configuration of Internal High Pass Filter of LIS302DL*/
  LIS302DL_InterruptStruct.Latch_Request = LIS302DL_INTERRUPTREQUEST_LATCHED;
  LIS302DL_InterruptStruct.SingleClick_Axes = LIS302DL_CLICKINTERRUPT_Z_ENABLE;
  LIS302DL_InterruptStruct.DoubleClick_Axes = LIS302DL_DOUBLECLICKINTERRUPT_Z_ENABLE;
  LIS302DL_InterruptConfig(&LIS302DL_InterruptStruct);

  /* Required delay for the MEMS Accelerometre: Turn-on time = 3/Output data Rate 
                                                             = 3/100 = 30ms */
  Delay(30);
  
  /* Configure Click Window */
  ctrl = 0xC0;
  LIS302DL_Write(&ctrl, LIS302DL_CLICK_CTRL_REG3_ADDR, 1);
}

/**
  * @brief  MEMS accelerometre management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t LIS302DL_TIMEOUT_UserCallback(void)
{
  /* MEMS Accelerometer Timeout error occured */
  while (1) ;
}
 
/**
  * @brief  Configures EXTI Line0 (connected to PA0 pin) in interrupt mode
  * @param  None
  * @retval None
  */
void EXTILine0_Config(void)
{
  
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOA clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect EXTI Line0 to PA0 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

  /* Configure EXTI Line0 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds
  * @retval None
  */
void Delay(uint32_t nTime)
{
  TimingDelay = nTime;

  while (TimingDelay != 0);

}

/**

  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  CycleCounter += SysTickLimit;
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */



/*********** Portions COPYRIGHT 2012 Embest Tech. Co., Ltd.*****END OF FILE****/
