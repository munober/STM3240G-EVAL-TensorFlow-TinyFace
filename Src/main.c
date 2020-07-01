/**
  ******************************************************************************
  * @file    Camera/Camera_To_USBDisk/Src/main.c 
  * @author  MCD Application Team
  * @brief   This application describes how to configure the camera in continuous mode
             and save picture under USBDisk.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "main_functions.h"
#include "constants.h"
#include "model_data.h"

/** @addtogroup STM32F4xx_HAL_Applications
  * @{
  */

/** @addtogroup Camera_To_USBDisk
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define PRESSED_FIRST    0x00

#define NUM_IN_CH 1
#define NUM_OUT_CH 1
#define IMG_WIDTH 320
#define IMG_HEIGHT 240
#define CNN_IMG_SIZE 50

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t ubPressedButton = PRESSED_FIRST;

FATFS MSC_FatFs;  /* File system object for USB disk logical drive */
FIL MyFile;       /* File object */
char MSC_Path[4]; /* USB Host logical drive path */
USBH_HandleTypeDef  hUSBHost;

/* Image header */  
const uint32_t aBMPHeader[14]=
{0xB0364D42, 0x00000004, 0x00360000, 0x00280000, 0x01400000, 0x00F00000, 0x00010000, 
 0x00000020, 0xF5400000, 0x00000006, 0x00000000, 0x00000000, 0x00000000, 0x0000};

typedef enum {
  STORAGE_IDLE = 0,  
  STORAGE_READY,    
}MSC_ApplicationTypeDef;

MSC_ApplicationTypeDef Appli_state = STORAGE_IDLE;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void PicturePrepare(void);
static void Error_Handler(void);
static void SavePicture(void);
static void CAMERA_Capture(void);
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);
void resize_rgb565in_rgb888out(uint8_t* camera_image, uint8_t* resize_image);
static void DetectFace(void);
static uint8_t NormalizeImage(uint8_t image);

#define RGB565_TO_R(pixel)   (((pixel & 0x1F) << 3) | ((((pixel & 0x1F) << 3) & 0xE0) >> 5));
#define RGB565_TO_G(pixel)   (((pixel & 0x7E0) >> 3) | ((((pixel & 0x7E0) >> 3) & 0xC0) >> 6));
#define RGB565_TO_B(pixel)   (((pixel & 0xF800) >> 8) | ((((pixel & 0xF800) >> 8) & 0xE0) >> 5));
  
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();
  
  /* Configure the system clock to 168 MHz */
  SystemClock_Config();
  
  /* Configure LED1 and LED3 */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);
  
  /*##-1- Init Host Library ##################################################*/
  USBH_Init(&hUSBHost, USBH_UserProcess, 0);
  
  /* Add Supported Class */
  USBH_RegisterClass(&hUSBHost, USBH_MSC_CLASS);
  
  /* Start Host Process */
  USBH_Start(&hUSBHost);
  
  /*##-2- Configure Tamper button ############################################*/
  BSP_PB_Init(BUTTON_TAMPER, BUTTON_MODE_GPIO);
  
  /*##-3- Link the USB Host disk I/O driver ##################################*/
  FATFS_LinkDriver(&USBH_Driver, MSC_Path);

  /*##-4- Initialize the SRAM and LCD ########################################*/ 
  BSP_LCD_Init(); 
  BSP_SRAM_Init();
  
  /*##-5- Camera Initialization and start capture ############################*/
  /* Initialize the Camera */
  BSP_CAMERA_Init(RESOLUTION_R320x240);
  
  /* Start the Camera Capture */
  BSP_CAMERA_ContinuousStart((uint8_t *)CAMERA_FRAME_BUFFER);
   
  /*##-6- Run Application ####################################################*/
  while (1)
  { 
    /* USB Host Background task */
    USBH_Process(&hUSBHost);

    switch(Appli_state)
    {
    case STORAGE_READY:
      CAMERA_Capture();
      break;
        
    case STORAGE_IDLE:
    default:
      break;      
    } 
  }
}

/**
  * @brief  Frame Event callback.
  * @param  None
  * @retval None
*/
void BSP_CAMERA_FrameEventCallback(void)
{
  /* Display on LCD */
  BSP_LCD_DrawRGBImage(0, 0, 320, 240, (uint8_t *)CAMERA_FRAME_BUFFER);
}

/**
  * @brief  Main routine for Mass Storage Class
  * @param  None
  * @retval None
  */
static void SavePicture(void)
{
  FRESULT res1, res2;     /* FatFs function common result code */
  uint32_t byteswritten;  /* File write count */
  
  static uint32_t counter = 0;
  uint8_t str[30];
  
  /* Suspend the camera capture */
  // BSP_CAMERA_Suspend();
  
  /* Prepare the image to be saved */
  // PicturePrepare();
  
  /* Format the string */
  // sprintf((char *)str, "image_%lu.bmp", counter);
  
  
  /* Create and Open a new file object with write access */
  // if(f_open(&MyFile, (const char*)str, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) 
  // {
  //   Error_Handler();
  // }
  // else
  // {
  //   /* Write data to the BMP file */
  //   res1 = f_write(&MyFile, (uint32_t *)aBMPHeader, 54, (void *)&byteswritten);
  //   res2 = f_write(&MyFile, (uint16_t *)SRAM_DEVICE_ADDR, (BSP_LCD_GetYSize()*BSP_LCD_GetXSize()*sizeof(uint32_t)), (void *)&byteswritten);

  //   /*
  //       file to be written is at &MyFile
  //       pointer to data is aBMPHeader
  //   */
    
    
  //   if((res1 != FR_OK) || (res2 != FR_OK) || (byteswritten == 0))
  //   {
  //     Error_Handler();
  //   }
  //   else
  //   {
  //     /* Close the open BMP file */
  //     f_close(&MyFile);
      
  //     /* Success of the demo: no error occurrence */
  //     BSP_LED_On(LED1);

  //     /* Holding up */
  //     HAL_Delay(500);

  //     counter++;
  //     BSP_LED_Off(LED1);
  //     BSP_LED_Off(LED4);
  //   }
  // }




}

/**
  * @brief  Main routine for Camera capture
  * @param  None
  * @retval None
  */
static void CAMERA_Capture(void)
{
  while(1)
  {
    if(BSP_PB_GetState(BUTTON_TAMPER) != GPIO_PIN_RESET) 
    {
      if(BSP_PB_GetState(BUTTON_TAMPER) != GPIO_PIN_SET) 
      {
        // SavePicture();
        BSP_CAMERA_Suspend();
        DetectFace();
        BSP_CAMERA_Resume();
      }
    }
  }
}  

/**
  * @brief  Prepares the picture to be Saved in USB.
  * @param  None
  * @retval None
  */
static void PicturePrepare(void) 
{
  uint32_t address = SRAM_DEVICE_ADDR;
  uint16_t x = 0;
  uint16_t y = 0;
  uint16_t tmp = 0;
  uint8_t aRGB[4];
  
  /* Go to the address of the last line of BMP file */
  address += ((BSP_LCD_GetXSize() * (BSP_LCD_GetYSize() - 1)) * 4);

  /* Read data from GRAM and swap it into SRAM */
  for(y = 0; y < (BSP_LCD_GetYSize()); y++)
  { 
    for(x = 0; x < (BSP_LCD_GetXSize()); x++)
    {      
      /* Write data to the SRAM memory */
      tmp  = BSP_LCD_ReadPixel(x, y); 
      
      aRGB[0] =  RGB565_TO_R(tmp);
      aRGB[1] =  RGB565_TO_G(tmp);
      aRGB[2] =  RGB565_TO_B(tmp);
      aRGB[3] =  0xFF;
      
      if(BSP_SRAM_WriteData(address, (uint16_t *)aRGB, 2) != SRAM_OK)
      {
        Error_Handler();
      }
      else
      {
        address += 4;
      }
    }
    address -= 8*BSP_LCD_GetXSize();
  }    
}

/**
  * @brief  User Process
  * @param  phost: Host handle
  * @param  id: Host Library user message ID
  * @retval None
  */
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id)
{  
  switch (id)
  {
  case HOST_USER_SELECT_CONFIGURATION:
    break;
    
  case HOST_USER_DISCONNECTION:
    Appli_state = STORAGE_IDLE;
    f_mount(NULL, (TCHAR const*)"", 0);      
    break;
    
  case HOST_USER_CLASS_ACTIVE:
    /* Register the file system object to the FatFs module */
    if(f_mount(&MSC_FatFs, (TCHAR const*)MSC_Path, 0) != FR_OK)
    {
      /* FatFs Initialization Error */
      Error_Handler();
    }
    else
    {
      Appli_state = STORAGE_READY;
    }
    break;
  }
}

void resize_rgb565in_rgb888out(uint8_t* camera_image, uint8_t* resize_image)
{
  // offset so that only the center part of rectangular image is selected for resizing
  int width_offset = ((IMG_WIDTH-IMG_HEIGHT)/2)*NUM_IN_CH;

  int yresize_ratio = (IMG_HEIGHT/CNN_IMG_SIZE)*NUM_IN_CH;
  int xresize_ratio = (IMG_WIDTH/CNN_IMG_SIZE)*NUM_IN_CH;
  int resize_ratio = (xresize_ratio<yresize_ratio)?xresize_ratio:yresize_ratio;

  for(int y=0; y<CNN_IMG_SIZE; y++) {
    for(int x=0; x<CNN_IMG_SIZE; x++) {
      int orig_img_loc = (y*IMG_WIDTH*resize_ratio + x*resize_ratio + width_offset);
      // correcting the image inversion here
      int out_img_loc = ((CNN_IMG_SIZE-1-y)*CNN_IMG_SIZE + (CNN_IMG_SIZE-1-x))*NUM_OUT_CH;
      uint8_t pix_lo = camera_image[orig_img_loc];
      uint8_t pix_hi = camera_image[orig_img_loc+1];
      // convert RGB565 to RGB888
      resize_image[out_img_loc] = (0xF8 & pix_hi); 
      resize_image[out_img_loc+1] = ((0x07 & pix_hi)<<5) | ((0xE0 & pix_lo)>>3);
      resize_image[out_img_loc+2] = (0x1F & pix_lo) << 3;
    }
  }
}

static void DetectFace(void){
  uint8_t resized_buffer[NUM_OUT_CH*CNN_IMG_SIZE*CNN_IMG_SIZE];
  resize_rgb565in_rgb888out((uint8_t *)CAMERA_FRAME_BUFFER, resized_buffer);
  float output_data[2];
  BSP_LED_On(LED1);
  static char ret_char;
  ret_char = loop((uint8_t*)resized_buffer, CNN_IMG_SIZE * CNN_IMG_SIZE);


  BSP_LCD_SetFont(&Font16);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_DisplayStringAt(20, 20, (uint8_t *)((int)output_data[0] * 100), LEFT_MODE);
  BSP_LCD_DisplayStringAt(40, 20, (uint8_t *)((int)output_data[1] * 100), LEFT_MODE);

  BSP_LED_On(LED4); // succesful inference
  HAL_Delay(500);
  BSP_LED_Off(LED4);
  BSP_LED_Off(LED1);
}

static uint8_t NormalizeImage(uint8_t image){
  uint8_t min = 0;
  uint8_t max = 0;

  // finding the minimum and maximum original range

  for(int i = 0; i < (sizeof(image) / sizeof(uint8_t)); i++){
    for(int j  = 0; j <  (sizeof(image) / sizeof(uint8_t)); j++){
      uint8_t tmp = image;
      if (tmp < min)
        min = tmp;
      if (tmp > max)
        max = tmp;
    }
  }

  float scale = 10 / (max - min);

  for(int i = 0; i < (sizeof(image) / sizeof(uint8_t)); i++){
    for(int j  = 0; j <  (sizeof(image) / sizeof(uint8_t)); j++){
      uint8_t tmp = image;
      // image[i][j] = uint8_t(scale * tmp);
    }
  }

  return image;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED3 on */
  BSP_LED_On(LED3); 
  while(1)
  {
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and Activate PLL with HSE ad source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  if (HAL_GetREVID() == 0x1001)
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
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

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
