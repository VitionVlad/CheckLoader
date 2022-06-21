/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"

#include "bootspl.h"

#include "err.h"

#include "upm.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t floffset;

uint8_t buffer[64];

extern USBD_HandleTypeDef hUsbDeviceFS;

uint32_t readWord(uint32_t address){
	uint32_t read_data;
	read_data = *(uint32_t*)(address);
	return read_data;
}

uint16_t TFT9341_WIDTH;
uint16_t TFT9341_HEIGHT;

void TFT9341_SendCommand(uint8_t cmd)
{
  DC_COMMAND();
  HAL_SPI_Transmit (&hspi1, &cmd, 1, 5000);
}

void TFT9341_SendData(uint8_t dt)
{
	DC_DATA();
	HAL_SPI_Transmit (&hspi1, &dt, 1, 5000);
}

static void TFT9341_WriteData(uint8_t* buff, size_t buff_size) {
	DC_DATA();
	while(buff_size > 0) {
		uint16_t chunk_size = buff_size > 32768 ? 32768 : buff_size;
		HAL_SPI_Transmit(&hspi1, buff, chunk_size, HAL_MAX_DELAY);
		buff += chunk_size;
		buff_size -= chunk_size;
	}
}

void TFT9341_reset(void)
{
	RESET_ACTIVE();
	HAL_Delay(5);
	RESET_IDLE();
}

static void TFT9341_SetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  // column address set
  TFT9341_SendCommand(0x2A); // CASET
  {
    uint8_t data[] = { (x0 >> 8) & 0xFF, x0 & 0xFF, (x1 >> 8) & 0xFF, x1 & 0xFF };
    TFT9341_WriteData(data, sizeof(data));
  }

  // row address set
  TFT9341_SendCommand(0x2B); // RASET
  {
    uint8_t data[] = { (y0 >> 8) & 0xFF, y0 & 0xFF, (y1 >> 8) & 0xFF, y1 & 0xFF };
    TFT9341_WriteData(data, sizeof(data));
  }

  // write to RAM
  TFT9341_SendCommand(0x2C); // RAMWR
}

void TFT9341_FillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
  if((x1 >= TFT9341_WIDTH) || (y1 >= TFT9341_HEIGHT) || (x2 >= TFT9341_WIDTH) || (y2 >= TFT9341_HEIGHT)) return;
	if(x1>x2) swap(x1,x2);
	if(y1>y2) swap(y1,y2);
  TFT9341_SetAddrWindow(x1, y1, x2, y2);
  uint8_t data[] = { color >> 8, color & 0xFF };
  DC_DATA();
  for(uint32_t i = 0; i < (x2-x1+1)*(y2-y1+1); i++)
  {
      HAL_SPI_Transmit(&hspi1, data, 2, HAL_MAX_DELAY);
  }
}

void TFT9341_ini(uint16_t w_size, uint16_t h_size){
	uint8_t data[15];
	CS_ACTIVE();
	TFT9341_reset();
	TFT9341_SendCommand(0x01);
	HAL_Delay(1000);
	 HAL_Delay(1000);
	  data[0] = 0x39;
	  data[1] = 0x2C;
	  data[2] = 0x00;
	  data[3] = 0x34;
	  data[4] = 0x02;
	  TFT9341_SendCommand(0xCB);
	  TFT9341_WriteData(data, 5);
	  data[0] = 0x00;
	  data[1] = 0xC1;
	  data[2] = 0x30;
	  TFT9341_SendCommand(0xCF);
	  TFT9341_WriteData(data, 3);
	  data[0] = 0x85;
	  data[1] = 0x00;
	  data[2] = 0x78;
	  TFT9341_SendCommand(0xE8);
	  TFT9341_WriteData(data, 3);
	  data[0] = 0x00;
	  data[1] = 0x00;
	  TFT9341_SendCommand(0xEA);
	  TFT9341_WriteData(data, 2);
	  data[0] = 0x64;
	  data[1] = 0x03;
	  data[2] = 0x12;
	  data[3] = 0x81;
	  TFT9341_SendCommand(0xED);
	  TFT9341_WriteData(data, 4);
	  data[0] = 0x20;
	  TFT9341_SendCommand(0xF7);
	  TFT9341_WriteData(data, 1);
	  data[0] = 0x10;
	  TFT9341_SendCommand(0xC0);
	  TFT9341_WriteData(data, 1);
	  data[0] = 0x10;
	  TFT9341_SendCommand(0xC1);
	  TFT9341_WriteData(data, 1);
	  data[0] = 0x3E;
	  data[1] = 0x28;
	  TFT9341_SendCommand(0xC5);
	  TFT9341_WriteData(data, 2);
	  data[0] = 0x86;
	  TFT9341_SendCommand(0xC7);
	  TFT9341_WriteData(data, 1);
	  data[0] = 0x48;
	  TFT9341_SendCommand(0x36);
	  TFT9341_WriteData(data, 1);
	  data[0] = 0x55;
	  TFT9341_SendCommand(0x3A);
	  TFT9341_WriteData(data, 1);
	  data[0] = 0x00;
	  data[1] = 0x18;
	  TFT9341_SendCommand(0xB1);
	  TFT9341_WriteData(data, 2);
	  data[0] = 0x08;
	  data[1] = 0x82;
	  data[2] = 0x27;
	  TFT9341_SendCommand(0xB6);
	  TFT9341_WriteData(data, 3);
	  data[0] = 0x00;
	  TFT9341_SendCommand(0xF2);
	  TFT9341_WriteData(data, 1);
	  data[0] = 0x01;
	  TFT9341_SendCommand(0x26);
	  TFT9341_WriteData(data, 1);
	  data[0] = 0x0F;
	  data[1] = 0x31;
	  data[2] = 0x2B;
	  data[3] = 0x0C;
	  data[4] = 0x0E;
	  data[5] = 0x08;
	  data[6] = 0x4E;
	  data[7] = 0xF1;
	  data[8] = 0x37;
	  data[9] = 0x07;
	  data[10] = 0x10;
	  data[11] = 0x03;
	  data[12] = 0x0E;
	  data[13] = 0x09;
	  data[14] = 0x00;
	  TFT9341_SendCommand(0xE0);
	  TFT9341_WriteData(data, 15);
	  data[0] = 0x00;
	  data[1] = 0x0E;
	  data[2] = 0x14;
	  data[3] = 0x03;
	  data[4] = 0x11;
	  data[5] = 0x07;
	  data[6] = 0x31;
	  data[7] = 0xC1;
	  data[8] = 0x48;
	  data[9] = 0x08;
	  data[10] = 0x0F;
	  data[11] = 0x0C;
	  data[12] = 0x31;
	  data[13] = 0x36;
	  data[14] = 0x0F;
	  TFT9341_SendCommand(0xE1);
	  TFT9341_WriteData(data, 15);
	  TFT9341_SendCommand(0x11);
	  HAL_Delay(120);
	  data[0] = TFT9341_ROTATION;
	  TFT9341_SendCommand(0x29);
	  TFT9341_WriteData(data, 1);
	  TFT9341_WriteData(data, 1);
	  TFT9341_WIDTH = w_size;
	  TFT9341_HEIGHT = h_size;
}

void TFT9341_FillScreen(uint16_t color)
{
  TFT9341_FillRect(0, 0, TFT9341_WIDTH-1, TFT9341_HEIGHT-1, color);
}

void TFT9341_DrawPixel(uint16_t x, uint16_t y, uint16_t color){
	TFT9341_SetAddrWindow(x, y, x, y);
	uint8_t data[] = { color >> 8, color & 0xFF };
	DC_DATA();
	HAL_SPI_Transmit(&hspi1, data, 2, 0);
}

void TFT9341_DrawMassive128x128(uint16_t mimage[16384], uint16_t orientation){
	uint16_t currcolnum = 0;
	uint16_t x = 0;
	for(uint16_t y = 0; y != TFT9341_HEIGHT;y++){
		for(; x != TFT9341_HEIGHT;x++){
			switch(orientation){
			case 0:
				TFT9341_DrawPixel(TFT9341_WIDTH-x, y, mimage[currcolnum]);
				break;
			case 1:
				TFT9341_DrawPixel(y, x, mimage[currcolnum]);
				break;
			case 2:
				TFT9341_DrawPixel(TFT9341_WIDTH-y, TFT9341_HEIGHT-x, mimage[currcolnum]);
				break;
			case 3:
				TFT9341_DrawPixel(x, TFT9341_HEIGHT-y, mimage[currcolnum]);
				break;
			}
			currcolnum++;
		}
		x = 0;
	}
}

uint16_t checkmem(){
	uint8_t emptyCellCount = 0;
	for(uint8_t i=0; i<10; i++){
		if(readWord(AppStart + (i*4)) == -1)
			emptyCellCount++;
	}
	if(emptyCellCount != 10)
		return 1;
	else
		return 0;
}

typedef void (application_t)(void);

typedef struct{
    uint32_t		stack_addr;     // Stack Pointer
    application_t*	func_p;        // Program Counter
} JumpStruct;

void deinitEverything()
{
	//-- reset peripherals to guarantee flawless start of user application
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2);
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7);
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);
	HAL_SPI_DeInit(&hspi1);
	USBD_DeInit(&hUsbDeviceFS);
	  __HAL_RCC_GPIOC_CLK_DISABLE();
	  __HAL_RCC_GPIOD_CLK_DISABLE();
	  __HAL_RCC_GPIOB_CLK_DISABLE();
	  __HAL_RCC_GPIOA_CLK_DISABLE();
	HAL_RCC_DeInit();
	HAL_DeInit();
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;
}

void jumpToApp(const uint32_t address){
	const JumpStruct* vector_p = (JumpStruct*)address;
	deinitEverything();
    asm("msr msp, %0; bx %1;" : : "r"(vector_p->stack_addr), "r"(vector_p->func_p));
}

void eraseMemory(){
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.Sector = AppStart;
	EraseInitStruct.NbSectors = 0X5800/0x400;
	uint32_t PageError;
	HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  TFT9341_ini(128, 128);

  TFT9341_DrawMassive128x128(bootsplash, 1);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

  HAL_Delay(1000);

  uint16_t check = checkmem();

  if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED){
	  check = 0;
  }

  if(check == 1){
	  TFT9341_reset();
	  jumpToApp(AppStart);
  }else{
	  if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED){
	  	  TFT9341_DrawMassive128x128(uploadimage, 1);
	  }else{
		  TFT9341_DrawMassive128x128(errscreen, 1);
	  }
	  while(HAL_FLASH_Unlock()!=HAL_OK)
	  		while(HAL_FLASH_Lock()!=HAL_OK);
	  	while(HAL_FLASH_OB_Unlock()!=HAL_OK)
	  		while(HAL_FLASH_OB_Lock()!=HAL_OK);
  }

  uint8_t single_ = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
	  if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && single_ == 0){
		  TFT9341_DrawMassive128x128(uploadimage, 1);
		  single_ = 1;
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA2 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
