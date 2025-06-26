/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fonts.h"
#include <stdio.h>
#include "ili9341.h"
#include <string.h>

#include "fft.h"
#include "guitar_tuner.h"
#include "unit_vec.h"
#include "guitar_tuner_bitmap.h"
#include "arrow_bitmaps.h"
#include "tunings.h"

//typedef enum {E, A, D, G, B, e} guitar_string;

//float ideal_frequencies[] = {82.0312*2, 109.375*3, 148.437, 195.312, 246.093, 328.125 };
//float thresh[6] = {0.5, 0.7, 0.9, 1.0, 1.2, 1.2};





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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
ILI9341TypeDef display;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void drawAxis(ILI9341TypeDef *display);
static void clearCursor(ILI9341TypeDef *display, uint16_t pos);
static void clearTrigger(ILI9341TypeDef *display, uint16_t pos);
static void drawCursor(ILI9341TypeDef *display, uint16_t pos, char *name, uint16_t color);
static void drawTrigger(ILI9341TypeDef *display, uint16_t pos, char *name, uint16_t color);
static void drawSignal(ILI9341TypeDef *display, uint32_t *adc_time, uint16_t *adc0, uint32_t adc_length, uint16_t pixel_dirty[280][2], uint16_t cursor, uint16_t color);
static void clearSignal(ILI9341TypeDef *display, uint16_t pixel_dirty[280][2]);
static void drawSignalParam(ILI9341TypeDef *display, char *string, size_t size, uint16_t adc_max, uint16_t adc_min, uint32_t adc_period);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//ADC buf init
	#define ADC_BUFFER_SIZE 256
	uint16_t adc_buffer[ADC_BUFFER_SIZE];

	volatile uint16_t adc_data[3] = { 0 };
	volatile uint32_t adc0_length = 0;
	volatile uint32_t adc1_length = 0;
	volatile uint8_t  adc0_filled = 0;
	volatile uint8_t  adc1_filled = 1;
	volatile uint8_t  adc_available = 1;
	volatile uint8_t  adc_reset_cyccnt = 1;

	volatile uint16_t adc_max[2]    = {  0 };
	volatile uint16_t adc_min[2]    = { -1 };
	volatile uint32_t adc_period[2] = {  0 };
	volatile uint8_t  adc_period0_detected = 0;
	volatile uint8_t  adc_period1_detected = 0;

	uint32_t adc0_time[ADC_BUFFER_SIZE];
	uint32_t adc1_time[ADC_BUFFER_SIZE];
	uint16_t adc0[ADC_BUFFER_SIZE];
	uint16_t adc1[ADC_BUFFER_SIZE];
	uint8_t  adc_immediate = 1;

	uint32_t xlim_us = 200;
	uint32_t ylim_uV = 1000000;

	uint16_t cursor0 = 120;
	uint16_t cursor1 = 196;

	uint16_t trigger0 = 70;
	uint16_t trigger1 = 146;

	uint8_t  trigger_mode   = 0;
	uint16_t trigger0_value = 2048;
	uint16_t trigger1_value = 2048;

	uint8_t event_adc      = 0;
	uint8_t event_axis     = 1;
	uint8_t event_mode     = 1;
	uint8_t event_cursor   = 1;
	uint8_t event_trigger  = 1;
	uint8_t event_channel  = 1;
	uint8_t event_seconds  = 1;
	uint8_t event_voltage  = 1;
	uint8_t event_button0  = 0;
	uint8_t event_button1  = 0;
	uint8_t event_button2  = 0;
	uint8_t event_selector = 1;
	uint8_t event_trigger_mode = 1;
	uint8_t event_trigger0_detected = 1;
	uint8_t event_trigger1_detected = 1;


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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // Display init
  display.spi             = &hspi1;
  display.cs_gpio_port    = ILI9341_CS_GPIO_Port;
  display.dc_gpio_port    = ILI9341_DC_GPIO_Port;
  display.reset_gpio_port = ILI9341_RESET_GPIO_Port;
  display.cs_pin          = ILI9341_CS_Pin;
  display.dc_pin          = ILI9341_DC_Pin;
  display.reset_pin       = ILI9341_RESET_Pin;
  display.width           = 320;
  display.height          = 240;
  display.orientation     = ILI9341_ORIENTATION_ROTATE_RIGHT;

   ILI9341_UNSELECT(&display);
   ILI9341_Init(&display);
   char string[255];

   ILI9341_FillScreen(&display, ILI9341_BLUE);
   snprintf(string, 255, "Guitar Tuner");
   ILI9341_WriteString(&display, 60, 18 * 0, string, Font_16x26, ILI9341_GREEN, ILI9341_BLUE);
   drawAxis(&display);
   HAL_Delay(1000);

   // Start the timer before starting ADC
   HAL_TIM_Base_Start(&htim2);

   // Start ADC with DMA
   HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE);





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */


  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1599;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ILI9341_LED_GPIO_Port, ILI9341_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ILI9341_RESET_Pin|ILI9341_DC_Pin|ILI9341_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ILI9341_LED_Pin */
  GPIO_InitStruct.Pin = ILI9341_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ILI9341_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ILI9341_RESET_Pin ILI9341_DC_Pin ILI9341_CS_Pin */
  GPIO_InitStruct.Pin = ILI9341_RESET_Pin|ILI9341_DC_Pin|ILI9341_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	char string[255];
	char str[10];
	//snprintf(string, 255, "A");
	//sprintf(str, "%d", adc_buffer[0]);
	//ILI9341_WriteString(&display, 60, 18 * 5, str, Font_16x26, ILI9341_GREEN, ILI9341_BLUE);


	//fft
    float top_freq;
    float mag[ADC_BUFFER_SIZE];
    float freq[ADC_BUFFER_SIZE];
    complex fft_samples[ADC_BUFFER_SIZE], temp[ADC_BUFFER_SIZE];

    int j=0;
	for(j=0; j<ADC_BUFFER_SIZE; j++){
		fft_samples[j].Re=adc_buffer[j];
		fft_samples[j].Im=0;
	}

		fft(fft_samples, ADC_BUFFER_SIZE, temp);
    	get_freq_bins(freq, ADC_BUFFER_SIZE, 0.001);
		get_magnitudes(mag, fft_samples, ADC_BUFFER_SIZE);
		filter_frequencies(mag, freq, 0.0, 4.0, ADC_BUFFER_SIZE);
		filter_frequencies(mag, freq, 60.0, 5.0, ADC_BUFFER_SIZE);

		// Dot product of unit_vecs with signal fft
		#define E 0
		#define A 1
		#define D 2
		#define G 3
		#define B 4
		#define e 5
		int i;
		float dp[6] = {0};
		for(i=0; i<UNIT_VEC_LENGTH; i++)
		{
		float f = eigen_frequencies[i];
		int index = freq_to_index(f, 1000, ADC_BUFFER_SIZE);
		dp[E] += E_fft_unit_vec[i] * mag[index];
		dp[A] += A_fft_unit_vec[i] * mag[index];
		dp[D] += D_fft_unit_vec[i] * mag[index];
		dp[G] += G_fft_unit_vec[i] * mag[index];
		dp[B] += B_fft_unit_vec[i] * mag[index];
		dp[e] += e_fft_unit_vec[i] * mag[index];
		}

		// find max string
		float max_val = 0.0;
		int max_index;
		char string_char[10];

		for(i=0; i<6; i++){
			if(dp[i] > max_val){
				max_val = dp[i];
				max_index = i;
			}
		}


	    switch (max_index) {
	        case 0:
	        	snprintf(string_char, 10, "E");  // 6ª corda (mais grossa)
	            break;
	        case 1:
	        	snprintf(string_char, 10, "A");  // 5ª corda
	            break;
	        case 2:
	        	snprintf(string_char, 10, "D");  // 4ª corda
	            break;
	        case 3:
	        	snprintf(string_char, 10, "G");  // 3ª corda
	            break;
	        case 4:
	        	snprintf(string_char, 10, "B");  // 2ª corda
	            break;
	        case 5:
	        	snprintf(string_char, 10, "e");  // 1ª corda (mais fina)
	            break;
	    }
		ILI9341_WriteString(&display, 60, 18 * 5, string_char, Font_16x26, ILI9341_GREEN, ILI9341_BLUE);
}


static void drawSignal(ILI9341TypeDef *display, uint32_t *adc_time, uint16_t *adc0, uint32_t adc_length, uint16_t pixel_dirty[280][2], uint16_t cursor, uint16_t color)
{
	uint16_t point[280];
	for (uint16_t i = 0; i < 280; i++)
		point[i] = 0;

	for (uint16_t i = 0; i < adc_length; i++) {

		float uV = (float)(adc0[i]) * 3300000.0f / 4096.0f;
		uint16_t x = (float)(adc_time[i]) * 280.0f / (float)(12.0f * xlim_us);
		uint16_t y = cursor - ((uV / (float)(ylim_uV)) * 200.0f / 8.0f);

		if (x < 0)
			x = 0;

		if (x > 274)
			x = 274;

		if (y < 21)
			y = 21;

		if (y > 219)
			y = 219;

		point[x] += (float)(y - point[x]) * 1.0f;
	}

	uint16_t pixel[280][2];
	for (uint16_t i = 0; i < 280; i++) {
		pixel[i][0] = 220;
		pixel[i][1] = 20;
	}

	for (uint16_t i = 1; i <= 279; i++) {

		if (point[i] == 0)
			continue;

		int16_t x1 = i;
		int16_t x0 = x1 - 1;

		for (; x0 >= 0; x0--) {
			if (point[x0] != 0)
				break;
		}

		if (x0 == 0 && point[x0] == 0)
			return;

		int16_t y0 = point[x0];
		int16_t y1 = point[x1];

		int16_t dx = (x1 - x0) > 0 ? (x1 - x0) : -(x1 - x0);
		int16_t sx = x0 < x1 ? 1 : -1;
		int16_t dy = (y1 - y0) > 0 ? -(y1 - y0) : (y1 - y0);
		int16_t sy = y0 < y1 ? 1 : -1;
		int16_t error = dx + dy;

		while (1) {

			if (pixel[x0][0] > y0)
				pixel[x0][0] = y0;

			if (pixel[x0][1] < y0)
				pixel[x0][1] = y0;

			if (x0 == x1 && y0 == y1)
				break;

			int16_t e2 = 2 * error;

			if (e2 >= dy) {
				if (x0 == x1)
					break;

				error = error + dy;
				x0 = x0 + sx;
			}

			if (e2 <= dx) {
				if (y0 == y1)
					break;

				error = error + dx;
				y0 = y0 + sy;
			}
		}
	}

	for (uint16_t i = 1; i <= 279; i++) {
		uint16_t min = pixel[i][0] < pixel_dirty[i][0] ? pixel[i][0] : pixel_dirty[i][0];
		uint16_t max = pixel[i][1] > pixel_dirty[i][1] ? pixel[i][1] : pixel_dirty[i][1];

		for (uint16_t j = min; j <= max; j++) {
			uint8_t draw = 0;
			if (j >= pixel[i][0] && j <= pixel[i][1])
				draw = 1;

			uint8_t clear = 0;
			if (j >= pixel_dirty[i][0] && j <= pixel_dirty[i][1])
				clear = 1;

			if (draw && !clear && j > 21)
				ILI9341_DrawPixel(display, i + 20, j, color);

			if (!draw && clear) {
				if (((i % 25) == 0 && (j % 2) == 0) || ((i % 2) == 0 && ((j - 20) % 25) == 0))
					ILI9341_DrawPixel(display, i + 20, j,  ILI9341_GRAY);
				else
					ILI9341_DrawPixel(display, i + 20, j, ILI9341_BLACK);
			}
		}

		pixel_dirty[i][0] = pixel[i][0];
		pixel_dirty[i][1] = pixel[i][1];
	}
}

static void drawAxis(ILI9341TypeDef *display)
{
	for (uint16_t i = 0; i < 9; i++) {
		uint16_t y = 20 + 25 * i;

		if (i == 0 || i == 8) {
			ILI9341_FillRectangle(display, 20, y, 276, 1, ILI9341_WHITE);
			continue;
		}

		for (uint16_t j = 0; j < 276; j += 2)
			ILI9341_DrawPixel(display, 20 + j, y, ILI9341_GRAY);
	}

	for (uint16_t i = 0; i < 12; i++) {
		uint16_t x = 20 + 25 * i;

		if (i == 0 || i == 11) {
			ILI9341_FillRectangle(display, x, 20, 1, 200, ILI9341_WHITE);
			continue;
		}

		for (uint16_t j = 0; j < 200; j += 2)
			ILI9341_DrawPixel(display, x, 20 + j, ILI9341_GRAY);
	}
}




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
