/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "fonts.h"
#include "GPS.h"
#include "fatfs_sd.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOG_PERIOD 10000
#define MAX_LOG_PERIOD 60000
#define MULTIPLIER MAX_LOG_PERIOD/LOG_PERIOD
#define CONVERSION_FACTOR 0.006315
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t cpm, gcTicks = 0;
float microSievert;

typedef enum {OFF, PRESSED} btn_t;
btn_t modeBtn = OFF;
btn_t setBtn = OFF;

typedef enum {RADIATION, TIMEDATE, GPSINFO, ALL, SETTIMEZONE, STANDBY} screenMode_t;
screenMode_t mode = TIMEDATE;

int8_t utcHOffset = 1;
uint32_t secTick = 0;
uint8_t hour = 0, minute = 0, second = 0;

char screenText[64];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM10_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void geigerCounter();
void readgpsTime();
void setTimeZone();

void radiationScreen();
void timedateScreen();
void gpsinfoScreen();
void generalScreen();
void standBy();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	MX_I2C1_Init();
	MX_SPI1_Init();
	MX_TIM10_Init();
	MX_USART2_UART_Init();
	MX_FATFS_Init();
	/* USER CODE BEGIN 2 */

	GPS_Init();
	SSD1306_Init();

	SSD1306_Fill (0);  // fill display with black color

	SSD1306_GotoXY (1, 30);
	SSD1306_Puts ("Initializing", &Font_7x10, 1);
	SSD1306_UpdateScreen ();  // update display
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		geigerCounter(); // read radiation sensor
		GPS_Process(); // read and parse data from GPS
		readgpsTime(); // read and parse time from GPS data
		switch(mode){
		case ALL:
			generalScreen();
			if(setBtn == PRESSED){
				setBtn = OFF;
			}
			if(modeBtn == PRESSED){
				modeBtn = OFF;
				mode = RADIATION;
			}
			break;
		case RADIATION:
			radiationScreen();

			if(setBtn == PRESSED){
				setBtn = OFF;
			}
			if(modeBtn == PRESSED){
				modeBtn = OFF;
				mode = TIMEDATE;
			}
			break;
		case TIMEDATE:
			timedateScreen();

			if(setBtn == PRESSED){
				setBtn = OFF;
				mode = SETTIMEZONE;
			}
			if(modeBtn == PRESSED){
				modeBtn = OFF;
				mode = GPSINFO;
			}
			break;

		case GPSINFO:
			gpsinfoScreen();

			if(setBtn == PRESSED){
				setBtn = OFF;
			}
			if(modeBtn == PRESSED){
				modeBtn = OFF;
				mode = STANDBY;
			}
			break;

		case STANDBY:
			standBy();

			mode = ALL;
			break;

		case SETTIMEZONE:
			setTimeZone();

			mode = RADIATION;
			break;
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		HAL_Delay(1000);
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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 84;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

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
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
 * @brief TIM10 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM10_Init(void)
{

	/* USER CODE BEGIN TIM10_Init 0 */

	/* USER CODE END TIM10_Init 0 */

	/* USER CODE BEGIN TIM10_Init 1 */

	/* USER CODE END TIM10_Init 1 */
	htim10.Instance = TIM10;
	htim10.Init.Prescaler = 1343;
	htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim10.Init.Period = 62499;
	htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM10_Init 2 */

	/* USER CODE END TIM10_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

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
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, STATUS_LED_Pin|CARD_SELECT_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : STATUS_LED_Pin CARD_SELECT_Pin */
	GPIO_InitStruct.Pin = STATUS_LED_Pin|CARD_SELECT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : GPS_FIX_Pin */
	GPIO_InitStruct.Pin = GPS_FIX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPS_FIX_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PPS_Pin */
	GPIO_InitStruct.Pin = PPS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(PPS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : CARD_DETECT_Pin */
	GPIO_InitStruct.Pin = CARD_DETECT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(CARD_DETECT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : GC_INT_Pin */
	GPIO_InitStruct.Pin = GC_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GC_INT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SET_BTN_Pin MODE_BTN_Pin */
	GPIO_InitStruct.Pin = SET_BTN_Pin|MODE_BTN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GC_INT_Pin){
		gcTicks++;
	}
	if(GPIO_Pin == MODE_BTN_Pin){
		modeBtn = PRESSED;
	}
	if(GPIO_Pin == SET_BTN_Pin){
		setBtn = PRESSED;
	}
	if(GPIO_Pin == GPS_FIX_Pin)
		HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin); // status led shows GPS fix status
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	GPS_CallBack();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim == &htim10)
		secTick++;
}

void geigerCounter(){
	uint32_t currentTick = HAL_GetTick();
	static uint32_t previousTick = 0;

	if (currentTick - previousTick > LOG_PERIOD){
		previousTick = currentTick;
		cpm = gcTicks * MULTIPLIER;
		microSievert = cpm * CONVERSION_FACTOR;
		gcTicks = 0;
	}
}

void readgpsTime(){
	int8_t tmpHour = (GPS.GPRMC.UTC_Hour + utcHOffset);

	if(tmpHour < 0)
		hour = 24 + tmpHour;
	else if(tmpHour > 23)
		hour = 0 + tmpHour;
	else
		hour = tmpHour;

	minute = GPS.GPRMC.UTC_Min;
	second = GPS.GPRMC.UTC_Sec;
}

void setTimeZone(){
	HAL_TIM_Base_Start_IT(&htim10);
	SSD1306_Clear();
	SSD1306_GotoXY(10, 20);
	sprintf(screenText, "UTC%+d", utcHOffset);
	SSD1306_Puts(screenText, &Font_16x26, 1);
	SSD1306_UpdateScreen();

	while (secTick < 10){
		if (modeBtn == PRESSED){
			modeBtn = OFF;
			HAL_TIM_Base_Stop_IT(&htim10);
			secTick = 0;
			return;
		}

		if(setBtn == PRESSED){

			setBtn = OFF;
			utcHOffset++;
			if(utcHOffset > 12 && utcHOffset > -12)
				utcHOffset = -12;
			if(utcHOffset < -12)
				utcHOffset = 12;

			SSD1306_Clear();
			SSD1306_GotoXY(10, 20);
			sprintf(screenText, "UTC%+d", utcHOffset);
			SSD1306_Puts(screenText, &Font_16x26, 1);
			SSD1306_UpdateScreen();
			secTick = 0;
		}

		HAL_Delay(500);
	}
	HAL_TIM_Base_Stop_IT(&htim10);
	secTick = 0;
}

void radiationScreen(){
	SSD1306_Clear();
	sprintf(screenText, "CPM:%d", cpm);
	SSD1306_GotoXY(1, 1);
	SSD1306_Puts(screenText, &Font_16x26, 1);
	sprintf(screenText, "uSv/h:%0.3f", microSievert);
	SSD1306_GotoXY(1, 30);
	SSD1306_Puts(screenText, &Font_11x18, 1);
	SSD1306_DrawLine(1, 51, 128, 51, 1);
	sprintf(screenText, "Time: %02d:%02d:%02d", hour, minute, second);
	SSD1306_GotoXY(1, 53);
	SSD1306_Puts(screenText, &Font_7x10, 1);
	SSD1306_UpdateScreen();
}

void timedateScreen(){

	SSD1306_Clear();
	sprintf(screenText, "UTC%+d", utcHOffset);
	SSD1306_GotoXY(1, 1);
	SSD1306_Puts(screenText, &Font_11x18, 1);
	sprintf(screenText, "%02hd:%02d:%02d", hour, minute, second);
	SSD1306_GotoXY(1, 17);
	SSD1306_Puts(screenText, &Font_11x18, 1);
	sprintf(screenText, "%02d-%02d-20%02d", GPS.GPRMC.Date, GPS.GPRMC.Month, GPS.GPRMC.Year);
	SSD1306_GotoXY(1, 32);
	SSD1306_Puts(screenText, &Font_11x18, 1);
	SSD1306_DrawLine(1, 51, 128, 51, 1);
	sprintf(screenText, "uSv/h: %0.3f", microSievert);
	SSD1306_GotoXY(1, 53);
	SSD1306_Puts(screenText, &Font_7x10, 1);
	SSD1306_UpdateScreen();
}

void gpsinfoScreen(){
	SSD1306_Clear();
	sprintf(screenText, "Lat:  %c%f", GPS.GPGGA.EW_Indicator, GPS.GPGGA.LatitudeDecimal);
	SSD1306_GotoXY(1, 1);
	SSD1306_Puts(screenText, &Font_7x10, 1);
	sprintf(screenText, "Long: %c%f", GPS.GPGGA.NS_Indicator, GPS.GPGGA.LongitudeDecimal);
	SSD1306_GotoXY(1, 10);
	SSD1306_Puts(screenText, &Font_7x10, 1);
	sprintf(screenText, "Sats: %d", GPS.GPGGA.SatellitesUsed);
	SSD1306_GotoXY(1, 20);
	SSD1306_Puts(screenText, &Font_7x10, 1);
	switch(GPS.GPGGA.PositionFixIndicator){
	case 0:
		SSD1306_GotoXY(1, 30);
		SSD1306_Puts("Fix:  N/A", &Font_7x10, 1);
		break;
	case 1:
		SSD1306_GotoXY(1, 30);
		SSD1306_Puts("Fix:  2D", &Font_7x10, 1);
		break;
	case 2:
		SSD1306_GotoXY(1, 30);
		SSD1306_Puts("Fix:  3D", &Font_7x10, 1);
	}
	sprintf(screenText, "Speed:%0.1f km/h", GPS.GPRMC.GroundSpeed * 1.852011);
	SSD1306_GotoXY(1, 40);
	SSD1306_Puts(screenText, &Font_7x10, 1);
	SSD1306_DrawLine(1, 51, 128, 51, 1);
	sprintf(screenText, "uSv/h: %0.3f", microSievert);
	SSD1306_GotoXY(1, 53);
	SSD1306_Puts(screenText, &Font_7x10, 1);
	SSD1306_UpdateScreen();
}

void generalScreen(){
	SSD1306_Clear();
	sprintf(screenText, "CPM:  %d", cpm);
	SSD1306_GotoXY(1, 1);
	SSD1306_Puts(screenText, &Font_7x10, 1);
	sprintf(screenText, "Time: %02d:%02d:%02d", hour, minute, second);
	SSD1306_GotoXY(1, 10);
	SSD1306_Puts(screenText, &Font_7x10, 1);
	sprintf(screenText, "Lat:  %c%f", GPS.GPGGA.EW_Indicator, GPS.GPGGA.LatitudeDecimal);
	SSD1306_GotoXY(1, 20);
	SSD1306_Puts(screenText, &Font_7x10, 1);
	sprintf(screenText, "Long: %c%f", GPS.GPGGA.NS_Indicator, GPS.GPGGA.LongitudeDecimal);
	SSD1306_GotoXY(1, 30);
	SSD1306_Puts(screenText, &Font_7x10, 1);
	SSD1306_DrawLine(1, 51, 128, 51, 1);
	sprintf(screenText, "uSv/h: %0.3f", microSievert);
	SSD1306_GotoXY(1, 53);
	SSD1306_Puts(screenText, &Font_7x10, 1);
	SSD1306_UpdateScreen();
}

void standBy(){
	SSD1306_Clear();
	SSD1306_GotoXY(20, 20);
	SSD1306_Puts("Entering", &Font_11x18, 1);
	SSD1306_GotoXY(25, 40);
	SSD1306_Puts("standby", &Font_11x18, 1);
	SSD1306_UpdateScreen();

	HAL_TIM_Base_Start_IT(&htim10);
	while(secTick<6){
		if(setBtn == PRESSED){
			setBtn = OFF;
			HAL_TIM_Base_Stop_IT(&htim10);
			secTick = 0;
			return;
		}

		if(modeBtn == PRESSED){
			modeBtn = OFF;
			HAL_TIM_Base_Stop_IT(&htim10);
			secTick = 0;
			return;
		}
	}
	HAL_TIM_Base_Stop_IT(&htim10);
	secTick = 0;
	SSD1306_Clear();
	while(1){
		if(setBtn == PRESSED){
			setBtn = OFF;
			break;
		}

		if(modeBtn == PRESSED){
			modeBtn = OFF;
			break;
		}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
