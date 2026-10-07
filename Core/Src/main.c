/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
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

/* USER CODE BEGIN PV */
#define f_carrier     25000
#define f_fundamental 50
#define TIMER_PERIOD  1439   // center-aligned: 72 MHz / (2 x 1439) ~= 25 kHz

const float PI = M_PI;

int Duty;
int k = 0;
int phase = 0;
int sampleNum;

float radVal;
float sineValue[1000];
float duty_coeff = 0.9f;

#define MA_MAX   0.90f   // max modulation index
#define MA_DEADB 0.03f   // below this, output off (pulses shorter than dead time are lost anyway)

float pot_filt = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Soft start: ramps duty over ~2s so you don't slam gate drivers/MOSFETs
 * with full duty the instant TIM1 starts. Harmless to leave in for a
 * scope-only test. Set soft_start_mult = 1.0f directly if you want to
 * skip the ramp while probing. */
float soft_start_mult = 0.0f;
const float soft_start_step = 0.00002f;

void SystemClock_Config(void);

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
	MX_TIM1_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */

	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start(&hadc1);        // continuous mode: it keeps converting
	HAL_Delay(10);

	/* Precompute one half-cycle of the sine table */

	sampleNum = (int) (f_carrier / f_fundamental) / 2; // 250
	radVal = 1.0f * PI / (float) sampleNum;

	for (int i = 1; i <= sampleNum; i++)
	{
		sineValue[i] = sinf(radVal * (float) i);
	}
	sineValue[sampleNum] = 0.0f;

	/* Start PWM on CH1 (PA8) and complementary CH1N (PA7) */

	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		uint32_t raw = HAL_ADC_GetValue(&hadc1);   // latest conversion, 0..4095
		pot_filt += 0.05f * ((float) raw - pot_filt); // low-pass, ~100 ms time constant

		float ma = (pot_filt / 4095.0f) * MA_MAX;
		if (ma < MA_DEADB)
			ma = 0.0f;

		duty_coeff = ma;                // ISR_SINE picks it up on the next tick
		HAL_Delay(5);

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit =
	{ 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
/* Called from TIM1_UP_IRQHandler() in stm32f1xx_it.c, AFTER HAL_TIM_IRQHandler() */
void ISR_SINE(void)
{
	k++;

	/* Soft start ramp */
	if (soft_start_mult < 1.0f)
	{
		soft_start_mult += soft_start_step;
	}
	else
	{
		soft_start_mult = 1.0f;
	}

	Duty = (int) (duty_coeff * TIMER_PERIOD * sineValue[k] * soft_start_mult);

	/* Beginning of half cycle: set phase direction pins + first CCR1 */
	if (k == 1)
	{
		if (phase == 0)
		{
			GPIOA->BRR = GPIO_PIN_9;  // HB OFF
			GPIOB->BSRR = GPIO_PIN_0;  // LB ON
			TIM1->CCR1 = Duty;
		}
		else
		{
			GPIOB->BRR = GPIO_PIN_0;  // LB OFF
			GPIOA->BSRR = GPIO_PIN_9;  // HB ON
			TIM1->CCR1 = TIMER_PERIOD - Duty;
		}
		TIM1->BDTR |= TIM_BDTR_MOE; // enable outputs
	}

	/* Mid half-cycle: just update CCR1 each ISR tick */
	if (k > 1 && k <= sampleNum)
	{
		if (phase == 0)
			TIM1->CCR1 = Duty;
		else
			TIM1->CCR1 = TIMER_PERIOD - Duty;
	}

	/* Zero crossing: kill outputs, flip phase, pre-load next CCR1 */
	if (k > sampleNum)
	{
		int nextDuty = (int) (duty_coeff * TIMER_PERIOD * sineValue[1]
				* soft_start_mult);

		if (phase == 0)
			TIM1->CCR1 = TIMER_PERIOD - nextDuty;
		else
			TIM1->CCR1 = nextDuty;

		k = 0;

		TIM1->BDTR &= ~(TIM_BDTR_MOE); // instantly stop outputs (no shoot-through glitch)
		GPIOA->BRR = GPIO_PIN_9;
		GPIOB->BRR = GPIO_PIN_0;

		phase = !phase;
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
