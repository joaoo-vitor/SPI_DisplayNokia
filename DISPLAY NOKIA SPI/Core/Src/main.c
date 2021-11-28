/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stm32f4xx.h"
#include <stdio.h>
#include "LCD.h"
#include "imagens.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DISP_DELAY 4000

/* tempo em segundos em que o código ficará em um
 * estado da máquina de estados (clock do tim11 foi
 * configurado para ter período de 1s)
 */
#define DELAY_TELA 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//!Handler do Display Nokia5110
LCD_HandleTypeDef hlcd;

//!flag e string para testar callback "adiconal"
static uint16_t conta = 0;
//!msg para ser escrita mostrando o valor de "conta"
char msg[50];

uint8_t estado_display = 0;
/*variável que diz qual das três telas irá começar aparecendo
 (ela será incrementada depois)*/
uint8_t tela = 1;
//!tempo em segundos que uma das três telas ficará aparecendo
uint16_t delayTela = DELAY_TELA;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_USART2_UART_Init();
	MX_DMA_Init();
	MX_SPI3_Init();
	MX_TIM11_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim11);
	//Inicialização da Instancia HLCD
	hlcd.hspi = &hspi3;
	hlcd.CS_Port = NK_CS_GPIO_Port;
	hlcd.CS_Pin = NK_CS_Pin;
	hlcd.DC_Port = NK_DO_GPIO_Port;
	hlcd.DC_Pin = NK_DO_Pin;
	hlcd.RST_Port = NK_RST_GPIO_Port;
	hlcd.RST_Pin = NK_RST_Pin;
	hlcd.modo = LCD_DMA;

	LCD5110_init(&hlcd);

	LCD5110_set_XY(0, 0); ///! primeira função n precisa repetir
	while (LCD5110_write_string("Carregando...") != HAL_OK)
		;
	HAL_Delay(1500);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
	}

	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM11) {
		/*<<<<<<<<<<<<<MAQUINAS DE ESTADO DISPLAY>>>>>>>>>>>>>>*/
		switch (estado_display) {
		//primeiro case é para delay
		case 0:
			delayTela--; //diminui um segundo da contagem do tempo DELAY_TELA
			if (delayTela == 0) {
				delayTela = DELAY_TELA;
				//! caso DELAY_TELA ja tenha passado, vai pro proximo estado
				estado_display++;
			}
			break;

			//case 1 é para clear
		case 1:
			if (LCD5110_clear() == HAL_OK) {
				estado_display++;
			}
			break;

			//case 2 é para definir sequencia do que aparece no display
		case 2:
			if (LCD5110_set_XY(0, 0) == HAL_OK) {
				/*"tela" define onde o display irá começar
				 * 1 - case 3 (teste callback)
				 * 2 - case 4 (alfabeto e numeros)
				 * 3 - case 5 (primeira figura)
				 * 4 - case 6 (segunda figura)
				 */
				estado_display += tela;
			}
			break;

			//printa o aflabeto maiuculo, minusculo e numeros de 0 a 9
		case 3:
			sprintf(msg, "ABCdefGHIjklMNOpqrSTUvwxYZ1234567890");
			if (LCD5110_write_string(msg) == HAL_OK) {
				//volta pro delay (case 0)
				estado_display = 0;
				//incrementa a tela para atualizar (depois) o estado_display
				tela ++;
			}
			break;

			//testa callback adicional do usuário (fora da biblioteca)
		case 4:
			sprintf(msg, "Ja foram %d callbacks do SPI", conta);
			if (LCD5110_write_string(msg) == HAL_OK) {
				//volta pro delay (case 0)
				estado_display = 0;
				tela++;
			}
			break;

			//manda a primeira figura
		case 5:
			if (LCD5110_write_block(monalisa, TAM_TELA) == HAL_OK) {
				//volta pro delay (case 0)
				estado_display = 0;
				//incrementa a tela para atualizar (depois) o estado_display
				tela++;
			}
			break;

			//manda a segunda figura
		case 6:
			if (LCD5110_write_block(liber_bmp3, TAM_TELA) == HAL_OK) {
				//volta pro delay (case 0)
				estado_display = 0;
				//volta pra primeira tela
				tela = 1;
			}
			break;
		}
	}

	/*START CÓDIGO DO USUÁRIO*/
	/* Aqui o usuário pode escrever o código independente do LCD5110*/

	/*END CÓDIGO DO USUÁRIO*/
}
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {

	/*
	 * chama o callback da biblioteca display nokia, caso
	 * for o spi utilizado nela
	 */
	if (hspi->Instance == hlcd.hspi->Instance)
		LCD5110_TxCpltCallback(hspi);

	/*START CÓDIGO DO USU�?RIO*/
	/* Aqui o usuário pode escrever o código independente do LCD5110*/
	conta++; //conta quantas vezes o callback foi chamado
	/*END CÓDIGO DO USU�?RIO*/
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
