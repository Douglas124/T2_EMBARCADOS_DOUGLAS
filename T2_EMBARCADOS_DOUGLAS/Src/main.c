/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "gpio.h"
//#include "adc.h"
//#include "rtc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "string.h"
#include "stdio.h"

#define SDA (1<<10) //PA.10 - PLACA stm32f103
#define SCL (1<<11) //PA.11 - PLACA stm32f103
#define SDA0 GPIOA->BSRR = 1<<(10+16)    // bit set/reset register 32 bit- 16 primeiros set, 16 ultimos reset: [0000010000000000 0000000000000000]
#define SDA1 GPIOA->BSRR = SDA					 // bit set/reset register 32 bit- 16 primeiros set, 16 ultimos reset: [0000000000000000 0000010000000000]
#define SCL0 GPIOA->BSRR = 1<<(11+16)
#define SCL1 GPIOA->BSRR = SCL


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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//--------------------------------------------------- MANDA COMANDO PARA O LCD
	void lcd_comando(uint8_t comando){
	HAL_GPIO_WritePin(GPIOA, RS_Pin,0);  //RS em zero para mandar comando
	GPIOA->BRR = 0xFF; //zera os bits de dados
	GPIOA->BSRR = comando; //seta os bits do comando
	
	HAL_GPIO_WritePin(GPIOA, EN_Pin,1);			//en 1
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, EN_Pin,0);			//en 0
	HAL_Delay(1);
	}
	
//--------------------------------------------------- DESLOCA CURSOR LCD
	void lcd_GOTO(int linha, int coluna){
		if (coluna<16){
		if (linha==0)lcd_comando(0x80+coluna);
		if (linha==1)lcd_comando(0xc0+coluna);
		if (linha==2)lcd_comando(0x94+coluna);
		if (linha==3)lcd_comando(0xD4+coluna);
		}
	}
	
	
//--------------------------------------------------- MANDA DADOS PARA ESCREVER NO LCD
	void lcd_dado(char dado){
	HAL_GPIO_WritePin(GPIOA, RS_Pin,1);  //RS em zero para mandar dados
	GPIOA->BRR = 0xFF; //zera os bits de dados
	GPIOA->BSRR = dado; //seta os bits do dado
	
	HAL_GPIO_WritePin(GPIOA, EN_Pin,1);			//en 1
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, EN_Pin,0);			//en 0
	HAL_Delay(1);
	}
	

//--------------------------------------------------- ESCREVE STRINGS NO LCD
	void lcd_STRING(char vetor[]){
		int tamanho, i=0;
		tamanho = strlen(vetor);
		for(i=0;i<=tamanho;i++){
			lcd_dado(vetor[i]);
		}
	}
	

//--------------------------------------------------- INICIALIZAÇAO DO LCD	
	void lcd_init(void){
		lcd_comando(0x38);
		lcd_comando(0x0e);
		lcd_comando(0x06);
		lcd_comando(0x01);
	  HAL_Delay(100);
	}
	
//--------------------------------------------------- APAGA TUDO NO DISLAY
	void lcd_clear(){
	int i=0, j=0;
		for (i=0; i<4;i++){
			for (j=0; j<16;j++){
				lcd_GOTO(i,j);
				lcd_dado(' ');
			}
		}
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
  /* USER CODE BEGIN 2 */
	
		lcd_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
	lcd_GOTO(0,0);
	lcd_STRING("testetetstetsa");
		
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
