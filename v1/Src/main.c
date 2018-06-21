
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "ADE7953.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t irqstata[4];

uint8_t v[4];
uint8_t ia[4];
uint8_t awatt[4];
uint8_t avar[4];
uint8_t ava[4];
uint8_t pfa[2];
uint8_t period[2];

uint8_t aenergya[4];
uint8_t renergya[4];
uint8_t apenergya[4];

uint8_t samplei[280][3];

uint8_t cmd1=0x01;
uint8_t cmd2=0x02;
uint8_t cmd3=0x03;
uint8_t end1=0xfe;

uint16_t cnt1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	ADE7953CommuCfg();	//Pull Down SPI_CS
	HAL_Delay(1);
	ADE7953Reset();			//RESET ADE7953
	HAL_Delay(100);			//Delay to RESET complete	
	ADE7953SPILock();		//SPILOCK
	
	//For optimum performance, Register Address 0x120 must be configured by the user after powering up the ADE7953.
	//SPIWrite1Byte(0x0FE,0xAD);
	//SPIWrite2Bytes(0x0120,0x0030);

	ADE7953Cfg();
	
	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
	HAL_TIM_Base_Start_IT(&htim3);
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void send1()
{
	BSP_ADE7953_Read(v, VA, 4);
	BSP_ADE7953_Read(ia, IA, 4);
	BSP_ADE7953_Read(awatt, AWATT, 4);
	BSP_ADE7953_Read(avar, AVAR, 4);
	BSP_ADE7953_Read(ava, AVA, 4);
	BSP_ADE7953_Read(pfa, PFA, 2);
	BSP_ADE7953_Read(period, PERIOD, 2);
	
	HAL_UART_Transmit(&huart1, &cmd1, 1, 0xFFFF);
	HAL_UART_Transmit(&huart1, v, 4, 0xFFFF);
	HAL_UART_Transmit(&huart1, ia, 4, 0xFFFF);
	HAL_UART_Transmit(&huart1, awatt, 4, 0xFFFF);
	HAL_UART_Transmit(&huart1, avar, 4, 0xFFFF);
	HAL_UART_Transmit(&huart1, ava, 4, 0xFFFF);
	HAL_UART_Transmit(&huart1, pfa, 2, 0xFFFF);
	HAL_UART_Transmit(&huart1, period, 2, 0xFFFF);
	HAL_UART_Transmit(&huart1, &end1, 1, 0xFFFF);
}

void send2()
{
	BSP_ADE7953_Read(aenergya, AENERGYA, 4);
	BSP_ADE7953_Read(renergya, RENERGYA, 4);
	BSP_ADE7953_Read(apenergya, APENERGYA, 4);
	
	HAL_UART_Transmit(&huart1, &cmd2, 1, 0xFFFF);
	HAL_UART_Transmit(&huart1, aenergya, 4, 0xFFFF);
	HAL_UART_Transmit(&huart1, renergya, 4, 0xFFFF);
	HAL_UART_Transmit(&huart1, apenergya, 4, 0xFFFF);
	HAL_UART_Transmit(&huart1, &end1, 1, 0xFFFF);
}

void send3()
{
	HAL_UART_Transmit(&huart1, &cmd3, 1, 0xFFFF);
	for(uint16_t i=0; i<280; i++)
	{
		HAL_UART_Transmit(&huart1, samplei[i], 3, 0xFFFF);
	}
	HAL_UART_Transmit(&huart1, &end1, 1, 0xFFFF);
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim3.Instance)
    {
			send1();
    }
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_1)
  {
		if (BSP_ADE7953_Read(irqstata, RSTIRQSTATA, 4) != ADE7953_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}	 
		if((irqstata[2]&0x10)==0x10)
		{
			printf("ADE7953 OK\n");
		}	//clear reset interrupt
		else if((irqstata[2]&0x02)==0x02)
		{
			if (cnt1<280) 
			{
				BSP_ADE7953_Read(samplei[cnt1], 0x216, 3);
				cnt1++;
			}
			else
			{
				SPIWrite4Bytes(IRQENA,0x140038);
				cnt1=0;
				send3();
			}
		}	//clear sample interrupt
		else if((irqstata[2]&0x04)==0x04)
		{
			send2();
		} //clear cycend interrupt
		else if((irqstata[0]&0x08)==0x08 || (irqstata[0]&0x10)==0x10 || (irqstata[0]&0x20)==0x20)
		{
			send2();
		}	//clear EOF interrupt
  } 
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
		printf("\n\r Something Wrong!!!\n\r");
		return;
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
