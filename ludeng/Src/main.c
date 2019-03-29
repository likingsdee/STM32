
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
#include "adc.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "lcd1602.h"
#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

u16 Get_Adc()//��ȡADת����ֵ
{
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,10);
	return HAL_ADC_GetValue(&hadc1);
}
u16 Get_Adc_Average(u8 times)//��������Ĳ����������������ȡƽ��ֵ
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc();
	}
	return temp_val/times;
} 
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
  MX_GPIO_Init();//IO�ڳ�ʼ��
  MX_ADC1_Init();//ADC��ʼ��
  MX_USART1_UART_Init();//UART_1����debug��
  /* USER CODE BEGIN 2 */
  float adcx;//����ADת���Ľ��
  int recharging_flag=0;//���״̬���
  int electricity_flag=0;//������ʾ״̬���
  HAL_ADCEx_Calibration_Start(&hadc1);//ADCУ׼����У׼��ֵ����ƫ��
  LCD_init();
 // LCD_write_string(1,1,"Welcome to use!");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    adcx=(2*(Get_Adc_Average(10)*3.3/4096));//���غ��Լ��0.2V��ѹ��
    if(recharging_flag == 0)//��⵽�����ڳ��״̬����ʾ����
    {
      if(electricity_flag == 0)
      {
        electricity_flag = 1;
        LCD_write_cmd(0x01);         //����
        if(adcx>4.20)
        {
          LCD_write_string(3,1,"Dump Energy");
          LCD_write_string(5,2,"100%");
        }
        else if(adcx<=4.20 && adcx >4.06)     //���ݵ���ʣ��״������ʾ����
        {
          LCD_write_string(3,1,"Dump Energy");
          LCD_write_string(5,2," 95%");
        }
        else if(adcx<=4.06 && adcx >3.98)
        {
          LCD_write_string(3,1,"Dump Energy");
          LCD_write_string(5,2," 86%");
        }
        else if(adcx<=3.98 && adcx >3.92)
        {
          LCD_write_string(3,1,"Dump Energy");
          LCD_write_string(5,2," 78%");
        }
         else if(adcx<=3.92 && adcx >3.87)
        {
          LCD_write_string(3,1,"Dump Energy");
          LCD_write_string(5,2," 69%");
        }
         else if(adcx<=3.87 && adcx >3.82)
        {
          LCD_write_string(3,1,"Dump Energy");
          LCD_write_string(5,2," 64%");
        }
        else if(adcx<=3.82 && adcx >3.79)
        {
          LCD_write_string(3,1,"Dump Energy");
          LCD_write_string(5,2," 50%");
        }
        else if(adcx<=3.79 && adcx >3.77)
        {
          LCD_write_string(3,1,"Dump Energy");
          LCD_write_string(5,2," 43%");
        }
        else if(adcx<=3.77 && adcx >3.74)
        {
          LCD_write_string(3,1,"Dump Energy");
          LCD_write_string(5,2," 33%");
        }
        else if(adcx<=3.74 && adcx >3.68)
        {
          LCD_write_string(3,1,"Dump Energy");
          LCD_write_string(5,2," 21%");
        }
        else if(adcx<=3.68 && adcx >3.45)
        {
          LCD_write_string(3,1,"Dump Energy");
          LCD_write_string(5,2," 12%");
        }
        else if(adcx<=3.45 )
        {
          LCD_write_string(3,1,"Dump Energy");
          LCD_write_string(5,2,"  5%");
        }
      }
    
    
    
    
    
    }
    /****************PB11Ϊ����������ţ��͵�ƽ״̬��ʾ������*******************/
    if(HAL_GPIO_ReadPin( GPIOB,GPIO_PIN_11) == 0&&recharging_flag == 0 )//�����⵽�����������Ҳ����ڳ��״̬
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_RESET);
      HAL_Delay(5000);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_SET);
    
    }
    /****************PB6Ϊ����ѹ���룬�ߵ�ƽ״̬��ʾ�����ڳ��*******************/
    if(HAL_GPIO_ReadPin( GPIOB,GPIO_PIN_6) == 1)//��⵽���״̬����ʾ�����
    {
      if(recharging_flag == 0)
      {
        electricity_flag = 0;
        recharging_flag = 1;
        LCD_write_cmd(0x01);         //����
        LCD_write_string(1,1,"  Recharging...");
      }
    
    }
    else recharging_flag = 0;

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
