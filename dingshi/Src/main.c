
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
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "DS1302.h"
#include "time.h"
#include "DHT11.h"
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
uint8_t button_time=0;//消抖计数器
uint8_t key_time=0; //按下按键的次数

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  struct tm gpstoutc;
  uint64_t  utc_time;//记录当前utc时间
  uint64_t  start_time; //记录开始计时的时间
  int timer = 0; //计时时长,单位S
  uint8_t timer_flag = 0;//计时标志位，是否开始计时,0处于非计时状态，1处于计时状态
  DHT11_Data_TypeDef DHT11_Data;//储存温度和湿度信息的结构体
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);//开启定时器中断
   Set_Ds1302_star();//启动DS1302

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

      
        gpstoutc.tm_sec  = DS1302_Get_Sec(); //获取秒
        gpstoutc.tm_min  = DS1302_Get_Min(); //获取分
        gpstoutc.tm_hour = DS1302_Get_Hour(); //获取时
        gpstoutc.tm_mday = DS1302_Get_Date(); //获取日期
        gpstoutc.tm_mon  = DS1302_Get_Mouth()-1; //获取月份
        gpstoutc.tm_year = DS1302_Get_Year()+100; //获取年份
        gpstoutc.tm_isdst= 0; //夏令时，必须是0，不然时间会差1小时
        utc_time  =  mktime(&gpstoutc); //将时间转化为时间戳
        DHT11_Read_TempAndHumidity(&DHT11_Data);//读取温度值
        if(((DHT11_Data.humidity-32)/1.8)>20)
        {
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);//温度大于20度，继电器关闭
        }
        if(key_time == 1) //按下一次按键,计时10S，亮一个灯
        {
          HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_RESET); //灯1亮
          HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_SET); //灯2灭
          HAL_GPIO_WritePin(GPIOB, LED3_Pin, GPIO_PIN_SET); //灯3灭
          if(timer_flag == 0)
          {
           start_time = utc_time; //记录开始计时的时间
           timer_flag = 1; //进入计时状态,标志位置1
          }
          timer = 10; //定时10s
        }
        if(key_time == 2) //按下两次按键,计时20S，亮两个灯
        {
          HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_RESET);//灯1亮
          HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);//灯2
          HAL_GPIO_WritePin(GPIOB, LED3_Pin, GPIO_PIN_SET);//灯3灭
          timer = 20; //定时20s
        }
        if(key_time == 3) //按下三次按键,计时30S，亮三个灯
        {
          HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_RESET);//灯1亮
          HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);//灯2亮
          HAL_GPIO_WritePin(GPIOB, LED3_Pin, GPIO_PIN_RESET);//灯3亮
          timer = 30;//定时30s
        }
        if(timer_flag == 1)
        {
          if((utc_time -  start_time) >= timer)//现在时间-开始计时的时间大于等于计时时长，退出计时,
          {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);//继电器状态翻转
            key_time = 4; //按键4状态就是退出计时
          }
        
        }
        if(key_time == 4) //按下四次按键,退出计时状态，所有数据清零，标志位置零,灯熄灭
        {
          timer_flag = 0;
          start_time = 0;
          timer = 0;
          key_time = 0;
          if(timer_flag == 0)
          HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_SET);//灯1灭
          HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_SET);//灯2灭
          HAL_GPIO_WritePin(GPIOB, LED3_Pin, GPIO_PIN_SET);//灯3灭
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
        
        if(htim->Instance == TIM2)
	{
          if(HAL_GPIO_ReadPin(GPIOB, KEY_Pin) == 1)
          {
            button_time++;
            if(button_time == 50)
            {
              key_time++;
            }
          }
          else button_time = 0;
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
