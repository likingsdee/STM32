
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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "BY8301.h"
#include "LCD1602.h"
#include "gps.h"
#include "string.h"
#include "geofence.h"
//#include "PN532.h"
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
 uint8_t GPS_MSG[500];
 uint8_t data[100];
 int len = 0;
 int delay_1s = 0;
 nmea_msg gpsx;
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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  LCD_init(); //LCD1602初始化
  int i=0;
  uint8_t weak_up[]={0x55,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x03,0xFD,0xD4,0x14,0x01,0x17,0x00}; //唤醒指令
  uint8_t data2[]={0x00,0x00,0xFF,0x04,0xFC,0xD4,0x4A,0x01,0x00,0xE1,0x00};  //ID卡扫描指令
  uint8_t card_1[]={0x00,0x00,0xFF,0x0C,0xF4,0xD5,0x4B,0x01,0x01,0x00,0x04,0x08,0x04,0x10,0x58,0x4C,0x63,0xB7,0x00};//卡1卡号
  uint8_t card_2[]={0x00,0x00,0xFF,0x0C,0xF4,0xD5,0x4B,0x01,0x01,0x00,0x04,0x08,0x04,0x3C,0x6D,0x56,0x1B,0xB4,0x00};//卡2卡号
  char latitude[20];//存储经度的字符串
  char longitude[20];//存储纬度的字符串
  int scenic = 0;    //景点标志，0：不在任意景点 1：处于午门 2：处于太和殿
  double fixed_flag=0;
  double dis_wumen; //保存到午门的距离
  double dis_taihe; //保存到太和殿的距离
  double dis_kunning;//保存到坤宁宫的距离
  double wumen_lat = 38.859236;//午门经度
  double wumen_lon = 121.509745;//午门纬度
  double taihe_lat = 38.858910;
  double taihe_lon = 121.510443;
  double kunning_lat = 38.858489;
  double kunning_lon = 121.509663;
  LCD_write_cmd(0x01);
  HAL_UART_Receive_DMA(&huart1, UART1_TYPE.usartDMA_rxBuf, RECEIVELEN);  //开启串口1DMA接收
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart2, data, RECEIVELEN);//开启串口2DMA接收
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE); 
  HAL_TIM_Base_Start_IT(&htim1);
 // BY8301_Play_Voice(0);
    HAL_UART_Transmit(&huart2,weak_up,24,1000);
    HAL_Delay(300);
    BY8301_Play_Voice(1);
    while(1) //循环读取卡片信息，读取到正确的卡片信息才会跳出
    {

        HAL_UART_Transmit(&huart2,data2,11,1000); //发送读卡指令
        HAL_Delay(180);
        LCD_write_string(1,1,"    Welcome     ");
        for(i=0;UART2_TYPE.usartDMA_rxBuf[i] == card_2[i];i++){} //判断读取到的卡号是否与保存的一致
        if( UART2_TYPE.rx_len == 19 && i < 19) //检测到卡号不一致
        {
          LCD_write_string(1,1,"   ERROR CARD   "); //串口屏显示错误的卡
          BY8301_Play_Voice(3);  //播放语言，错误的卡号
          HAL_Delay(2000);
        }
        if(i >= 19)
          break;
    }
    BY8301_Play_Voice(2); //播放开机语言
   LCD_write_string(3,1,"   HELLO   "); //串口屏显示HELLO 
   HAL_Delay(2000);
   delay_1s=0;
 //    LCD_write_string(5,2,"shinobi");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  // HAL_Delay(10);
      
    if(UART1_TYPE.receive_flag ==1)  //串口1接收到消息
   {  
       for(i=0;i<68;i++)          //将受到的消息保存到一个字符串，便于解析
       {
         GPS_MSG[i]=UART1_TYPE.usartDMA_rxBuf[i];
       }
         NMEA_GNRMC_Analysis(&gpsx,GPS_MSG); //解析定位模块的消息
         sprintf(latitude,"%lf       ",(double)gpsx.latitude/10000000);   //经度转换成字符串形式
         sprintf(longitude,"%lf       ",(double)gpsx.longitude/10000000); //纬度转换成字符串形式

         memset(GPS_MSG, 0, sizeof(GPS_MSG)); 
   }
   if(gpsx.latitude!=0) //如果经度不为0，则定位成功
   {
    if(fixed_flag == 0)
    {
      fixed_flag = 1;
      BY8301_Play_Voice(4);
    
    }
    dis_wumen=get_distance((double)gpsx.latitude/10000000,(double)gpsx.longitude/10000000,wumen_lat,wumen_lon)*1000;//计算当前坐标到午门的距离
    dis_taihe=get_distance((double)gpsx.latitude/10000000,(double)gpsx.longitude/10000000,taihe_lat,taihe_lon)*1000;//计算当前坐标到太和殿的距离
    dis_kunning=get_distance((double)gpsx.latitude/10000000,(double)gpsx.longitude/10000000,kunning_lat,kunning_lon)*1000;
    if(dis_wumen<=30&&scenic != 1) //距离午门的距离小于30米
    {
        BY8301_Play_Voice(6); //播放午门的语言
        scenic = 1;   //位置标志置1，保证一个点只播放一次语音
    }
     if(dis_taihe<=30&&scenic != 2)//距离太和殿的距离小于30米
     {
        BY8301_Play_Voice(7);
        scenic = 2;
     }
     if(dis_kunning<=30&&scenic != 3)//距离坤宁宫的距离小于30米
     {
        BY8301_Play_Voice(8);
        scenic = 3;
     }

       if(scenic == 1 && delay_1s >=3)
   {
      LCD_write_string(1,1,"    welcome to   ");
      LCD_write_string(1,2,"      wumen      ");
   }
   if(scenic == 2 && delay_1s >=3)
   {
      LCD_write_string(1,1,"    welcome to   ");
      LCD_write_string(1,2,"    taihedian    ");
   }
      if(scenic == 3 && delay_1s >=3)
   {
      LCD_write_string(1,1,"    welcome to   ");
      LCD_write_string(1,2,"    kunninggong    ");
   }
   if(delay_1s<3)
   {
         LCD_write_string(1,1,latitude);  //显示屏打印经度
         LCD_write_string(1,2,longitude); //显示屏打印纬度
   }
  }

   if(gpsx.latitude == 0) //定位的坐标为0，这是没有信号
   {
  //  fixed_flag = 0;
    LCD_write_string(1,1,"    No Signal    ");
    LCD_write_string(1,2,"                 ");
   }

 //  }
 //  dis=get_distance()

   //if(UART2_TYPE.rx_len == 19)
  //   LCD_write_string(3,1,"   card   ");
  // else  LCD_write_string(3,1,"No card");
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
void UsartReceive_IDLE(UART_HandleTypeDef *huart)  
{  
    uint32_t temp;  
    
    if((__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET))  
    {   
        __HAL_UART_CLEAR_IDLEFLAG(huart);             
        temp = huart->hdmarx->Instance->CNDTR;  
        HAL_UART_DMAStop(huart); 
        if(huart->Instance == USART1)
	{
            UART1_TYPE.rx_len =  RECEIVELEN - temp; 
            UART1_TYPE.receive_flag=  1; 
            HAL_UART_Receive_DMA(&huart1,UART1_TYPE.usartDMA_rxBuf,RECEIVELEN);
	}
        else if(huart->Instance == USART2)
        {   
            UART2_TYPE.rx_len =  RECEIVELEN - temp;   
            UART2_TYPE.receive_flag = 1;           
            HAL_UART_Receive_DMA(&huart2,UART2_TYPE.usartDMA_rxBuf,RECEIVELEN);
        }

    }  
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
        
        if(htim->Instance == TIM1)
	{
          delay_1s++;
          if(delay_1s>=5)
            delay_1s=0;
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
