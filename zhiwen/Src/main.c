
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "bsp_EEPROM.h"
#include "HW_key.h"
#include "as608.h"
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
uint8_t I2c_Buf_Write[256]={0};
uint8_t I2c_Buf_Read[256]={0};
uint8_t key[8];
SysPara AS608Para;//指纹模块AS608参数
u16 ValidN;//模块内有效指纹个数
void Add_FR(void);	//录指纹
void Del_FR(void);	//删除指纹
void press_FR(void);//刷指纹
//显示确认码错误信息
void ShowErrMessage(u8 ensure)
{
//	LCD_Fill(0,120,lcddev.width,160,WHITE);
//	Show_Str_Mid(0,120,(u8*)EnsureMessage(ensure),16,240);
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
  u8 ensure;
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
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
 __HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);//开启接收中断
// HAL_TIM_Base_Start_IT(&htim3);
//  uint16_t i;
//  	for ( i=0; i<256; i++ ) //填充缓冲
//  {
//    I2c_Buf_Read[i]=0;      // 清空接收缓冲区
//    I2c_Buf_Write[i] = i;   // 为发送缓冲区填充数据
//    printf("0x%02X ", I2c_Buf_Write[i]);
//    if(i%16 == 15)    
//        printf("\n");
//   }
//  for(i=0;i<256;i+=8)
//  {
//    I2C_EEPROM_WriteBuffer(EEPROM_I2C_ADDRESS,i,I2C_MEMADD_SIZE_8BIT,&I2c_Buf_Write[i],8);
//    HAL_Delay(5);// 短延时不能少
//  }
//   
//   printf("读出的数据:\n");
//   I2C_EEPROM_ReadBuffer(EEPROM_I2C_ADDRESS,0,I2C_MEMADD_SIZE_8BIT,&I2c_Buf_Read[0],256);
//   for (i=0;i<256;i++)
//	 {    
//    if(I2c_Buf_Read[i] != I2c_Buf_Write[i])
//		{
//			printf("0x%02X ", I2c_Buf_Read[i]);
//			printf("错误:I2C EEPROM写入与读出的数据不一致\n\r");
//			break;
//		}
//    printf("0x%02X ", I2c_Buf_Read[i]);
//    if(i%16 == 15)    
//        printf("\n");
//	}
//  if(i==256)
//  {
//    printf("EEPROM(AT24C02)读写测试成功\n\r");
//  }
       	while(PS_HandShake(&AS608Addr))//与AS608模块握手
	{	
          HAL_Delay(100);
	}
  char key_confirm;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

 //   ensure=PS_ValidTempleteNum(&ValidN);//读库指纹个数   
   // key_confirm = KEY_SCAN();
    Add_FR();
    printf("1");
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
//录指纹
void Add_FR(void)
{
	u8 i,ensure ,processnum=0;
	u16 ID;
	while(1)
	{
		switch (processnum)
		{
			case 0:
				i++;
			//	LCD_Fill(0,100,lcddev.width,160,WHITE);
				//Show_Str_Mid(0,100,"请按指纹",16,240);
				ensure=PS_GetImage();
				if(ensure==0x00) 
				{
					//PCF8574_WriteBit(BEEP_IO,0);//打开蜂鸣器
					ensure=PS_GenChar(CharBuffer1);//生成特征
					//PCF8574_WriteBit(BEEP_IO,1);//关闭蜂鸣器
					if(ensure==0x00)
					{
					//	LCD_Fill(0,120,lcddev.width,160,WHITE);
						//Show_Str_Mid(0,120,"指纹正常",16,240);
						i=0;
						processnum=1;//跳到第二步						
					}else ShowErrMessage(ensure);				
				}else ShowErrMessage(ensure);						
				break;
			
			case 1:
				i++;
	//			LCD_Fill(0,100,lcddev.width,160,WHITE);
				//Show_Str_Mid(0,100,"请按再按一次指纹",16,240);
				ensure=PS_GetImage();
				if(ensure==0x00) 
				{
				//	PCF8574_WriteBit(BEEP_IO,0);//打开蜂鸣器
					ensure=PS_GenChar(CharBuffer2);//生成特征
			//		PCF8574_WriteBit(BEEP_IO,1);//关闭蜂鸣器
					if(ensure==0x00)
					{
						//LCD_Fill(0,120,lcddev.width,160,WHITE);
						//Show_Str_Mid(0,120,"指纹正常",16,240);
						i=0;
						processnum=2;//跳到第三步
					}else ShowErrMessage(ensure);	
				}else ShowErrMessage(ensure);		
				break;

			case 2:
			//	LCD_Fill(0,100,lcddev.width,160,WHITE);
				//Show_Str_Mid(0,100,"对比两次指纹",16,240);
				ensure=PS_Match();
				if(ensure==0x00) 
				{
			//		LCD_Fill(0,120,lcddev.width,160,WHITE);
					//Show_Str_Mid(0,120,"对比成功,两次指纹一样",16,240);
					processnum=3;//跳到第四步
				}
				else 
				{
			//		LCD_Fill(0,100,lcddev.width,160,WHITE);
					//Show_Str_Mid(0,100,"对比失败，请重新录入指纹",16,240);
					ShowErrMessage(ensure);
					i=0;
					processnum=0;//跳回第一步		
				}
				HAL_Delay(1200);
				break;

			case 3:
		//		LCD_Fill(0,100,lcddev.width,160,WHITE);
				//Show_Str_Mid(0,100,"生成指纹模板",16,240);
				ensure=PS_RegModel();
				if(ensure==0x00) 
				{
		//			LCD_Fill(0,120,lcddev.width,160,WHITE);
					//Show_Str_Mid(0,120,"生成指纹模板成功",16,240);
					processnum=4;//跳到第五步
				}else {processnum=0;ShowErrMessage(ensure);}
				HAL_Delay(1200);
				break;
				
			case 4:	
			//	LCD_Fill(0,100,lcddev.width,160,WHITE);
				//Show_Str_Mid(0,100,"请输入储存ID,按Enter保存",16,240);
				//Show_Str_Mid(0,120,"0=< ID <=299",16,240);
				do
					//ID=GET_NUM();
                                        ID = 10;
				while(!(ID<AS608Para.PS_max));//输入ID必须小于指纹容量的最大值
				ensure=PS_StoreChar(CharBuffer2,ID);//储存模板
				if(ensure==0x00) 
				{			
				//	LCD_Fill(0,100,lcddev.width,160,WHITE);					
					//Show_Str_Mid(0,120,"录入指纹成功",16,240);
					PS_ValidTempleteNum(&ValidN);//读库指纹个数
				//	LCD_ShowNum(56,80,AS608Para.PS_max-ValidN,3,16);
					HAL_Delay(1500);
				//	LCD_Fill(0,100,240,160,WHITE);
					return ;
				}else {processnum=0;ShowErrMessage(ensure);}					
				break;				
		}
		HAL_Delay(400);
		if(i==5)//超过5次没有按手指则退出
		{
		//	LCD_Fill(0,100,lcddev.width,160,WHITE);
			break;	
		}				
	}
}

////刷指纹
//void press_FR(void)
//{
//	SearchResult seach;
//	u8 ensure;
//	char *str;
//	ensure=PS_GetImage();
//	if(ensure==0x00)//获取图像成功 
//	{	
//		PCF8574_WriteBit(BEEP_IO,0);//打开蜂鸣器	
//		ensure=PS_GenChar(CharBuffer1);
//		if(ensure==0x00) //生成特征成功
//		{		
//			PCF8574_WriteBit(BEEP_IO,1);//关闭蜂鸣器	
//			ensure=PS_HighSpeedSearch(CharBuffer1,0,AS608Para.PS_max,&seach);
//			if(ensure==0x00)//搜索成功
//			{				
//				LCD_Fill(0,100,lcddev.width,160,WHITE);
//				//Show_Str_Mid(0,100,"刷指纹成功",16,240);				
//				str=mymalloc(SRAMIN,50);
//				sprintf(str,"确有此人,ID:%d  匹配得分:%d",seach.pageID,seach.mathscore);
//				//Show_Str_Mid(0,140,(u8*)str,16,240);
//				myfree(SRAMIN,str);
//			}
//			else 
//				ShowErrMessage(ensure);					
//	  }
//		else
//			ShowErrMessage(ensure);
//	 PCF8574_WriteBit(BEEP_IO,1);//关闭蜂鸣器
//	 HAL_Delay(600);
//	 LCD_Fill(0,100,lcddev.width,160,WHITE);
//	}
//		
//}
//
////删除指纹
//void Del_FR(void)
//{
//	u8  ensure;
//	u16 num;
//	LCD_Fill(0,100,lcddev.width,160,WHITE);
//	//Show_Str_Mid(0,100,"删除指纹",16,240);
//	//Show_Str_Mid(0,120,"请输入指纹ID按Enter发送",16,240);
//	//Show_Str_Mid(0,140,"0=< ID <=299",16,240);
//	HAL_Delay(50);
//	AS608_load_keyboard(0,170,(u8**)kbd_delFR);
//	num=GET_NUM();//获取返回的数值
//	if(num==0xFFFF)
//		goto MENU ; //返回主页面
//	else if(num==0xFF00)
//		ensure=PS_Empty();//清空指纹库
//	else 
//		ensure=PS_DeletChar(num,1);//删除单个指纹
//	if(ensure==0)
//	{
//		LCD_Fill(0,120,lcddev.width,160,WHITE);
//		//Show_Str_Mid(0,140,"删除指纹成功",16,240);		
//	}
//  else
//		ShowErrMessage(ensure);	
//	HAL_Delay(1200);
//	PS_ValidTempleteNum(&ValidN);//读库指纹个数
//	LCD_ShowNum(56,80,AS608Para.PS_max-ValidN,3,16);
//MENU:	
//	LCD_Fill(0,100,lcddev.width,160,WHITE);
//	HAL_Delay(50);
//	AS608_load_keyboard(0,170,(u8**)kbd_menu);
//}

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
