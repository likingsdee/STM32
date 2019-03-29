
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
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#define CharBuffer1 0x01
#define CharBuffer2 0x02
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
typedef struct
{
	uint16_t PS_max;//指纹最大容量
	uint8_t  PS_level;//安全等级
	uint32_t PS_addr;
	uint8_t  PS_size;//通讯数据包大小
	uint8_t  PS_N;//波特率基数N
}SysPara;
  uint8_t data[100];
  uint8_t check[]={0XEF,0X01,0XFF,0XFF,0XFF,0XFF,0X01,0X00,0X00};
  uint8_t PS_GetImage_buf[]={0XEF,0X01,0XFF,0XFF,0XFF,0XFF,0X01,0X00,0X03,0X01,0X00,0X05};
  uint8_t PS_GenChar_buf[]={0XEF,0X01,0XFF,0XFF,0XFF,0XFF,0X01,0X00,0X04,0X02,0X01,0X00,0X08};
  uint8_t PS_Match_buf[] = {0XEF,0X01,0XFF,0XFF,0XFF,0XFF,0X01,0X00,0X03,0X03,0X00,0X07};
  uint8_t PS_RegModel_buf[] ={0XEF,0X01,0XFF,0XFF,0XFF,0XFF,0X01,0X00,0X03,0X05,0X00,0X09};
  uint8_t PS_StoreChar_buf[]={0XEF,0X01,0XFF,0XFF,0XFF,0XFF,0X01,0X00,0X06,0X06,0X01,0X00,0X00,0X00,0X0E};
  uint8_t PS_HighSpeedSearch_buf[]={0XEF,0X01,0XFF,0XFF,0XFF,0XFF,0X01,0X00,0X08,0X1B,0X01,0X00,0X00,0X01,0X00,0X26};
  uint8_t PS_ReadSysPara_buf[]={0XEF,0X01,0XFF,0XFF,0XFF,0XFF,0X01,0X00,0X03,0X0F,0X00,0X13};
  uint8_t PS_ValidTempleteNum_buf[]={0XEF,0X01,0XFF,0XFF,0XFF,0XFF,0X01,0X00,0X03,0X1D,0X00,0X21};
  void Add_FR(void);
  void press_FR(void);
  uint8_t PS_ReadSysPara(SysPara *p);
  uint8_t PS_ValidTempleteNum(uint16_t *ValidN);
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
  /* USER CODE BEGIN 2 */
  
  HAL_UART_Receive_DMA(&huart3, data, RECEIVELEN);//开启串口3DMA接收
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE); 
  while(UART3_TYPE.receive_flag == 0)
  {
    HAL_UART_Transmit(&huart3,check,9,0xffff);
    HAL_Delay(5);
  }
  UART3_TYPE.receive_flag=  0; 
  SysPara p;
  uint16_t num;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
   PS_ValidTempleteNum(&num);
   //PS_ReadSysPara(&p);
// while(UART3_TYPE.usartDMA_rxBuf[9] != 0X00 &&UART3_TYPE.usartDMA_rxBuf[11] != 0X0A)
// {
//   HAL_UART_Transmit(&huart3,PS_GetImage_buf,12,0xffff);
//   HAL_Delay(5);
// }
 // HAL_UART_Transmit(&huart3,PS_GenChar_1,13,0xffff);
 // if(UART3_TYPE.usartDMA_rxBuf[9] == 0X00 &&UART3_TYPE.usartDMA_rxBuf[11] == 0X0A){}
 // Add_FR();
  //  press_FR();
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
        if(huart->Instance == USART3)
	{
            UART3_TYPE.rx_len =  RECEIVELEN - temp; 
            UART3_TYPE.receive_flag=  1; 
            HAL_UART_Receive_DMA(&huart3,UART3_TYPE.usartDMA_rxBuf,RECEIVELEN);
	}

    }  
}
//显示确认码错误信息
void ShowErrMessage(uint8_t ensure)
{
	//LCD_Fill(0,120,lcddev.width,160,WHITE);
	//Show_Str_Mid(0,120,(uint8_t*)EnsureMessage(ensure),16,240);
}
//录入图像 PS_GetImage
//功能:探测手指，探测到后录入指纹图像存于ImageBuffer。 
//模块返回确认字
uint8_t PS_GetImage(void)
{
  uint8_t  ensure;
  HAL_UART_Transmit(&huart3,PS_GetImage_buf,12,0xffff);
 // HAL_Delay(1);
//	if(UART3_TYPE.receive_flag ==  1 )
//        {
//		ensure=UART3_TYPE.usartDMA_rxBuf[9];
//                UART3_TYPE.receive_flag=  0; 
//        }
//	else
//		ensure=0xff;
       while(UART3_TYPE.receive_flag !=  1 );
       // {
		ensure=UART3_TYPE.usartDMA_rxBuf[9];
                UART3_TYPE.receive_flag=  0; 
        //}
	//else
	//	ensure=0xff;
	return ensure;
}
//生成特征 PS_GenChar
//功能:将ImageBuffer中的原始图像生成指纹特征文件存于CharBuffer1或CharBuffer2			 
//参数:BufferID --> charBuffer1:0x01	charBuffer1:0x02												
//模块返回确认字
uint8_t PS_GenChar(uint8_t BufferID)
{
        uint8_t  ensure;
        if(BufferID == 0X01)
        {
          PS_GenChar_buf[10] = 0X01;
          PS_GenChar_buf[12]= 0X08;
          HAL_UART_Transmit(&huart3,PS_GenChar_buf,13,0xffff);
        //    HAL_Delay(200);
        }
        else {
          PS_GenChar_buf[10] = 0X02;
          PS_GenChar_buf[12]= 0X09;
          HAL_UART_Transmit(&huart3,PS_GenChar_buf,13,0xffff);
       //     HAL_Delay(200);
        }
//	if(UART3_TYPE.receive_flag ==  1 )
//        {
//		ensure=UART3_TYPE.usartDMA_rxBuf[9];
//                UART3_TYPE.receive_flag=  0;
//        }
//	else
//		ensure=0xff;
         while(UART3_TYPE.receive_flag !=  1 );
       		ensure=UART3_TYPE.usartDMA_rxBuf[9];
               UART3_TYPE.receive_flag=  0;
	return ensure;
}
//精确比对两枚指纹特征 PS_Match
//功能:精确比对CharBuffer1 与CharBuffer2 中的特征文件 
//模块返回确认字
uint8_t PS_Match(void)
{
        uint8_t  ensure;
        HAL_UART_Transmit(&huart3,PS_Match_buf,12,0xffff);
          HAL_Delay(200);
	if(UART3_TYPE.receive_flag ==  1)
        {
		ensure=UART3_TYPE.usartDMA_rxBuf[9];
                UART3_TYPE.receive_flag=  0;
        }
	else
		ensure=0xff;
	return ensure;
}
//合并特征（生成模板）PS_RegModel
//功能:将CharBuffer1与CharBuffer2中的特征文件合并生成 模板,结果存于CharBuffer1与CharBuffer2	
//说明:  模块返回确认字
uint8_t PS_RegModel(void)
{
        uint8_t  ensure;
	HAL_UART_Transmit(&huart3,PS_RegModel_buf,12,0xffff);
        HAL_Delay(200);
	if(UART3_TYPE.receive_flag ==  1)
        {
		ensure=UART3_TYPE.usartDMA_rxBuf[9];
                UART3_TYPE.receive_flag=  0;
        }
	else
		ensure=0xff;
	return ensure;		
}
//储存模板 PS_StoreChar
//功能:将 CharBuffer1 或 CharBuffer2 中的模板文件存到 PageID 号flash数据库位置。			
//参数:  BufferID @ref charBuffer1:0x01	charBuffer1:0x02
//       PageID（指纹库位置号）
//说明:  模块返回确认字
//uint8_t PS_StoreChar(uint8_t BufferID,uint16_t PageID)
uint8_t PS_StoreChar(void)
{
        uint8_t  ensure;
	HAL_UART_Transmit(&huart3,PS_StoreChar_buf,15,0xffff);
        HAL_Delay(200);
	if(UART3_TYPE.receive_flag ==  1)
        {
		ensure=UART3_TYPE.usartDMA_rxBuf[9];
                UART3_TYPE.receive_flag=  0;
        }
	else
		ensure=0xff;
	return ensure;	
}
//读系统基本参数 PS_ReadSysPara
//功能:  读取模块的基本参数（波特率，包大小等)
//参数:  无
//说明:  模块返回确认字 + 基本参数（16bytes）
uint8_t PS_ReadSysPara(SysPara *p)
{
         uint8_t  ensure;
        HAL_UART_Transmit(&huart3,PS_ReadSysPara_buf,12,0xffff);
        HAL_Delay(200);
	if(UART3_TYPE.receive_flag ==  1)
	{
		ensure = UART3_TYPE.usartDMA_rxBuf[9];
		p->PS_max = (UART3_TYPE.usartDMA_rxBuf[14]<<8)+UART3_TYPE.usartDMA_rxBuf[15];
		p->PS_level = data[17];
		p->PS_addr=(UART3_TYPE.usartDMA_rxBuf[18]<<24)+(UART3_TYPE.usartDMA_rxBuf[19]<<16)+(UART3_TYPE.usartDMA_rxBuf[20]<<8)+UART3_TYPE.usartDMA_rxBuf[21];
		p->PS_size = UART3_TYPE.usartDMA_rxBuf[23];
		p->PS_N = UART3_TYPE.usartDMA_rxBuf[25];
                UART3_TYPE.receive_flag=  0;
	}		
	else
		ensure=0xff;
	if(ensure==0x00)
	{
		printf("\r\n模块最大指纹容量=%d",p->PS_max);
		printf("\r\n对比等级=%d",p->PS_level);
		printf("\r\n地址=%x",p->PS_addr);
		printf("\r\n波特率=%d",p->PS_N*9600);
	}
//	else 
	//		printf("\r\n%s",EnsureMessage(ensure));
	return ensure;
}
//高速搜索PS_HighSpeedSearch
//功能：以 CharBuffer1或CharBuffer2中的特征文件高速搜索整个或部分指纹库。
//		  若搜索到，则返回页码,该指令对于的确存在于指纹库中 ，且登录时质量
//		  很好的指纹，会很快给出搜索结果。
//参数:  BufferID， StartPage(起始页)，PageNum（页数）
//说明:  模块返回确认字+页码（相配指纹模板）
//uint8_t PS_HighSpeedSearch(uint8_t BufferID,u16 StartPage,u16 PageNum,SearchResult *p)
uint8_t PS_HighSpeedSearch(void)
{
  uint8_t  ensure;
	HAL_UART_Transmit(&huart3,PS_HighSpeedSearch_buf,16,0xffff);
        HAL_Delay(500);
	if(UART3_TYPE.receive_flag ==  1)
        {
		ensure=UART3_TYPE.usartDMA_rxBuf[9];
                UART3_TYPE.receive_flag=  0;
        }
	else
		ensure=0xff;
	return ensure;
}
//读有效模板个数 PS_ValidTempleteNum
//功能：读有效模板个数
//参数: 无
//说明: 模块返回确认字+有效模板个数ValidN
uint8_t PS_ValidTempleteNum(uint16_t *ValidN)
{
        uint8_t  ensure;
        HAL_UART_Transmit(&huart3,PS_ValidTempleteNum_buf,12,0xffff);
        HAL_Delay(200);
	if(UART3_TYPE.receive_flag ==  1)
	{
		ensure=UART3_TYPE.usartDMA_rxBuf[9];
		*ValidN = (UART3_TYPE.usartDMA_rxBuf[10]) +UART3_TYPE.usartDMA_rxBuf[11];
	}		
	else
		ensure=0xff;
	
	if(ensure==0x00)
	{
		printf("\r\n有效指纹个数=%d",(UART3_TYPE.usartDMA_rxBuf[10])+UART3_TYPE.usartDMA_rxBuf[11]);
	}
//	else
//		printf("\r\n%s",EnsureMessage(ensure));
	return ensure;
}
//刷指纹
void press_FR(void)
{
	//SearchResult seach;
	uint8_t ensure;
	//char *str;
	ensure=PS_GetImage();
	if(ensure==0x00)//获取图像成功 
	{	
		ensure=PS_GenChar(0X01);
		if(ensure==0x00) //生成特征成功
		{		

			ensure=PS_HighSpeedSearch();
			if(ensure==0x00)//搜索成功
			{				
			
                           printf("1");
			}
			else 
				ShowErrMessage(ensure);					
	  }
		else
			ShowErrMessage(ensure);
	}
		
}
void Add_FR(void)
{
	uint8_t i,ensure ,processnum=0;
	//u16 ID;
	while(1)
	{
		switch (processnum)
		{
			case 0:
				i++;
				ensure=PS_GetImage();
				if(ensure==0x00) 
				{
					ensure=PS_GenChar(0X01);//生成特征
					if(ensure==0x00)
					{
						i=0;
						processnum=1;//跳到第二步						
					}else ShowErrMessage(ensure);				
				}else ShowErrMessage(ensure);						
				break;
			
			case 1:
				i++;
				ensure=PS_GetImage();
				if(ensure==0x00) 
				{
					ensure=PS_GenChar(0X02);//生成特征
					if(ensure==0x00)
					{

						i=0;
						processnum=2;//跳到第三步
					}else ShowErrMessage(ensure);	
				}else ShowErrMessage(ensure);		
				break;

			case 2:
          //                PS_RegModel();
          //                PS_StoreChar();
				ensure=PS_Match();
				if(ensure==0x00) 
				{
					processnum=3;//跳到第四步
				}
				else 
				{
					ShowErrMessage(ensure);
					i=0;
					processnum=0;//跳回第一步		
				}
				HAL_Delay(1200);
				break;
                }
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
