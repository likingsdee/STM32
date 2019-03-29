
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
#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t
#include "w5500.h"
#include "tcp_demo.h"
#include "w5500_conf.h"
#include "socket.h"
#include "utility.h"
#include "dhcp.h"

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
void HM_TRP_Init(void);
void HM_TRP_uart_Init(void);
void HM_TRP_send_Init(void);
void HM_TRP_rank_Init(void);
uint8_t   Com1_Rxbuff[1024];  //���ڽ�������
uint16_t  Com1_Counter=0;     //���ռ���
uint16_t  Com1_TxCounter=0;   //���ͼ���
uint8_t   Com2_Rxbuff[1024];  //���ڽ�������
uint16_t  Com2_Counter=0;     //���ռ���
uint16_t  Com2_TxCounter=0;   //���ͼ���
uint8_t   Com3_Rxbuff[1024];  //���ڽ�������
uint16_t  Com3_Counter=0;     //���ռ���
uint16_t  Com3_TxCounter=0;   //���ͼ���
uint8_t   Com4_Rxbuff[1024];  //���ڽ�������
uint16_t  Com4_Counter=0;     //���ռ���
uint16_t  Com4_TxCounter=0;   //���ͼ���
uint8_t aRxStartMessages_1[1];
uint8_t aRxStartMessages_2[1];
uint8_t aRxStartMessages_3[1];
uint8_t aRxStartMessages_4[1];
uint8_t config[]={0xAA,0xFA,0x87};
uint8_t set_uart_speed[]={0xAA,0xFA,0x1E,0x00,0x01,0xC2,0x00};//���õ�̨���ڲ�����Ϊ38400
uint8_t set_send_speed[]={0xAA,0xFA,0xC3,0x00,0x00,0x25,0x80};//�������ߴ�������Ϊ38400
uint8_t set_rank_915[]={0xAA,0xFA,0xD2,0x36,0x89,0xCA,0xC0};//���ù���Ƶ��Ϊ915000000HZ
uint8_t set_rank_434[]={0xAA,0xFA,0xD2,0x19,0xDE,0x50,0x80};//���ù���Ƶ��Ϊ434000000HZ
uint8_t read_config[]={0xAA,0xFA,0xE1};
uint8_t reset[]={0xAA,0xFA,0xF0};
/*
int fputc(int ch, FILE *f)
{      
	//HAL_UART_Transmit(&huart5, (uint8_t *)&ch, 1, 0xffff); 
    while((USART1->SR&0X40)==0); 
        USART1->DR = (uint8_t) ch; 
	return ch;
}
*/
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
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
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  
//  int i=0;
       printf(" W5500����ͨ�Ų��� Demo V1.0 \r\n");		
        reset_w5500();                     /* W5500Ӳ����λ */
	set_w5500_mac();										/*����MAC��ַ*/
	set_w5500_ip();											/*����IP��ַ*/
	
	socket_buf_init(txsize, rxsize);		/*��ʼ��8��Socket�ķ��ͽ��ջ����С*/

        printf(" ������ΪTCP������,��W5500��Ϊ TCP�ͻ���ȥ���� \n");
	printf(" ������IP:%d.%d.%d.%d\n",remote_ip[0],remote_ip[1],remote_ip[2],remote_ip[3]);
	printf(" �����˿�:%d \n",remote_port);
	printf(" ���ӳɹ��󣬷������������ݸ�W5500��W5500�����ض�Ӧ���� \n");
	printf(" Ӧ�ó���ִ���С���\n");										/*����IP��ַ*/
	
	socket_buf_init(txsize, rxsize);		/*��ʼ��8��Socket�ķ��ͽ��ջ����С*/
        HAL_UART_Receive_IT(&huart1 ,(uint8_t*)aRxStartMessages_1,1);  //ʹ��UART����
        HAL_UART_Receive_IT(&huart2 ,(uint8_t*)aRxStartMessages_1,1);
        HAL_UART_Receive_IT(&huart3 ,(uint8_t*)aRxStartMessages_1,1);
        HAL_UART_Receive_IT(&huart4 ,(uint8_t*)aRxStartMessages_1,1);
        HM_TRP_Init();//��̨ģ���ʼ��
        //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); //��̨2��������ģʽ
       // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); //��̨1��������ģʽ
        int i=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      do_udp();
    //do_tcp_client();
   //  HAL_UART_Transmit_IT(&huart2 ,read_config,sizeof(read_config));
    // HAL_UART_Transmit_IT(&huart2 ,reset,sizeof(reset));
    //HAL_UART_Transmit_IT(&huart3 ,reset,sizeof(reset));
//       HAL_Delay(2000);
//    if(Com4_Counter>0)
//    {
//      HAL_UART_Transmit_IT(&huart3 ,Com4_Rxbuff,Com4_Counter);
//      memset(Com4_Rxbuff,0,sizeof(Com4_Rxbuff));
//      Com4_Counter = 0; 
//    }
//
//     if(Com2_Counter>0)
//     {
//
//       printf("%s",Com2_Rxbuff);
//       memset(Com2_Rxbuff,0,sizeof(Com2_Rxbuff));
//      printf("\r\n");
//      Com2_Counter=0;
//     }
    /*
    HAL_Delay(500);
   HAL_UART_Transmit_IT(&huart3 ,read_config,sizeof(read_config)); 
  
    if(Com3_Counter>0)
     {
      for(i=0;i<Com3_Counter;i++)
       printf("%02X ",Com3_Rxbuff[i]);
       memset(Com3_Rxbuff,0,sizeof(Com3_Rxbuff));
      printf("\r\n");
      Com3_Counter=0;
     }
 */
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);
}

/* USER CODE BEGIN 4 */
/*******************************************************
function:  HAL_UART_RxCpltCallback UART���ջص�����
input parameter:			 ���ں�
output parameter:			 ��
*******************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   UNUSED(huart);
   
	if(huart == (&huart1))
	{
	  HAL_UART_Receive_IT(&huart1 ,(uint8_t*)aRxStartMessages_1,1);
          Com1_Rxbuff[Com1_Counter++] = aRxStartMessages_1[0];
    if(Com1_Counter==0xffff)
	  {
		 Com1_Counter = 0;
	  }
	}
       if(huart == (&huart2))
	{
	  HAL_UART_Receive_IT(&huart2 ,(uint8_t*)aRxStartMessages_2,1);
          Com2_Rxbuff[Com2_Counter++] = aRxStartMessages_2[0];
    if(Com2_Counter==0xffff)
	  {
		 Com2_Counter = 0;
	  }
	} 
        if(huart == (&huart3))
	{
	  HAL_UART_Receive_IT(&huart3 ,(uint8_t*)aRxStartMessages_3,1);
          Com3_Rxbuff[Com3_Counter++] = aRxStartMessages_3[0];
    if(Com3_Counter==0xffff)
	  {
		 Com3_Counter = 0;
	  }
	} 
         if(huart == (&huart4))
	{
	  HAL_UART_Receive_IT(&huart4 ,(uint8_t*)aRxStartMessages_4,1);
          Com4_Rxbuff[Com4_Counter++] = aRxStartMessages_4[0];
    if(Com4_Counter==0xffff)
	  {
		 Com4_Counter = 0;
	  }
	} 

}

//HM-TRP��̨ģ��������ú���
void HM_TRP_uart_Init(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);//config�������ͣ���������ģʽ
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_UART_Transmit_IT(&huart2 ,set_uart_speed,sizeof(set_uart_speed));//���õ�̨1��������
    HAL_UART_Transmit_IT(&huart3 ,set_uart_speed,sizeof(set_uart_speed));//���õ�̨2��������
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);//config�������ߣ����빤��ģʽ
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
}

void HM_TRP_send_Init(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);//config�������ͣ���������ģʽ
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_UART_Transmit_IT(&huart2 ,set_send_speed,sizeof(set_send_speed));//���õ�̨1���ߴ�������
    HAL_UART_Transmit_IT(&huart3 ,set_send_speed,sizeof(set_send_speed));//���õ�̨2���ߴ�������
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);//config�������ߣ����빤��ģʽ
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
}
void HM_TRP_rank_Init(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);//config�������ͣ���������ģʽ
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_UART_Transmit_IT(&huart2 ,set_rank_915,sizeof(set_rank_915));//2���ڽ���Ƶ��915
    HAL_UART_Transmit_IT(&huart3 ,set_rank_434,sizeof(set_rank_434));//3���ڷ���Ƶ��Ϊ434
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);//config�������ߣ����빤��ģʽ
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
}
void HM_TRP_Init(void)
{
   HM_TRP_send_Init();
 // HAL_Delay(50);
 // HM_TRP_rank_Init();
  // HM_TRP_uart_Init();
  HAL_Delay(50);
 
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
