
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
#include "gpio.h"

/* USER CODE BEGIN Includes */

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
#define  SCL_SET      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)
#define  SDA_SET      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET)
#define  SCL_RESET    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)
#define  SDA_RESET    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET)
void Delay(uint32_t count)//���ڲ���400KHzIIC�ź�����Ҫ����ʱ
{
 // count=count*5;	
  while (count--);
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Send_Byte(u8 txd)
*��������:	    IIC����һ���ֽ�
*******************************************************************************/		  
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;
    uint8_t t_temp;
   // SDA_OUT(); 	    
    SCL_RESET;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
      t_temp=txd&0x80;
      if((t_temp>>7)==0)
      SDA_RESET;
      else if((t_temp>>7)==1)
      SDA_SET;
        txd<<=1; 	  
			
		Delay(15);   
		SCL_SET;
		Delay(15);
		SCL_RESET;	
		Delay(15);
    }	 
} 	 
   
void LCD_wr(uint8_t lcd_com,uint8_t lcd_data)                                         //д��LCD���ݻ�����
{
        uint8_t lcd_data_msb,lcd_data_lsb;
        uint8_t cmd=0xf8;
        uint8_t data=0xfa;
        lcd_data_msb=0xf0&lcd_data;
       lcd_data_lsb= (lcd_data<< 4) & 0xf0;
        //lcd_data_lsb=(0x0f&lcd_data)<<4;
        if(lcd_com==0)                                        //д����
        {
          //      while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET);        //�ȴ� SPI1 ���ͻ����
          IIC_Send_Byte(cmd);
          //  HAL_SPI_Transmit_DMA(&hspi1,    &cmd,1);                        // SPI1 ��������--����ָ��-- 
        }
        if(lcd_com==1)                                        //д����
        {
           //     while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET);                //�ȴ� SPI1 ���ͻ����
          IIC_Send_Byte(data);
       //     HAL_SPI_Transmit_DMA(&hspi1, &data,1);                    //SPI1 ��������--����ָ��--
        }
       // HAL_Delay(100);
       // while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET);                //�ȴ� SPI1 �����������
        IIC_Send_Byte(lcd_data_msb);
        IIC_Send_Byte(lcd_data_lsb);
      //  HAL_SPI_Transmit_DMA(&hspi1, &lcd_data_msb,1);                   //SPI1 ���͸�4λ����
   //     HAL_Delay(100);
    //    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET);                //Wait for SPI1 Tx buffer empty
     //   HAL_SPI_Transmit_DMA(&hspi1, &lcd_data_lsb,1);            // SPI1 ���͵�4λ����
        HAL_Delay(100);
}    
void initlcd(void)                          //LCD��ʼ��
{
  LCD_wr(0,0x30);                  //30---����ָ��� 
  HAL_Delay (10);
  LCD_wr(0,0x01);                  //��������ַָ��ָ��00H
  HAL_Delay (10);
  LCD_wr(0,0x06);                  //�����ƶ�����
  HAL_Delay (10);
  LCD_wr(0,0x0c);                  //����ʾ�����α�
  HAL_Delay (10);        
}

//========================================================
 /*  
        1.�ܹ���Ϊ����������Ļ,���е�0->31�к͵�32->63�е��е�ַ��ͬ

        (�� 0->31)��һ����Ļ���׵�ַ(0x80,0x80) -> ... -> (0x80,0x87)
                                   ....
                                  (0xa0,0x80) -> ... -> (0xa0,0x87)
        (�� 32->63)�ڶ�����Ļ���׵�ַ(0x80,0x88) -> ... -> (0x80,0x90)
                                    ...
                                 (0xa0,0x88) -> ... -> (0xa0,0x90)

        2.ÿ����ַ���ǿ��� һ��16��С��,�����������ݷֱ����ǰ8��ͺ�8����,
            ��д���01010101 01010101��������Ļ�ĵ� , 1��ʾ 0����ʾ

        3.��д���е�ַ,��д���е�ַ,����д�е�ַ���Զ�����
        */

        //������������ LCD12864_Clear_Screen(0xFF)ȫ��
            //LCD12864_Clear_Screen(0xFF)ȫ��
        void LCD12864_Clear_Screen(unsigned char value)
        {
            unsigned char i,j;

            LCD_wr(0,0x34);
            LCD_wr(0,0x36);


            for(i=0;i<64;i++){
                    if(i<32){
                    LCD_wr(0,0x80+i);
                    LCD_wr(0,0x80);
                    }else {
                    LCD_wr(0,0x80+(i-32));
                    LCD_wr(0,0x88);
                    }

                    for(j=0;j<16;j++)
                        LCD_wr(1,value);
            }
        }
//��ʾ�������ַ� �� �����ַ������,��Ϊ�Դ����ֿ�,ֻ��ע��д���ֵ�ʱ��Ҫ����д!
// ע��������������,һ��Ҫע��!!!
    /*  
    ʵ������:
        0x80 -> 0x87
        0x90 -> 0x97
        0x88 -> 0x8f
        0x98 -> 0x9f

        //������������Լ��涨��,�ٻ���������ʵ������Ϳ�����
        ָ������(1,1) -> (1,8)
                (4,1) -> (4,8)
        ��ָ����Ҫ��ʾ���ַ�
        */
        unsigned char addresses[] = {0x80,0x90,0x88,0x98};
        void LCD12864_Display_Char(unsigned char x,unsigned char y,unsigned char dat){


                LCD_wr(0,0x30);
                LCD_wr(0,0x06);

                //д���ַ
                LCD_wr(0,addresses[x-1]+(y-1));

                //д������
                LCD_wr(1,dat);

        }

        //��ʾ����
        void LCD12864_Display_Chars(unsigned char x,unsigned char y,unsigned char *dat){


                LCD_wr(0,0x30);
                LCD_wr(0,0x06);

                //д���ַ
                LCD_wr(0,addresses[x-1]+(y-1));

                //д������
                while(*dat != '\0'){
                    LCD_wr(1,*dat);
                    dat++;
                }
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
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
initlcd();

unsigned char *datas ="LCDҺ����ʾ";
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    LCD12864_Display_Char(1,2,'A');
        LCD12864_Display_Chars(2,1,datas);
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
