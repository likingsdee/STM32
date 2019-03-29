/************************************************************************************
							�������ṩ�����µ��̣�
								Ilovemcu.taobao.com
								epic-mcu.taobao.com
							ʵ�������Χ��չģ����������ϵ���
							���ߣ����زر���							
*************************************************************************************/
#include "LCD1602.h"
#include "stm32f1xx_hal.h"

//GPIO_InitTypeDef GPIO_InitStructure;
/* If processor works on high frequency delay has to be increased, it can be 
   increased by factor 2^N by this constant                                   */
#define DELAY_2N     0

//==================================================
void LCD_init(void)
{
//    /*********************Һ��ʹ�õ�I/O�ڳ�ʼ��**************************/ 
//	GPIO_InitTypeDef GPIO_InitStructure;
//	
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//  
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);
//
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    GPIO_Init(GPIOB, &GPIO_InitStructure);

    CLR_RW();			//��дλֱ�ӵ͵�ƽ��ֻд����

    /*********************Һ����ʼ��**************************/        
    delay (15000);
	
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_SET);
    HAL_Delay(20);
//   
//    CLR_RS();
//    LCD_Write_half_byte(0x3);                 
//    delay (15000);
//    LCD_Write_half_byte(0x3);
//    delay (15000);
//    LCD_Write_half_byte(0x3);
//    LCD_Write_half_byte(0x2);
   
    
    LCD_write_cmd(0x20);          // 4bit��ʾģʽ,2��,5x7����
    delay (20000);  
  //  LCD_write_cmd(0x08);         // ��ʾ�ر� 
    //delay (20000); 
    LCD_write_cmd(0x01);         // ��ʾ���� 
    delay (20000); 
    LCD_write_cmd(0x06);         // ��ʾ����ƶ����� 
    delay (20000);
    LCD_write_cmd(0x0C);         //��ʾ��,���أ�
    //LCD_write_cmd(0x0F);         // ��ʾ������꿪�������˸
    delay (20000);
	LCD_write_cmd(0x01);         //����
}
/*--------------------------------------------------
����˵����д���Һ��


---------------------------------------------------*/
void LCD_write_cmd(unsigned char cmd)
{
    
    CLR_RS();
    LCD_Write_half_byte(cmd >> 4);
    LCD_Write_half_byte(cmd);
    delay (10000);
}
/*--------------------------------------------------
����˵����д���ݵ�Һ��


---------------------------------------------------*/
void LCD_write_data(unsigned char w_data)
{
    SET_RS();
    LCD_Write_half_byte(w_data >> 4);
    LCD_Write_half_byte(w_data);
    delay (10000);
}
/*--------------------------------------------------
����˵����д4bit��Һ��
--------------------------------------------------*/
void LCD_Write_half_byte(unsigned char half_byte)
{  
//    u16 temp_io = 0x0000;
//    temp_io = GPIO_ReadOutputData(GPIOE);   //���˿�E����ڵ�����
//    temp_io &= 0xfff0;                      //���ε���λ
//    temp_io |= (u16)(half_byte&0x0f);       //�õ�������
//    GPIO_Write(GPIOE,temp_io);              //д��������
     SET_EN();
    delay(2000);
	if (half_byte&0x01)
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3,GPIO_PIN_SET);
	else
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3,GPIO_PIN_RESET);

	if (half_byte&0x02)
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_SET);
	else
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_RESET);

	if (half_byte&0x04)
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_SET);
	else
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_RESET);

	if (half_byte&0x08)
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,GPIO_PIN_SET);
	else
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,GPIO_PIN_RESET);

 
    CLR_EN(); 
    delay(2000);
}

/*----------------------------------------------------
LCD_set_xy        : ����LCD��ʾ����ʼλ��
���������x��y    : ��ʾ�ַ�����λ�ã�X:1-16��Y:1-2                
-----------------------------------------------------*/
void LCD_set_xy( unsigned char x, unsigned char y )
{
    unsigned char address;
    if (y==1) 
    {
        address=0x80-1+x;
    }
    else 
    {
        address=0xc0-1+x;
    }
    LCD_write_cmd(address);
}
/*---------------------------------------------------
LCD_write_string  : Ӣ���ַ�����ʾ����
���������*s      ��Ӣ���ַ���ָ�룻
          X��Y    : ��ʾ�ַ�����λ��                
---------------------------------------------------*/
void LCD_write_string(unsigned char X,unsigned char Y,unsigned char *s)
{
    LCD_set_xy( X, Y );   
    while (*s) 
    {
        LCD_write_data(*s);
        s++;
    }
}
//=======================================================
/*
void Move(unsigned int step,unsigned int dirction,unsigned int time)
{
          unsigned int i;
         for(i=0;i<step-1;i++)
         {
                   LCD_write_byte(1,dirction);      //�ַ��ƶ�����                                    
         Delay_nms(time);                //�����ƶ�ʱ��
         }
}
*/
//=========================================================
/*
void Flash_lcd(unsigned int delay_t, unsigned int times)
{
           unsigned int j;
         for(j=0;j<times;j++)
         {
                  LCD_write_byte(1,0x08);
                Delay_nms(delay_t);
                LCD_write_byte(1,0x0c);
                Delay_nms(delay_t);
         }
}
*/
//========================================================
void delay(volatile unsigned long cnt)
{
  cnt <<= DELAY_2N;

  while (cnt--);
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

            LCD_write_cmd(0x34);
            LCD_write_cmd(0x36);


            for(i=0;i<64;i++){
                    if(i<32){
                    LCD_write_cmd(0x80+i);
                    LCD_write_cmd(0x80);
                    }else {
                    LCD_write_cmd(0x80+(i-32));
                    LCD_write_cmd(0x88);
                    }

                    for(j=0;j<16;j++)
                        LCD_write_data(value);
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


                LCD_write_cmd(0x30);
                LCD_write_cmd(0x06);

                //д���ַ
                LCD_write_cmd(addresses[x-1]+(y-1));

                //д������
                LCD_write_data(dat);

        }

        //��ʾ����
        void LCD12864_Display_Chars(unsigned char x,unsigned char y,unsigned char *dat){


                LCD_write_cmd(0x30);
                LCD_write_cmd(0x06);

                //д���ַ
                LCD_write_cmd(addresses[x-1]+(y-1));

                //д������
                while(*dat != '\0'){
                    LCD_write_data(*dat);
                    dat++;
                }
        }

