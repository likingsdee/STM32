/************************************************************************************
							本例程提供自以下店铺：
								Ilovemcu.taobao.com
								epic-mcu.taobao.com
							实验相关外围扩展模块均来自以上店铺
							作者：神秘藏宝室							
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
//    /*********************液晶使用的I/O口初始化**************************/ 
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

    CLR_RW();			//读写位直接低电平，只写不读

    /*********************液晶初始化**************************/        
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
   
    
    LCD_write_cmd(0x20);          // 4bit显示模式,2行,5x7字体
    delay (20000);  
  //  LCD_write_cmd(0x08);         // 显示关闭 
    //delay (20000); 
    LCD_write_cmd(0x01);         // 显示清屏 
    delay (20000); 
    LCD_write_cmd(0x06);         // 显示光标移动设置 
    delay (20000);
    LCD_write_cmd(0x0C);         //显示开,光标关，
    //LCD_write_cmd(0x0F);         // 显示开，光标开，光标闪烁
    delay (20000);
	LCD_write_cmd(0x01);         //清屏
}
/*--------------------------------------------------
函数说明：写命令到液晶


---------------------------------------------------*/
void LCD_write_cmd(unsigned char cmd)
{
    
    CLR_RS();
    LCD_Write_half_byte(cmd >> 4);
    LCD_Write_half_byte(cmd);
    delay (10000);
}
/*--------------------------------------------------
函数说明：写数据到液晶


---------------------------------------------------*/
void LCD_write_data(unsigned char w_data)
{
    SET_RS();
    LCD_Write_half_byte(w_data >> 4);
    LCD_Write_half_byte(w_data);
    delay (10000);
}
/*--------------------------------------------------
函数说明：写4bit到液晶
--------------------------------------------------*/
void LCD_Write_half_byte(unsigned char half_byte)
{  
//    u16 temp_io = 0x0000;
//    temp_io = GPIO_ReadOutputData(GPIOE);   //读端口E输出口的数据
//    temp_io &= 0xfff0;                      //屏蔽低四位
//    temp_io |= (u16)(half_byte&0x0f);       //得到新数据
//    GPIO_Write(GPIOE,temp_io);              //写入新数据
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
LCD_set_xy        : 设置LCD显示的起始位置
输入参数：x、y    : 显示字符串的位置，X:1-16，Y:1-2                
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
LCD_write_string  : 英文字符串显示函数
输入参数：*s      ：英文字符串指针；
          X、Y    : 显示字符串的位置                
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
                   LCD_write_byte(1,dirction);      //字符移动方向                                    
         Delay_nms(time);                //控制移动时间
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
        1.总共分为上下两个屏幕,其中第0->31行和第32->63行的行地址相同

        (上 0->31)第一个屏幕的首地址(0x80,0x80) -> ... -> (0x80,0x87)
                                   ....
                                  (0xa0,0x80) -> ... -> (0xa0,0x87)
        (下 32->63)第二个屏幕的首地址(0x80,0x88) -> ... -> (0x80,0x90)
                                    ...
                                 (0xa0,0x88) -> ... -> (0xa0,0x90)

        2.每个地址就是控制 一行16个小点,发送两次数据分别控制前8点和后8个点,
            例写入的01010101 01010101来控制屏幕的点 , 1显示 0不显示

        3.先写入行地址,后写入列地址,连续写列地址会自动增加
        */

        //进行清屏操作 LCD12864_Clear_Screen(0xFF)全亮
            //LCD12864_Clear_Screen(0xFF)全灭
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
//显示非中文字符 和 中文字符都差不多,因为自带汉字库,只是注意写汉字的时候要连续写!
// 注意这里的坐标变了,一定要注意!!!
    /*  
    实际坐标:
        0x80 -> 0x87
        0x90 -> 0x97
        0x88 -> 0x8f
        0x98 -> 0x9f

        //这个坐标我们自己规定的,再换算成上面的实际坐标就可以了
        指出坐标(1,1) -> (1,8)
                (4,1) -> (4,8)
        再指出需要显示的字符
        */
        unsigned char addresses[] = {0x80,0x90,0x88,0x98};
        void LCD12864_Display_Char(unsigned char x,unsigned char y,unsigned char dat){


                LCD_write_cmd(0x30);
                LCD_write_cmd(0x06);

                //写入地址
                LCD_write_cmd(addresses[x-1]+(y-1));

                //写入数据
                LCD_write_data(dat);

        }

        //显示汉字
        void LCD12864_Display_Chars(unsigned char x,unsigned char y,unsigned char *dat){


                LCD_write_cmd(0x30);
                LCD_write_cmd(0x06);

                //写入地址
                LCD_write_cmd(addresses[x-1]+(y-1));

                //写入数据
                while(*dat != '\0'){
                    LCD_write_data(*dat);
                    dat++;
                }
        }

