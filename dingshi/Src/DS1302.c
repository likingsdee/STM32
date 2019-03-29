/**
  ******************************************************************************
  * @file    DS1302.c
  * @version 孙文捷毕设 1.0
  * @brief   DS1302驱动
  ******************************************************************************
  */
  
#include "DS1302.h"

/* 宏定义 ------------------------------------------------ */
#define u8 uint8_t
/* 内部函数声明 ------------------------------------------ */
void DS1302_Config_DIO_Input(void);
void DS1302_Config_DIO_Output(void);
  


/**
  **********************************************************
  * 函数名称：DS1302_Config_DIO_Input
  * 功    能：将DIO口配置为输入模式，用于读取数据	
  **********************************************************
  */
void DS1302_Config_DIO_Input(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);  
}

/**
  **********************************************************
  * 函数名称：DS1302_Config_DIO_Output
  * 功    能：将DIO口配置为输出模式，用于写入数据	
  **********************************************************
  */
void DS1302_Config_DIO_Output(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  **********************************************************
  * 函数名称：DS1302_SPI_ReadByte
  * 功    能：模拟SPI通信读取一个字节
  * 返    回：读取到的字节
  **********************************************************
  */
unsigned char DS1302_SPI_ReadByte(void)  
{  
    unsigned char i = 0, data = 0;
	
    DS1302_Config_DIO_Input();    //DIO配置为输入模式
	
    for(i = 0; i <8; i++)
    {  
        data = data >> 1;  
		
        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == 1)
			data = data | 0x80;
		
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);  
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    }  

    return data;  
}

/**
  **********************************************************
  * 函数名称：DS1302_SPI_WriteByte
  * 功    能：模拟SPI通信写入一个字节
  * 参    数：dat 待写入的字节
  **********************************************************
  */  
void DS1302_SPI_WriteByte(u8 dat)  
{  
    u8 i = 0, data = dat;  
	
    DS1302_Config_DIO_Output();  
	
    for(i = 0; i < 8; i++)  
    {  
         if(data&0x01)//将数据送到IO口
        {
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
        }
        else 
        {
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
        }
     
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);  			//拉高时钟,保持诗句稳定，可以延时100us
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);  			//拉低时钟，准备传输下一位
		
        data = data >> 1;  
    }  
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);				//
}  

/**
**********************************************************
* 函数名称：DS1302_ReadReg
* 功    能：读取DS1302寄存器中的值
* 参    数：address 寄存器的地址（无须刻意加1，软件实现）
* 返    回：目标寄存器中的值
**********************************************************
*/
unsigned char DS1302_ReadReg(unsigned char address)  
{  
    unsigned char data = 0; 

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);   
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); 
	
    DS1302_SPI_WriteByte(address|0x01); //读取地址需要与0x01相或，最低为变成1  
    data = DS1302_SPI_ReadByte();  
	
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);  
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);  
	
    return data;  
}

/**
**********************************************************
* 函数名称：Ds1302_WriteReg
* 功    能：向DS1302寄存器中写入一个值
* 参    数：address 寄存器的地址
* 参    数：dat     待写入的值
**********************************************************
*/
void Ds1302_WriteReg( unsigned char address, unsigned char dat )  
{    
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);  
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);  
	
    DS1302_SPI_WriteByte(address);  
    DS1302_SPI_WriteByte(dat);  
	
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);  
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);  
}


/**
**********************************************************
* 函数名称：DS1302_Get_Sec
* 功    能：获取秒
* 返    回：一个整数，当前秒的值
*
* 80H  Bit  7  |  6  5  4   | 3  2  1  0 | 范围
*           CH | 10 Seconds |   Seconds  | 00-59
*                 秒 - 十位    秒 - 个位
**********************************************************
*/
unsigned char DS1302_Get_Sec( void )
{
	unsigned char val = DS1302_ReadReg(DS1302_REG_SEC);
	
	val = ((val >> 4) &0x07 ) * 10 + ( val & 0x0F ); //把BCD码转换为实际的数值
	//      取高4位 中的 低3位的值      取低四位的值
	return val;
}

/**
**********************************************************
* 函数名称：DS1302_Get_Min
* 功    能：获取分
* 返    回：一个整数，当前分的值
*
* 82H  Bit  7  6  5  4 | 3  2  1  0 | 范围
*           10 Minutes |   Minutes  | 00-59
**********************************************************
*/
unsigned char DS1302_Get_Min( void )
{
	unsigned char val = DS1302_ReadReg(DS1302_REG_MIN);
	
	val = (val >> 4) * 10 + (val & 0x0F);
	
	return val;
}

/**
********************************************************************
* 函数名称：DS1302_Get_Hour
* 功    能：获取小时
* 返    回：一个整数，当前小时的值
*
* 84H  Bit     7  | 6 |   5     |   4    | 3  2  1  0 | 范围
*           12/24 | 0 |  10/AP  |10Hours |    Hours   | 1-12或0-24
* 备    注：当BIT7=1时为12小时制，此时，BIT=1代表PM
*          当BIT7=0时为24小时制，BIT5表示小时十位的数字
*          本程序未支持12小时制
*********************************************************************
*/
unsigned char DS1302_Get_Hour( void )
{
	unsigned char val = DS1302_ReadReg(DS1302_REG_HR);
	
	val =((val >> 4) & 0x03) * 10 + (val & 0x0F);
	
	return val;
}

/**
********************************************************************
* 函数名称：DS1302_Get_Date
* 功    能：获取日的值
* 返    回：一个整数，当前日的值
*
* 8+H  Bit     7  6 | 5  4   | 3  2  1  0 |          范围
*              0  0 | 10DATE |    DATE    |01-28/29或01-30或01-31
*********************************************************************
*/
unsigned char DS1302_Get_Date( void )
{
	unsigned char val = DS1302_ReadReg(DS1302_REG_DATE);
	
	val =(val >> 4) * 10 + (val & 0x0F);
	
	return val;
}

/**
********************************************************************
* 函数名称：DS1302_Get_Mouth
* 功    能：获取月份
* 返    回：一个整数，当前月份的值
*
* 88H  Bit    7  6  5 |    4     | 3  2  1  0 | 范围
*             0  0  0 | 10Mouths |   Mouths   | 01-12
*********************************************************************
*/
unsigned char DS1302_Get_Mouth( void )
{
	unsigned char val = DS1302_ReadReg(DS1302_REG_MONTH);
	
	val =(val >> 4) * 10 + (val & 0x0F);
	
	return val;
}

/**
********************************************************************
* 函数名称：DS1302_Get_Day
* 功    能：获取星期几
* 返    回：一个整数，当前星期几的值
*
* 88H  Bit    7  6  5  4  3 | 2  1  0 | 范围
*             0  0  0  0  0 |   DAY   | 01-07
*********************************************************************
*/
unsigned char DS1302_Get_Day( void )
{
	return DS1302_ReadReg(DS1302_REG_DAY); //BCD码等于真实值，无须转换
}

/**
********************************************************************
* 函数名称：DS1302_Get_Day
* 功    能：获取星期几
* 返    回：一个整数，当前星期几的值
*
* 88H  Bit    7  6  5  4 | 3  2  1  0 | 范围
*               10YEAR   |    YEAR    | 00-99
*********************************************************************
*/
unsigned char DS1302_Get_Year( void )
{
	unsigned char val = DS1302_ReadReg(DS1302_REG_YEAR);
	
	val =(val >> 4) * 10 + (val & 0x0F);
	
	return val;
}


void DS1302_Set_Sec  ( unsigned char val )
{
	Ds1302_WriteReg( DS1302_REG_CONTROL, 0x00 );
	Ds1302_WriteReg( DS1302_REG_SEC, ((val/10)<<4)+val%10 );
	Ds1302_WriteReg( DS1302_REG_CONTROL, 0x80 );
}
void DS1302_Set_Min  ( unsigned char val )
{
	Ds1302_WriteReg( DS1302_REG_CONTROL, 0x00 );
	Ds1302_WriteReg( DS1302_REG_MIN, ((val/10)<<4)+val%10 );
	Ds1302_WriteReg( DS1302_REG_CONTROL, 0x80 );
}
void DS1302_Set_Hour ( unsigned char val )
{
	Ds1302_WriteReg( DS1302_REG_CONTROL, 0x00 );
	Ds1302_WriteReg( DS1302_REG_HR, ((val/10)<<4)+val%10 );
	Ds1302_WriteReg( DS1302_REG_CONTROL, 0x80 );
}
void DS1302_Set_Date ( unsigned char val )
{
	Ds1302_WriteReg( DS1302_REG_CONTROL, 0x00 );
	Ds1302_WriteReg( DS1302_REG_DATE, ((val/10)<<4)+val%10 );
	Ds1302_WriteReg( DS1302_REG_CONTROL, 0x80 );
}
void DS1302_Set_Mouth( unsigned char val )
{
	Ds1302_WriteReg( DS1302_REG_CONTROL, 0x00 );
	Ds1302_WriteReg( DS1302_REG_MONTH, ((val/10)<<4)+val%10 );
	Ds1302_WriteReg( DS1302_REG_CONTROL, 0x80 );
}
void DS1302_Set_Day  ( unsigned char val )
{
	Ds1302_WriteReg( DS1302_REG_CONTROL, 0x00 );
	Ds1302_WriteReg( DS1302_REG_DAY, val );
	Ds1302_WriteReg( DS1302_REG_CONTROL, 0x80 );
}
void DS1302_Set_Year ( unsigned char val )
{
	Ds1302_WriteReg( DS1302_REG_CONTROL, 0x00 );
	Ds1302_WriteReg( DS1302_REG_YEAR, ((val/10)<<4)+val%10 );
	Ds1302_WriteReg( DS1302_REG_CONTROL, 0x80 );
}

void Set_Ds1302_star(void)
{
unsigned char temp;
Ds1302_WriteReg(0x8E,0X00); //写入允许
temp=DS1302_ReadReg(0x81); //读当前时钟
Ds1302_WriteReg(0x80,temp&0x7f);//置秒位高位为0,恢复时钟计时
Ds1302_WriteReg(0x8E,0x80); //禁止写入
}

/* End of File ------------------------------------------------------------- */

