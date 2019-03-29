/**
  ******************************************************************************
  * @file    DS1302.c
  * @version ���Ľݱ��� 1.0
  * @brief   DS1302����
  ******************************************************************************
  */
  
#include "DS1302.h"

/* �궨�� ------------------------------------------------ */
#define u8 uint8_t
/* �ڲ��������� ------------------------------------------ */
void DS1302_Config_DIO_Input(void);
void DS1302_Config_DIO_Output(void);
  


/**
  **********************************************************
  * �������ƣ�DS1302_Config_DIO_Input
  * ��    �ܣ���DIO������Ϊ����ģʽ�����ڶ�ȡ����	
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
  * �������ƣ�DS1302_Config_DIO_Output
  * ��    �ܣ���DIO������Ϊ���ģʽ������д������	
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
  * �������ƣ�DS1302_SPI_ReadByte
  * ��    �ܣ�ģ��SPIͨ�Ŷ�ȡһ���ֽ�
  * ��    �أ���ȡ�����ֽ�
  **********************************************************
  */
unsigned char DS1302_SPI_ReadByte(void)  
{  
    unsigned char i = 0, data = 0;
	
    DS1302_Config_DIO_Input();    //DIO����Ϊ����ģʽ
	
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
  * �������ƣ�DS1302_SPI_WriteByte
  * ��    �ܣ�ģ��SPIͨ��д��һ���ֽ�
  * ��    ����dat ��д����ֽ�
  **********************************************************
  */  
void DS1302_SPI_WriteByte(u8 dat)  
{  
    u8 i = 0, data = dat;  
	
    DS1302_Config_DIO_Output();  
	
    for(i = 0; i < 8; i++)  
    {  
         if(data&0x01)//�������͵�IO��
        {
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
        }
        else 
        {
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
        }
     
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);  			//����ʱ��,����ʫ���ȶ���������ʱ100us
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);  			//����ʱ�ӣ�׼��������һλ
		
        data = data >> 1;  
    }  
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);				//
}  

/**
**********************************************************
* �������ƣ�DS1302_ReadReg
* ��    �ܣ���ȡDS1302�Ĵ����е�ֵ
* ��    ����address �Ĵ����ĵ�ַ����������1�����ʵ�֣�
* ��    �أ�Ŀ��Ĵ����е�ֵ
**********************************************************
*/
unsigned char DS1302_ReadReg(unsigned char address)  
{  
    unsigned char data = 0; 

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);   
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); 
	
    DS1302_SPI_WriteByte(address|0x01); //��ȡ��ַ��Ҫ��0x01������Ϊ���1  
    data = DS1302_SPI_ReadByte();  
	
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);  
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);  
	
    return data;  
}

/**
**********************************************************
* �������ƣ�Ds1302_WriteReg
* ��    �ܣ���DS1302�Ĵ�����д��һ��ֵ
* ��    ����address �Ĵ����ĵ�ַ
* ��    ����dat     ��д���ֵ
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
* �������ƣ�DS1302_Get_Sec
* ��    �ܣ���ȡ��
* ��    �أ�һ����������ǰ���ֵ
*
* 80H  Bit  7  |  6  5  4   | 3  2  1  0 | ��Χ
*           CH | 10 Seconds |   Seconds  | 00-59
*                 �� - ʮλ    �� - ��λ
**********************************************************
*/
unsigned char DS1302_Get_Sec( void )
{
	unsigned char val = DS1302_ReadReg(DS1302_REG_SEC);
	
	val = ((val >> 4) &0x07 ) * 10 + ( val & 0x0F ); //��BCD��ת��Ϊʵ�ʵ���ֵ
	//      ȡ��4λ �е� ��3λ��ֵ      ȡ����λ��ֵ
	return val;
}

/**
**********************************************************
* �������ƣ�DS1302_Get_Min
* ��    �ܣ���ȡ��
* ��    �أ�һ����������ǰ�ֵ�ֵ
*
* 82H  Bit  7  6  5  4 | 3  2  1  0 | ��Χ
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
* �������ƣ�DS1302_Get_Hour
* ��    �ܣ���ȡСʱ
* ��    �أ�һ����������ǰСʱ��ֵ
*
* 84H  Bit     7  | 6 |   5     |   4    | 3  2  1  0 | ��Χ
*           12/24 | 0 |  10/AP  |10Hours |    Hours   | 1-12��0-24
* ��    ע����BIT7=1ʱΪ12Сʱ�ƣ���ʱ��BIT=1����PM
*          ��BIT7=0ʱΪ24Сʱ�ƣ�BIT5��ʾСʱʮλ������
*          ������δ֧��12Сʱ��
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
* �������ƣ�DS1302_Get_Date
* ��    �ܣ���ȡ�յ�ֵ
* ��    �أ�һ����������ǰ�յ�ֵ
*
* 8+H  Bit     7  6 | 5  4   | 3  2  1  0 |          ��Χ
*              0  0 | 10DATE |    DATE    |01-28/29��01-30��01-31
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
* �������ƣ�DS1302_Get_Mouth
* ��    �ܣ���ȡ�·�
* ��    �أ�һ����������ǰ�·ݵ�ֵ
*
* 88H  Bit    7  6  5 |    4     | 3  2  1  0 | ��Χ
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
* �������ƣ�DS1302_Get_Day
* ��    �ܣ���ȡ���ڼ�
* ��    �أ�һ����������ǰ���ڼ���ֵ
*
* 88H  Bit    7  6  5  4  3 | 2  1  0 | ��Χ
*             0  0  0  0  0 |   DAY   | 01-07
*********************************************************************
*/
unsigned char DS1302_Get_Day( void )
{
	return DS1302_ReadReg(DS1302_REG_DAY); //BCD�������ʵֵ������ת��
}

/**
********************************************************************
* �������ƣ�DS1302_Get_Day
* ��    �ܣ���ȡ���ڼ�
* ��    �أ�һ����������ǰ���ڼ���ֵ
*
* 88H  Bit    7  6  5  4 | 3  2  1  0 | ��Χ
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
Ds1302_WriteReg(0x8E,0X00); //д������
temp=DS1302_ReadReg(0x81); //����ǰʱ��
Ds1302_WriteReg(0x80,temp&0x7f);//����λ��λΪ0,�ָ�ʱ�Ӽ�ʱ
Ds1302_WriteReg(0x8E,0x80); //��ֹд��
}

/* End of File ------------------------------------------------------------- */

