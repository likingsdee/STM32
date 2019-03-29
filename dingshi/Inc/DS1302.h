/**
  ******************************************************************************
  * @file    DS1302.h
  * @author  Velscode
  * @email   velscode@gmail.com
  * @version ѧϰ��ȫ�� 1.0
  * @brief   ����STM32F1x��ʱ��оƬDS1302����
  ******************************************************************************
  */
 
#ifndef __DS1302_H
#define __DS1302_H

/* ͷ�ļ�   ------------------------------------------------------------------ */
#include "stm32f1xx_hal.h"




/* �Ĵ�����ַ   -------------------------------------------------------------- */
#define DS1302_REG_SEC          0x80        //�����ݵ�ַ  
#define DS1302_REG_MIN          0x82        //�����ݵ�ַ  
#define DS1302_REG_HR           0x84        //ʱ���ݵ�ַ  
#define DS1302_REG_DATE         0x86        //�����ݵ�ַ  
#define DS1302_REG_MONTH        0x88        //�����ݵ�ַ  
#define DS1302_REG_DAY          0x8a        //���ڼ����ݵ�ַ  
#define DS1302_REG_YEAR         0x8c        //�����ݵ�ַ  
#define DS1302_REG_CONTROL      0x8e        //д�����Ĵ�����ַ  
#define DS1302_REG_CHARGER      0x90        //������Ĵ���

/* ȫ�ֱ��� ------------------------------------------------------------------ */

/* �������� ------------------------------------------------------------------ */
void DS1302_Init(void);	//��ʼ��DS1302

void Ds1302_WriteReg( unsigned char address, unsigned char dat );
unsigned char DS1302_ReadReg(unsigned char address);

unsigned char DS1302_Get_Sec  ( void );
unsigned char DS1302_Get_Min  ( void );
unsigned char DS1302_Get_Hour ( void );
unsigned char DS1302_Get_Date ( void );
unsigned char DS1302_Get_Mouth( void );
unsigned char DS1302_Get_Day  ( void );
unsigned char DS1302_Get_Year ( void );

void DS1302_Set_Sec  ( unsigned char val );
void DS1302_Set_Min  ( unsigned char val );
void DS1302_Set_Hour ( unsigned char val );
void DS1302_Set_Date ( unsigned char val );
void DS1302_Set_Mouth( unsigned char val );
void DS1302_Set_Day  ( unsigned char val );
void DS1302_Set_Year ( unsigned char val );
void Set_Ds1302_star(void);

#endif /* __DS1302_H */
/* End of File ------------------------------------------------------------- */

