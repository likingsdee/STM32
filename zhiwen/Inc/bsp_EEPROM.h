#ifndef __I2C_EEPROM_H__
#define	__I2C_EEPROM_H__
/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
#define I2C_OWN_ADDRESS                            0x0A              // stm32本机I2C地址
#define I2C_SPEEDCLOCK                             400000            // I2C通信速率(最大为400K)
#define I2C_DUTYCYCLE                              I2C_DUTYCYCLE_2   // I2C占空比模式：1/2 

#define EEPROM_I2Cx                                I2C1
#define EEPROM_I2C_RCC_CLK_ENABLE()                __HAL_RCC_I2C1_CLK_ENABLE()
#define EEPROM_I2C_RCC_CLK_DISABLE()               __HAL_RCC_I2C1_CLK_DISABLE()

#define EEPROM_I2C_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOB_CLK_ENABLE()
#define EEPROM_I2C_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOB_CLK_DISABLE()   
#define EEPROM_I2C_GPIO_PORT                       GPIOB   
#define EEPROM_I2C_SCL_PIN                         GPIO_PIN_6
#define EEPROM_I2C_SDA_PIN                         GPIO_PIN_7

/* 
 * EEPROM 2kb = 2048bit = 2048/8 B = 256 B
 * 32 pages of 8 bytes each
 *
 * Device Address
 * 1 0 1 0 A2 A1 A0 R/W
 * 1 0 1 0 0  0  0  0 = 0XA0
 * 1 0 1 0 0  0  0  1 = 0XA1 
 */
/* EEPROM Addresses defines */ 
#define EEPROM_I2C_ADDRESS                         0xA2

/* 扩展变量 ------------------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c_eeprom;

/* 函数声明 ------------------------------------------------------------------*/
//void               MX_I2C_EEPROM_Init(void);
void               I2C_EEPROM_WriteData(uint16_t Addr, uint8_t Reg, uint8_t Value);
HAL_StatusTypeDef  I2C_EEPROM_WriteBuffer(uint16_t Addr, uint8_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length);
uint8_t            I2C_EEPROM_ReadData(uint16_t Addr, uint8_t Reg);
HAL_StatusTypeDef  I2C_EEPROM_ReadBuffer(uint16_t Addr, uint8_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length);
HAL_StatusTypeDef  I2C_EEPROM_IsDeviceReady(uint16_t DevAddress, uint32_t Trials);

#endif /* __I2C_EEPROM_H__ */

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
