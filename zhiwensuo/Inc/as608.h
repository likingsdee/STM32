#include "main.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include "usart.h"

typedef struct
{
	uint16_t PS_max;//指纹最大容量
	uint8_t  PS_level;//安全等级
	uint32_t PS_addr;
	uint8_t  PS_size;//通讯数据包大小
	uint8_t  PS_N;//波特率基数N
}SysPara;

void ShowErrMessage(uint8_t ensure);
uint8_t PS_GetImage(void);
uint8_t PS_GenChar(uint8_t BufferID);
uint8_t PS_Match(void);
uint8_t PS_RegModel(void);
uint8_t PS_StoreChar(void);
uint8_t PS_ReadSysPara(SysPara *p);
uint8_t PS_HighSpeedSearch(void);
uint8_t PS_ValidTempleteNum(uint16_t *ValidN);
void press_FR(void);
void Add_FR(void);
