

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/
#define HMI_USARTx                                 USART2
#define HMI_USARTx_BAUDRATE                        115200
#define HMI_USART_RCC_CLK_ENABLE()                 __HAL_RCC_USART2_CLK_ENABLE()
#define HMI_USART_RCC_CLK_DISABLE()                __HAL_RCC_USART2_CLK_DISABLE()
#define husartx_HMI                                huart2

#define HMI_USARTx_GPIO_ClK_ENABLE()               __HAL_RCC_GPIOA_CLK_ENABLE()
#define HMI_USARTx_PORT                            GPIOA
#define HMI_USARTx_Tx_PIN                          GPIO_PIN_2
#define HMI_USARTx_Rx_PIN                          GPIO_PIN_3

#define HMI_USARTx_IRQHANDLER                      USART2_IRQHandler
#define HMI_USARTx_IRQn                            USART2_IRQn
 
/* ��չ���� ------------------------------------------------------------------*/
extern UART_HandleTypeDef husartx_HMI;

/* �������� ------------------------------------------------------------------*/
void HMI_USARTx_Init(void);
void nfc_WakeUp(void);
void  nfc_InListPassiveTarget(void);
void nfc_read(void);
void nfc_write(uint8_t write_data);
void nfc_PsdVerifyKeyA(void);
