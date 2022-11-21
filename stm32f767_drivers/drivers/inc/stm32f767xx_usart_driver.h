
#ifndef INC_STM32F767XX_USART_DRIVER_H_
#define INC_STM32F767XX_USART_DRIVER_H_

#include <stdint.h>
#include <stddef.h>
#include "stm32f767xx.h"

/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX 0
#define USART_MODE_ONLY_RX 1
#define USART_MODE_TXRX    2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200                 1200
#define USART_STD_BAUD_2400                 2400
#define USART_STD_BAUD_9600                 9600
#define USART_STD_BAUD_19200                19200
#define USART_STD_BAUD_38400                38400
#define USART_STD_BAUD_57600                57600
#define USART_STD_BAUD_115200               115200
#define USART_STD_BAUD_230400               230400
#define USART_STD_BAUD_460800               460800
#define USART_STD_BAUD_921600               921600
#define USART_STD_BAUD_2M                   2000000
#define SUART_STD_BAUD_3M                   3000000

/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_DISABLE  0
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_EN_ODD   2

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOWCTRL_NONE     0
#define USART_HW_FLOWCTRL_CTS      1
#define USART_HW_FLOWCTRL_RTS      2
#define USART_HW_FLOWCTRL_CTS_RTS  3

typedef struct _usart_config_t
{
    uint8_t USART_Mode;
    uint8_t USART_NoOfStopBits;
    uint8_t USART_WordLength;
    uint8_t USART_ParityControl;
    uint8_t USART_HWFlowControl;
    uint32_t USART_Baud;
}USART_Config_t;

typedef struct _usart_handle_t
{
    USART_RegDef_t *pUSARTx;
    USART_Config_t USART_Config;
}USART_Handle_t;

void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);

uint8_t USART_GetFlagsStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

/*Send and Receive Data*/
void USART_SendData(USART_RegDef_t *pUSARTx, uint8_t *pTxBuffer, uint32_t len);
void USART_ReceiveData(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t len);
void USART_SendDataIT(USART_RegDef_t *pUSARTx, uint8_t *pTxBuffer, uint32_t len);
void USART_ReceiveDataIT(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t len);

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

void USART_IRQHandling(USART_Handle_t *pHandle);


#endif /* INC_STM32F767XX_USART_DRIVER_H_ */
