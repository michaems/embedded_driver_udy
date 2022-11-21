#ifndef INC_STM32F767XX_SPI_DRVIER_H_
#define INC_STM32F767XX_SPI_DRVIER_H_

#include <stddef.h>
#include "stm32f767xx.h"

typedef struct _spi_config_t
{
    uint8_t SPI_DeviceMode;
    uint8_t SPI_BusConfig;
    uint8_t SPI_SclkSpeed;
    uint8_t SPI_CPOL;
    uint8_t SPI_CPHA;
    uint8_t SPI_SSM;
}SPI_Config_t;

typedef struct _spi_handle_t
{
    SPI_RegDef_t *pSPIx;
    SPI_Config_t SPIConfig;
    uint8_t *pTxBuffer;
    uint8_t *pRxBuffer;
    uint32_t TxLen;
    uint32_t RxLen;
    uint8_t  TxState;
    uint8_t  RxState;
}SPI_Handle_t;

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIOx);

void SPI_PeriClkCtrl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);


/*Data send and receive*/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t *pRxBuffer, uint32_t len);

uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len);

/*IRQ Configuration and ISR handling*/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSMConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

void SPI_ApplicationCallback(SPI_Handle_t *pSPIHandle, uint8_t appEv);

#endif /* INC_STM32F767XX_SPI_DRVIER_H_ */
