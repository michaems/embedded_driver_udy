#include "stm32f767xx_spi_drvier.h"

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
    SPI_PeriClkCtrl(pSPIHandle->pSPIx, ENABLE);

    //CR1 register
    uint32_t temp_reg = 0;

    //1. configure device mode
    temp_reg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR_BIT;

    //2. Bus config
    if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
    {
        //bidi mode should be cleared
        temp_reg &= ~(1 << SPI_CR1_BIDIMODE_BIT);

    }
    else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
    {
        //bidi mode should be set
        temp_reg |= (1 << SPI_CR1_BIDIMODE_BIT);
    }
    else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX_ONLY)
    {
        //bidi mode should be cleared and RXONLY bit must be set
        temp_reg &= ~(1 << SPI_CR1_BIDIMODE_BIT);
        temp_reg |= (1 << SPI_CR1_RXONLY_BIT);
    }

    //3. SPI Serial Clock speed
    temp_reg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BAUD_RATE_CTRL_BIT;

    //5. Configure the CPOL
    temp_reg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL_BIT;

    //6. Configure the CPHA
    temp_reg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA_BIT;

    //7. Config SSM
    temp_reg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM_BIT;

    //8. Save to CR1
    pSPIHandle->pSPIx->CR1 = temp_reg;
}

void SPI_DeInit(SPI_RegDef_t *pSPIOx)
{

}

void SPI_PeriClkCtrl(SPI_RegDef_t  *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi)
    {
        if (pSPIx == SPI1)
        {
            SPI1_PCLK_EN();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLK_EN();
        }
        else if(pSPIx == SPI3)
        {
            SPI3_PCLK_EN();
        }

    }
    else
    {
        //do this later
    }
}


/*Data send and receive*/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
    /*this is blocking call*/
    while (len > 0)
    {
        //1. Wait until TXE is set
        while((pSPIx->SR & (1 << 1)) == 0);

        //2. Check the format (16 bit will be done later. pSpix->DR = *((uint16_t *)pTxBuffer); len--; len--;
        pSPIx->DR = *pTxBuffer;
        len--;
        pTxBuffer++;
    }
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SPE_BIT);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE_BIT);
    }
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SSI_BIT);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI_BIT);
    }
}

void SPI_SSMConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SSM_BIT);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSM_BIT);
    }
}

void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
    while (len > 0)
    {
        while((pSPIx->SR & (1<<0)) == 0);

        *pRxBuffer = pSPIx->DR;

        pRxBuffer++;
        len--;
    }
}

/*IRQ Configuration and ISR handling*/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{

}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

}
