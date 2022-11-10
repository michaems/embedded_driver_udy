#include "stm32f767xx_spi_drvier.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_overrun_interrupt_handle(SPI_Handle_t *pSPIHandle);

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

uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len)
{
    uint8_t state = pSPIHandle->TxState;

    if (state != SPI_BUSY_IN_TX)
    {
        pSPIHandle->pTxBuffer = pTxBuffer;
        pSPIHandle->TxLen = len;

        pSPIHandle->TxState = SPI_BUSY_IN_TX;

        /*Enable TXEIE flag in SR*/
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE_BIT);
    }

    return state;
}

uint8_t SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len)
{

    uint8_t state = pSPIHandle->RxState;

    if (state != SPI_BUSY_IN_RX)
    {
        pSPIHandle->pRxBuffer = pRxBuffer;
        pSPIHandle->RxLen = len;

        pSPIHandle->RxState = SPI_BUSY_IN_RX;

        /*Enable TXEIE flag in SR*/
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE_BIT);
    }

    return state;
}

/*IRQ Configuration and ISR handling*/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
        {
            if (IRQNumber <= 31)
            {
                *NVIC_ISER0 |= (1 << IRQNumber);
            }
            else if(IRQNumber > 31 && IRQNumber < 64)
            {
                *NVIC_ISER1 |= (1 << (IRQNumber % 32));
            }
            else if (IRQNumber >= 64 && IRQNumber < 96)
            {
                *NVIC_ISER3 |= (1 << (IRQNumber % 64));
            }
        }
        else
        {
            if (IRQNumber <= 31)
            {
                *NVIC_ICER0 |= (1 << IRQNumber);
            }
            else if(IRQNumber > 31 && IRQNumber < 64)
            {
                *NVIC_ICER1 |= (1 << (IRQNumber % 32));
            }
            else if (IRQNumber >= 64 && IRQNumber < 96)
            {
                *NVIC_ICER3 |= (1 << (IRQNumber % 64));
            }
        }

}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx, iprx_section;

    iprx = IRQNumber / 4;
    iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NUM_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
    //Let's check for TXE
    uint8_t temp_1, temp_2;

    temp_1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE_BIT);
    temp_2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE_BIT);

    if (temp_1 && temp_2)
    {
        //Handle TXE
        spi_txe_interrupt_handle(pHandle);
    }

    temp_1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE_BIT);
    temp_2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE_BIT);

    if (temp_1 && temp_2)
    {
        //Handle TXE
        spi_rxne_interrupt_handle(pHandle);
    }

    temp_1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR_BIT);
    temp_2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE_BIT);

    if (temp_1 && temp_2)
    {
        //Handle TXE
        spi_overrun_interrupt_handle(pHandle);
    }

}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
    pSPIHandle->TxLen--;
    pSPIHandle->pTxBuffer++;

    if (!pSPIHandle->TxLen)
    {
        pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE_BIT);
        pSPIHandle->pTxBuffer = NULL;
        pSPIHandle->TxLen = 0;
        pSPIHandle->TxState = SPI_READY;

        SPI_ApplicationCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
    }
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    *(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
    pSPIHandle->RxLen--;
    pSPIHandle->pRxBuffer--;

    if (!pSPIHandle->RxLen)
    {
        pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE_BIT);
        pSPIHandle->pRxBuffer = NULL;
        pSPIHandle->RxLen = 0;
        pSPIHandle->RxState = SPI_READY;
        SPI_ApplicationCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
    }
}

static void spi_overrun_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    //1. Clear OVR register
    if (pSPIHandle->TxState != SPI_BUSY_IN_TX)
    {
        uint8_t temp = pSPIHandle->pSPIx->DR;
        temp = pSPIHandle->pSPIx->SR;
        (void)temp;
        //2. Call app. callback function
        SPI_ApplicationCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
    }
}

__attribute__((weak)) void SPI_ApplicationCallback(SPI_Handle_t *pSPIHandle, uint8_t appEv)
{

}
