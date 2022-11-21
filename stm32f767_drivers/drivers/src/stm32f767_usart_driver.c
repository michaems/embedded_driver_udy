#include "stm32f767xx_usart_driver.h"

void USART_Init(USART_Handle_t *pUSARTHandle)
{

    //Temporary variable
    uint32_t tempreg=0;

/******************************** Configuration of CR1******************************************/

    //Implement the code to enable the Clock for given USART peripheral
    if (pUSARTHandle->pUSARTx == USART1)
    {
        USART1_PCLK_EN();
    }
    else if (pUSARTHandle->pUSARTx == USART2)
    {
        USART2_PCLK_EN();
    }
    else if (pUSARTHandle->pUSARTx == USART3)
    {
        USART3_PCLK_EN();
    }
    else if (pUSARTHandle->pUSARTx == USART6)
    {
        USART6_PCLK_EN();
    }



    //Enable USART Tx and Rx engines according to the USART_Mode configuration item
    if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
    {
        tempreg |= USART_CR1_RE_BITMASK;

    }
    else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
    {
        tempreg |= USART_CR1_TE_BITMASK;

    }
    else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
    {
        //Implement the code to enable the both Transmitter and Receiver bit fields
        tempreg |= ( USART_CR1_RE_BITMASK | USART_CR1_TE_BITMASK );
    }

    //Implement the code to configure the Word length configuration item
    tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M1_BIT;


    //Configuration of parity control bit fields
    if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
    {
        //Implement the code to enale the parity control
        tempreg |= ( 0 << USART_CR1_PS_BIT );

        //Implement the code to enable EVEN parity
        //Not required because by default EVEN parity will be selected once you enable the parity control

    }else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
    {
        //Implement the code to enable the parity control
        tempreg |= USART_CR1_PCE_BITMASK;

        //Implement the code to enable ODD parity
        tempreg |= ( 1 << USART_CR1_PS_BIT);

    }

   //Program the CR1 register
    pUSARTHandle->pUSARTx->CR1 = tempreg;

/******************************** Configuration of CR2******************************************/

    tempreg=0;

    //Implement the code to configure the number of stop bits inserted during USART frame transmission
    tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << 12;

    //Program the CR2 register
    pUSARTHandle->pUSARTx->CR2 |= tempreg;

/******************************** Configuration of CR3******************************************/

    tempreg=0;

    //Configuration of USART hardware flow control
    if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOWCTRL_CTS)
    {
        //Implement the code to enable CTS flow control
        tempreg |= ( 1 << 8);


    }else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOWCTRL_RTS)
    {
        //Implement the code to enable RTS flow control
        tempreg |= (1 << 9);

    }else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOWCTRL_CTS_RTS)
    {
        //Implement the code to enable both CTS and RTS Flow control
        tempreg |= ((1 << 8) | (1 << 9));
    }


    pUSARTHandle->pUSARTx->CR3 = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

    //Implement the code to configure the baud rate
    //We will cover this in the lecture. No action required her
    USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}

void USART_DeInit(USART_RegDef_t *pUSARTx)
{

}

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{

}

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{

}

uint8_t USART_GetFlagsStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName)
{
    return 0;
}

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{

}

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
    uint32_t PCLKx;

    uint32_t usartdiv;

    uint32_t M_part, F_part;

    uint32_t tempreg = 0;

    //Get the value of APB bus clock in to the variable PCLKx
    if(pUSARTx == USART1 || pUSARTx == USART6)
    {
         //USART1 and USART6 are hanging on APB2 bus
         PCLKx = RCC_GetPCLK2Value();
    }else
    {
         PCLKx = RCC_GetPCLK1Value();
    }

    //Check for OVER8 configuration bit
    if(pUSARTx->CR1 & (1 << USART_CR1_OVER8_BIT))
    {
         //OVER8 = 1 , over sampling by 8
         usartdiv = ((25 * PCLKx) / (2 *BaudRate));
    }else
    {
         //over sampling by 16
         //TODO
    }

    //Calculate the Mantissa part
    M_part = TODO/100;

    //Place the Mantissa part in appropriate bit position . refer USART_BRR
    tempreg |= M_part << 4;

    //Extract the fraction part
    F_part = (usartdiv - (TODO * 100));

    //Calculate the final fractional
    if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
     {
        //OVER8 = 1 , over sampling by 8
        F_part = ((( F_part * TODO)+ 50) / 100)& ((uint8_t)0x07);

     }else
     {
         //over sampling by 16
         F_part = ((( F_part * TODO)+ 50) / 100) & ((uint8_t)0x0F);

     }

    //Place the fractional part in appropriate bit position . refer USART_BRR
    tempreg |= F_part;

    //copy the value of tempreg in to BRR register
    pUSARTx->BRR = tempreg;
}

/*Send and Receive Data*/
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len)
{
    uint16_t *pdata;
    //Loop over until "Len" number of bytes are transferred
    for(uint32_t i = 0 ; i < len; i++)
    {
        //Implement the code to wait until TXE flag is set in the SR
        while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_ISR_TXE_BIT));

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
        if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
        {
            //if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
            pdata = (uint16_t*) pTxBuffer;
            pUSARTHandle->pUSARTx->TDR = (*pdata & (uint16_t)0x01FF);

            //check for USART_ParityControl
            if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
            {
                //No parity is used in this transfer. so, 9bits of user data will be sent
                //Implement the code to increment pTxBuffer twice
                pTxBuffer++;
                pTxBuffer++;
            }
            else
            {
                //Parity bit is used in this transfer . so , 8bits of user data will be sent
                //The 9th bit will be replaced by parity bit by the hardware
                pTxBuffer++;
            }
        }
        else
        {
            //This is 8bit data transfer
            pUSARTHandle->pUSARTx->TDR = (*pTxBuffer  & (uint8_t)0xFF);

            //increment the buffer address
            pTxBuffer++;
        }
    }

    //Implement the code to wait till TC flag is set in the SR
    while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_ISR_TC_BIT));

}

void USART_ReceiveData(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t len)
{
    //Loop over until "Len" number of bytes are transferred
    for(uint32_t i = 0 ; i < TODO; i++)
    {
        //Implement the code to wait until RXNE flag is set in the SR
        TODO

        //Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
        if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
        {
            //We are going to receive 9bit data in a frame

            //check are we using USART_ParityControl control or not
            if(pUSARTHandle->USART_Config.USART_ParityControl == TODO)
            {
                //No parity is used. so, all 9bits will be of user data

                //read only first 9 bits. so, mask the DR with 0x01FF
                *((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)TODO);

                //Now increment the pRxBuffer two times
                TODO
            }
            else
            {
                //Parity is used, so, 8bits will be of user data and 1 bit is parity
                 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

                 //Increment the pRxBuffer
                TODO
            }
        }
        else
        {
            //We are going to receive 8bit data in a frame

            //check are we using USART_ParityControl control or not
            if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
            {
                //No parity is used , so all 8bits will be of user data

                //read 8 bits from DR
                 *pRxBuffer = TODO;
            }

            else
            {
                //Parity is used, so , 7 bits will be of user data and 1 bit is parity

                //read only 7 bits , hence mask the DR with 0X7F
                 *pRxBuffer = (uint8_t) TODO

            }

            //increment the pRxBuffer
            pRxBuffer++;
        }
    }
}

void USART_SendDataIT(USART_RegDef_t *pUSARTx, uint8_t *pTxBuffer, uint32_t len)
{
    uint8_t txstate = pUSARTHandle->TODO;

    if(txstate != USART_BUSY_IN_TX)
    {
        pUSARTHandle->TODO = Len;
        pUSARTHandle->pTxBuffer = TODO;
        pUSARTHandle->TxBusyState = TODO;

        //Implement the code to enable interrupt for TXE
        TODO


        //Implement the code to enable interrupt for TC
        TODO


    }

    return txstate;
}

void USART_ReceiveDataIT(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t len)
{
    uint8_t rxstate = pUSARTHandle->TODO;

    if(rxstate != TODO)
    {
        pUSARTHandle->RxLen = len;
        pUSARTHandle->pRxBuffer = TODO;
        pUSARTHandle->RxBusyState = TODO;

        //Implement the code to enable interrupt for RXNE
        //TODO

    }

    return rxstate;

}

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{

}

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}

void USART_IRQHandling(USART_Handle_t *pHandle)
{

}

