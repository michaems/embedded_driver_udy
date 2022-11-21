#include <string.h>

#include "stm32f767xx.h"
#include "stm32f767xx_spi_drvier.h"
#include "stm32f767xx_gpio_driver.h"

#include "stm32f767xx_usart_driver.h"


void USART3_GPIOInit(void);
void USART3_Init(void);

int main(void)
{
    USART3_GPIOInit();
    USART3_Init();
    USART_PeripheralControl(USART3, ENABLE);

    USART_SendData(&usart2_handle, (uint8_t *)msg, strlen(msg));

    return 0;
}

void USART3_GPIOInit(void)
{

}

void USART3_Init(void)
{

}
