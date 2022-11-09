#include <string.h>

#include "stm32f767xx.h"
#include "stm32f767xx_spi_drvier.h"
#include "stm32f767xx_gpio_driver.h"


/*
 * PB14 - SPI2 MISO
 * PB15 - SPI2 MOSI
 * PB13 - SPI2 SCK
 * PB12 - SPI2 NCC
 * ALT function mode : 5
 */

void SPI2_GPIOInits(void);

void SPI2_Inits(void);



int main(void)
{

    char user_data[] = "Hello World!";

    SPI2_GPIOInits();

    /*Enable SPI2*/

    SPI2_Inits();

    SPI_SSIConfig(SPI2, ENABLE);

    SPI_PeripheralControl(SPI2, ENABLE);
    SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

    SPI_PeripheralControl(SPI2, DISABLE);

    while(1);

    return 0;
}


void SPI2_Inits(void)
{
    SPI_Handle_t SPI2_Handle;

    SPI2_Handle.pSPIx = SPI2;
    SPI2_Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPI2_Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI2_Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV_2; //8MHz
    SPI2_Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPI2_Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPI2_Handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

    SPI_Init(&SPI2_Handle);
}

void SPI2_GPIOInits(void)
{
    GPIO_Handle_t SPI_Pins;

    SPI_Pins.pGPIOx = GPIOB;

    SPI_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPI_Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    SPI_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPI_Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    SPI_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    //SCLK
    SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&SPI_Pins);

    //MOSI
    SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&SPI_Pins);

    //MISO
    SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    GPIO_Init(&SPI_Pins);

    //NSS
    SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&SPI_Pins);
}
