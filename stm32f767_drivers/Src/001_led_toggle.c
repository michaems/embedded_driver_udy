#include "stm32f767xx.h"
#include "stm32f767xx_gpio_driver.h"

void delay(void);

int main(void)
{
    GPIO_Handle_t gpio_led;

    /*1. Initialize*/
    gpio_led.pGPIOx = GPIOB;
    gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
    gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
    gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


    GPIO_PeriClkCtrl(GPIOB, ENABLE);

    GPIO_Init(&gpio_led);

    for (;;)
    {
        GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_7);
        delay();
    }

    return 0;
}

void delay(void)
{
    for (int i=0; i < (651600); i++);
}

