#ifndef __STM32F767xx_I2C_DRIVER_H__
#define __STM32F767xx_I2C_DRIVER_H__

#include <stdint.h>
#include "stm32f767xx.h"

#define I2C_SCL_SPEED_SM   100000
#define I2C_SCL_SPEED_FM4K 400000
#define I2C_SCL_SPEED_FM2K 200000

/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE
#define I2C_ACK_DISABLE

/*
 * @I2C_FMDytyCycle
 */
#define I2C_FM_DUTY_CYCLE

typedef struct _i2c_config_t
{
    uint32_t I2C_SCLSpeed;
    uint8_t  I2C_DeviceAddress;
    uint8_t  I2C_ACKControl;
    uint8_t I2C_FMDutyCycle;
}I2C_Config_t;

typedef struct _i2c_handle_t
{
    I2C_RegDef_t *pI2Cx;
    I2C_Config_t I2C_Config;
}I2C_Handle_t;


void I2C_Init(I2C_Handle_t *pSPIHandle);
void I2C_DeInit(I2C_RegDef_t *pSPIOx);

void I2C_PeriClkCtrl(I2C_RegDef_t *pSPIx, uint8_t EnOrDi);


/*Data send and receive*/


/*IRQ Configuration and ISR handling*/
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

void I2C_ApplicationCallback(I2C_Handle_t *pI2CHandle, uint8_t appEv);

#endif /*__STM32F767xx_I2C_DRIVER_H__*/
