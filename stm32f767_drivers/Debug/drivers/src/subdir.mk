################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU_TOOLS_for_STM32(10_3_2021_10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/stm32f767xx_gpio_driver.c 

OBJS += \
./drivers/src/stm32f767xx_gpio_driver.o 

C_DEPS += \
./drivers/src/stm32f767xx_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/%.o drivers/src/%.su: ../drivers/src/%.c drivers/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DSTM32 -DNUCLEO_F767ZI -DSTM32F7 -DSTM32F767ZITx -c -I../Inc -I../drivers/inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-src

clean-drivers-2f-src:
	-$(RM) ./drivers/src/stm32f767xx_gpio_driver.d ./drivers/src/stm32f767xx_gpio_driver.o ./drivers/src/stm32f767xx_gpio_driver.su

.PHONY: clean-drivers-2f-src
