################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU_TOOLS_for_STM32(10_3_2021_10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/006_spi_tx_testing.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/006_spi_tx_testing.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/006_spi_tx_testing.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DSTM32 -DNUCLEO_F767ZI -DSTM32F7 -DSTM32F767ZITx -c -I../Inc -I../drivers/inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/006_spi_tx_testing.d ./Src/006_spi_tx_testing.o ./Src/006_spi_tx_testing.su ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

