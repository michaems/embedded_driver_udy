################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU_TOOLS_for_STM32(10_3_2021_10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Startup/startup_stm32f767zitx.s 

OBJS += \
./Startup/startup_stm32f767zitx.o 

S_DEPS += \
./Startup/startup_stm32f767zitx.d 


# Each subdirectory must supply rules for building sources it contributes
Startup/%.o: ../Startup/%.s Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m7 -g3 -DDEBUG -c -I"C:/Users/ett14478/Documents/stm32_cube_workspace/embedded_driver_udy/stm32f767_drivers/drivers/inc" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=soft -mthumb -o "$@" "$<"

clean: clean-Startup

clean-Startup:
	-$(RM) ./Startup/startup_stm32f767zitx.d ./Startup/startup_stm32f767zitx.o

.PHONY: clean-Startup

