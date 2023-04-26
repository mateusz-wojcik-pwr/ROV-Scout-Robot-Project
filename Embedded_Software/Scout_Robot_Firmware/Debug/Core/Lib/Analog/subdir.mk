################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Lib/Analog/Analog.c 

OBJS += \
./Core/Lib/Analog/Analog.o 

C_DEPS += \
./Core/Lib/Analog/Analog.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Lib/Analog/%.o Core/Lib/Analog/%.su Core/Lib/Analog/%.cyclo: ../Core/Lib/Analog/%.c Core/Lib/Analog/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Lib -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Lib-2f-Analog

clean-Core-2f-Lib-2f-Analog:
	-$(RM) ./Core/Lib/Analog/Analog.cyclo ./Core/Lib/Analog/Analog.d ./Core/Lib/Analog/Analog.o ./Core/Lib/Analog/Analog.su

.PHONY: clean-Core-2f-Lib-2f-Analog

