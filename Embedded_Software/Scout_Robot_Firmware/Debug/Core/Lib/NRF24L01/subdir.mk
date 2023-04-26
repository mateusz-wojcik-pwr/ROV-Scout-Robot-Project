################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Lib/NRF24L01/NRF24L01.c 

OBJS += \
./Core/Lib/NRF24L01/NRF24L01.o 

C_DEPS += \
./Core/Lib/NRF24L01/NRF24L01.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Lib/NRF24L01/%.o Core/Lib/NRF24L01/%.su Core/Lib/NRF24L01/%.cyclo: ../Core/Lib/NRF24L01/%.c Core/Lib/NRF24L01/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Lib -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Lib-2f-NRF24L01

clean-Core-2f-Lib-2f-NRF24L01:
	-$(RM) ./Core/Lib/NRF24L01/NRF24L01.cyclo ./Core/Lib/NRF24L01/NRF24L01.d ./Core/Lib/NRF24L01/NRF24L01.o ./Core/Lib/NRF24L01/NRF24L01.su

.PHONY: clean-Core-2f-Lib-2f-NRF24L01

