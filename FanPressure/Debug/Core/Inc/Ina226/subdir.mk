################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/Ina226/ina226.c 

C_DEPS += \
./Core/Inc/Ina226/ina226.d 

OBJS += \
./Core/Inc/Ina226/ina226.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/Ina226/%.o Core/Inc/Ina226/%.su Core/Inc/Ina226/%.cyclo: ../Core/Inc/Ina226/%.c Core/Inc/Ina226/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc/Ads1256 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-Ina226

clean-Core-2f-Inc-2f-Ina226:
	-$(RM) ./Core/Inc/Ina226/ina226.cyclo ./Core/Inc/Ina226/ina226.d ./Core/Inc/Ina226/ina226.o ./Core/Inc/Ina226/ina226.su

.PHONY: clean-Core-2f-Inc-2f-Ina226

