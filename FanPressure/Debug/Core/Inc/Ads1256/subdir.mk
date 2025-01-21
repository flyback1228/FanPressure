################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/Ads1256/ads1256.c 

C_DEPS += \
./Core/Inc/Ads1256/ads1256.d 

OBJS += \
./Core/Inc/Ads1256/ads1256.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/Ads1256/%.o Core/Inc/Ads1256/%.su Core/Inc/Ads1256/%.cyclo: ../Core/Inc/Ads1256/%.c Core/Inc/Ads1256/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc/Ads1256 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-Ads1256

clean-Core-2f-Inc-2f-Ads1256:
	-$(RM) ./Core/Inc/Ads1256/ads1256.cyclo ./Core/Inc/Ads1256/ads1256.d ./Core/Inc/Ads1256/ads1256.o ./Core/Inc/Ads1256/ads1256.su

.PHONY: clean-Core-2f-Inc-2f-Ads1256

