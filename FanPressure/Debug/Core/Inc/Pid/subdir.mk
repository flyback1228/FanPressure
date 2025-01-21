################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Inc/Pid/pid.cpp 

OBJS += \
./Core/Inc/Pid/pid.o 

CPP_DEPS += \
./Core/Inc/Pid/pid.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/Pid/%.o Core/Inc/Pid/%.su Core/Inc/Pid/%.cyclo: ../Core/Inc/Pid/%.cpp Core/Inc/Pid/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc/Ads1256 -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-Pid

clean-Core-2f-Inc-2f-Pid:
	-$(RM) ./Core/Inc/Pid/pid.cyclo ./Core/Inc/Pid/pid.d ./Core/Inc/Pid/pid.o ./Core/Inc/Pid/pid.su

.PHONY: clean-Core-2f-Inc-2f-Pid

