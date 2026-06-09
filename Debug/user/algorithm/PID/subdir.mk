################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../user/algorithm/PID/pid.c 

OBJS += \
./user/algorithm/PID/pid.o 

C_DEPS += \
./user/algorithm/PID/pid.d 


# Each subdirectory must supply rules for building sources it contributes
user/algorithm/PID/%.o user/algorithm/PID/%.su user/algorithm/PID/%.cyclo: ../user/algorithm/PID/%.c user/algorithm/PID/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I"C:/Users/FMI/Documents/GitHub/wheel_leg/user/bsp/bsp_can" -I"C:/Users/FMI/Documents/GitHub/wheel_leg/Drivers/CMSIS/DSP/Include" -I"C:/Users/FMI/Documents/GitHub/wheel_leg/user/devices/dm_motor" -I"C:/Users/FMI/Documents/GitHub/wheel_leg/user/devices/feetech" -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-user-2f-algorithm-2f-PID

clean-user-2f-algorithm-2f-PID:
	-$(RM) ./user/algorithm/PID/pid.cyclo ./user/algorithm/PID/pid.d ./user/algorithm/PID/pid.o ./user/algorithm/PID/pid.su

.PHONY: clean-user-2f-algorithm-2f-PID

