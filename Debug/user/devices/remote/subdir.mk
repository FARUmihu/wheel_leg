################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../user/devices/remote/remote_esp32.c 

OBJS += \
./user/devices/remote/remote_esp32.o 

C_DEPS += \
./user/devices/remote/remote_esp32.d 


# Each subdirectory must supply rules for building sources it contributes
user/devices/remote/%.o user/devices/remote/%.su user/devices/remote/%.cyclo: ../user/devices/remote/%.c user/devices/remote/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I"C:/Users/FMI/Documents/GitHub/wheel_leg/user/app" -I"C:/Users/FMI/Documents/GitHub/wheel_leg/user/devices/remote" -I"C:/Users/FMI/Documents/GitHub/wheel_leg/user/algorithm/leg_kinematics" -I"C:/Users/FMI/Documents/GitHub/wheel_leg/user/bsp/bsp_can" -I"C:/Users/FMI/Documents/GitHub/wheel_leg/Drivers/CMSIS/DSP/Include" -I"C:/Users/FMI/Documents/GitHub/wheel_leg/user/devices/dm_motor" -I"C:/Users/FMI/Documents/GitHub/wheel_leg/user/devices/feetech" -I"C:/Users/FMI/Documents/GitHub/wheel_leg/user/devices/imu" -I"C:/Users/FMI/Documents/GitHub/wheel_leg/user/devices/remote" -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-user-2f-devices-2f-remote

clean-user-2f-devices-2f-remote:
	-$(RM) ./user/devices/remote/remote_esp32.cyclo ./user/devices/remote/remote_esp32.d ./user/devices/remote/remote_esp32.o ./user/devices/remote/remote_esp32.su

.PHONY: clean-user-2f-devices-2f-remote

