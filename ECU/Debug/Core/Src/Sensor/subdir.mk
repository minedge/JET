################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Sensor/as5147.c 

OBJS += \
./Core/Src/Sensor/as5147.o 

C_DEPS += \
./Core/Src/Sensor/as5147.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Sensor/%.o Core/Src/Sensor/%.su: ../Core/Src/Sensor/%.c Core/Src/Sensor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Sensor

clean-Core-2f-Src-2f-Sensor:
	-$(RM) ./Core/Src/Sensor/as5147.d ./Core/Src/Sensor/as5147.o ./Core/Src/Sensor/as5147.su

.PHONY: clean-Core-2f-Src-2f-Sensor

