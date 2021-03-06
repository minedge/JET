################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Serial/spi.c \
../Core/Src/Serial/uart.c 

OBJS += \
./Core/Src/Serial/spi.o \
./Core/Src/Serial/uart.o 

C_DEPS += \
./Core/Src/Serial/spi.d \
./Core/Src/Serial/uart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Serial/%.o Core/Src/Serial/%.su: ../Core/Src/Serial/%.c Core/Src/Serial/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Serial

clean-Core-2f-Src-2f-Serial:
	-$(RM) ./Core/Src/Serial/spi.d ./Core/Src/Serial/spi.o ./Core/Src/Serial/spi.su ./Core/Src/Serial/uart.d ./Core/Src/Serial/uart.o ./Core/Src/Serial/uart.su

.PHONY: clean-Core-2f-Src-2f-Serial

