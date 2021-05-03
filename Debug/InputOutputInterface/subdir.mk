################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../InputOutputInterface/InputOutputInterface.c 

OBJS += \
./InputOutputInterface/InputOutputInterface.o 

C_DEPS += \
./InputOutputInterface/InputOutputInterface.d 


# Each subdirectory must supply rules for building sources it contributes
InputOutputInterface/InputOutputInterface.o: ../InputOutputInterface/InputOutputInterface.c InputOutputInterface/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I"C:/Users/nesat/STM32CubeIDE/workspace_1.6.0/testF1/taskManagerInterface" -I"C:/Users/nesat/STM32CubeIDE/workspace_1.6.0/testF1/motorDriverInterface" -I"C:/Users/nesat/STM32CubeIDE/workspace_1.6.0/testF1/InputOutputInterface" -I"C:/Users/nesat/STM32CubeIDE/workspace_1.6.0/testF1/ComputerInterface" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"InputOutputInterface/InputOutputInterface.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

