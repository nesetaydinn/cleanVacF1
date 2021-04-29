################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../motorDriverInterface/motorDriverInterface.c 

OBJS += \
./motorDriverInterface/motorDriverInterface.o 

C_DEPS += \
./motorDriverInterface/motorDriverInterface.d 


# Each subdirectory must supply rules for building sources it contributes
motorDriverInterface/motorDriverInterface.o: ../motorDriverInterface/motorDriverInterface.c motorDriverInterface/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I"C:/Users/90551/Downloads/cleanVacF1-main/taskManagerInterface" -I"C:/Users/90551/Downloads/cleanVacF1-main/motorDriverInterface" -I"C:/Users/90551/Downloads/cleanVacF1-main/InputOutputInterface" -I"C:/Users/90551/Downloads/cleanVacF1-main/ComputerInterface" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"motorDriverInterface/motorDriverInterface.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

