################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Src/usbh_cdc.c \
/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_core.c \
/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ctlreq.c \
/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ioreq.c \
/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_pipes.c 

OBJS += \
./Middlewares/USB_Host_Library/usbh_cdc.o \
./Middlewares/USB_Host_Library/usbh_core.o \
./Middlewares/USB_Host_Library/usbh_ctlreq.o \
./Middlewares/USB_Host_Library/usbh_ioreq.o \
./Middlewares/USB_Host_Library/usbh_pipes.o 

C_DEPS += \
./Middlewares/USB_Host_Library/usbh_cdc.d \
./Middlewares/USB_Host_Library/usbh_core.d \
./Middlewares/USB_Host_Library/usbh_ctlreq.d \
./Middlewares/USB_Host_Library/usbh_ioreq.d \
./Middlewares/USB_Host_Library/usbh_pipes.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/USB_Host_Library/usbh_cdc.o: /home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Src/usbh_cdc.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F407xx -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Inc" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/ST/STM32_USB_Host_Library/Core/Inc" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Middlewares/USB_Host_Library/usbh_core.o: /home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_core.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F407xx -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Inc" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/ST/STM32_USB_Host_Library/Core/Inc" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Middlewares/USB_Host_Library/usbh_ctlreq.o: /home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ctlreq.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F407xx -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Inc" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/ST/STM32_USB_Host_Library/Core/Inc" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Middlewares/USB_Host_Library/usbh_ioreq.o: /home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ioreq.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F407xx -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Inc" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/ST/STM32_USB_Host_Library/Core/Inc" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Middlewares/USB_Host_Library/usbh_pipes.o: /home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_pipes.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F407xx -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Inc" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/ST/STM32_USB_Host_Library/Core/Inc" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/ale/.Ac6/SystemWorkbench/workspace/PFSDTR-F4/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


