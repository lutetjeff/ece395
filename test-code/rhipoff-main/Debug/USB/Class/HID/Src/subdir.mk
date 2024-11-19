################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../USB/Class/HID/Src/usbd_hid.c 

OBJS += \
./USB/Class/HID/Src/usbd_hid.o 

C_DEPS += \
./USB/Class/HID/Src/usbd_hid.d 


# Each subdirectory must supply rules for building sources it contributes
USB/Class/HID/Src/%.o USB/Class/HID/Src/%.su USB/Class/HID/Src/%.cyclo: ../USB/Class/HID/Src/%.c USB/Class/HID/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H533xx -c -I../Core/Inc -I../Drivers/STM32H5xx_HAL_Driver/Inc -I../Drivers/STM32H5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H5xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/sapph/STM32CubeIDE/workspace_1.16.0/rhipoff-main/USB/Class/CDC/Inc" -I"C:/Users/sapph/STM32CubeIDE/workspace_1.16.0/rhipoff-main/USB/Class/CompositeBuilder/Inc" -I"C:/Users/sapph/STM32CubeIDE/workspace_1.16.0/rhipoff-main/USB/Class/HID/Inc" -I"C:/Users/sapph/STM32CubeIDE/workspace_1.16.0/rhipoff-main/USB/Core/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-USB-2f-Class-2f-HID-2f-Src

clean-USB-2f-Class-2f-HID-2f-Src:
	-$(RM) ./USB/Class/HID/Src/usbd_hid.cyclo ./USB/Class/HID/Src/usbd_hid.d ./USB/Class/HID/Src/usbd_hid.o ./USB/Class/HID/Src/usbd_hid.su

.PHONY: clean-USB-2f-Class-2f-HID-2f-Src

