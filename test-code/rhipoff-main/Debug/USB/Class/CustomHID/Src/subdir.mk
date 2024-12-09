################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../USB/Class/CustomHID/Src/usbd_customhid.c \
../USB/Class/CustomHID/Src/usbd_customhid_if.c 

OBJS += \
./USB/Class/CustomHID/Src/usbd_customhid.o \
./USB/Class/CustomHID/Src/usbd_customhid_if.o 

C_DEPS += \
./USB/Class/CustomHID/Src/usbd_customhid.d \
./USB/Class/CustomHID/Src/usbd_customhid_if.d 


# Each subdirectory must supply rules for building sources it contributes
USB/Class/CustomHID/Src/%.o USB/Class/CustomHID/Src/%.su USB/Class/CustomHID/Src/%.cyclo: ../USB/Class/CustomHID/Src/%.c USB/Class/CustomHID/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H533xx -c -I../Core/Inc -I../Drivers/STM32H5xx_HAL_Driver/Inc -I../Drivers/STM32H5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H5xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/sapph/STM32CubeIDE/workspace_1.16.0/rhipoff-main/USB/Class/CDC/Inc" -I"C:/Users/sapph/STM32CubeIDE/workspace_1.16.0/rhipoff-main/USB/Class/CompositeBuilder/Inc" -I"C:/Users/sapph/STM32CubeIDE/workspace_1.16.0/rhipoff-main/USB/Core/Inc" -I"C:/Users/sapph/STM32CubeIDE/workspace_1.16.0/rhipoff-main/USB/Class/CustomHID/Inc" -I"C:/Users/sapph/STM32CubeIDE/workspace_1.16.0/rhipoff-main/USB" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-USB-2f-Class-2f-CustomHID-2f-Src

clean-USB-2f-Class-2f-CustomHID-2f-Src:
	-$(RM) ./USB/Class/CustomHID/Src/usbd_customhid.cyclo ./USB/Class/CustomHID/Src/usbd_customhid.d ./USB/Class/CustomHID/Src/usbd_customhid.o ./USB/Class/CustomHID/Src/usbd_customhid.su ./USB/Class/CustomHID/Src/usbd_customhid_if.cyclo ./USB/Class/CustomHID/Src/usbd_customhid_if.d ./USB/Class/CustomHID/Src/usbd_customhid_if.o ./USB/Class/CustomHID/Src/usbd_customhid_if.su

.PHONY: clean-USB-2f-Class-2f-CustomHID-2f-Src

