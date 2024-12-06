################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32h533cetx.s 

OBJS += \
./Core/Startup/startup_stm32h533cetx.o 

S_DEPS += \
./Core/Startup/startup_stm32h533cetx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m33 -g3 -DDEBUG -c -I"C:/Users/sapph/STM32CubeIDE/workspace_1.16.0/rhipoff-main/USB/Class/CDC/Inc" -I"C:/Users/sapph/STM32CubeIDE/workspace_1.16.0/rhipoff-main/USB/Class/CompositeBuilder/Inc" -I"C:/Users/sapph/STM32CubeIDE/workspace_1.16.0/rhipoff-main/USB/Class/HID/Inc" -I"C:/Users/sapph/STM32CubeIDE/workspace_1.16.0/rhipoff-main/USB/Core/Inc" -I"C:/Users/sapph/STM32CubeIDE/workspace_1.16.0/rhipoff-main/USB/Class/CustomHID" -I"C:/Users/sapph/STM32CubeIDE/workspace_1.16.0/rhipoff-main/USB/Class/CustomHID/Src" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32h533cetx.d ./Core/Startup/startup_stm32h533cetx.o

.PHONY: clean-Core-2f-Startup

