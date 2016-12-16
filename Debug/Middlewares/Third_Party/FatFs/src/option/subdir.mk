################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FatFs/src/option/syscall.c 

OBJS += \
./Middlewares/Third_Party/FatFs/src/option/syscall.o 

C_DEPS += \
./Middlewares/Third_Party/FatFs/src/option/syscall.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FatFs/src/option/%.o: ../Middlewares/Third_Party/FatFs/src/option/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F405xx -I"/home/thinkpot/workspace/dr_v3/Inc" -I"/home/thinkpot/workspace/dr_v3/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/home/thinkpot/workspace/dr_v3/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/home/thinkpot/workspace/dr_v3/Drivers/CMSIS/Include" -I"/home/thinkpot/workspace/dr_v3/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/thinkpot/workspace/dr_v3/Inc" -I"/home/thinkpot/workspace/dr_v3/Middlewares/Third_Party/FatFs/src/drivers" -I"/home/thinkpot/workspace/dr_v3/Middlewares/Third_Party/FatFs/src"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


