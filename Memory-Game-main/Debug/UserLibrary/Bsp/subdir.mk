################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../UserLibrary/Bsp/Bsp_Gt9xx.c \
../UserLibrary/Bsp/Bsp_Lcd.c \
../UserLibrary/Bsp/Bsp_W25q64.c \
../UserLibrary/Bsp/Image.c \
../UserLibrary/Bsp/w25q_mem.c 

C_DEPS += \
./UserLibrary/Bsp/Bsp_Gt9xx.d \
./UserLibrary/Bsp/Bsp_Lcd.d \
./UserLibrary/Bsp/Bsp_W25q64.d \
./UserLibrary/Bsp/Image.d \
./UserLibrary/Bsp/w25q_mem.d 

OBJS += \
./UserLibrary/Bsp/Bsp_Gt9xx.o \
./UserLibrary/Bsp/Bsp_Lcd.o \
./UserLibrary/Bsp/Bsp_W25q64.o \
./UserLibrary/Bsp/Image.o \
./UserLibrary/Bsp/w25q_mem.o 


# Each subdirectory must supply rules for building sources it contributes
UserLibrary/Bsp/%.o UserLibrary/Bsp/%.su UserLibrary/Bsp/%.cyclo: ../UserLibrary/Bsp/%.c UserLibrary/Bsp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../TouchGFX/App -I../TouchGFX/target/generated -I../TouchGFX/target -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/touchgfx/framework/include -I../TouchGFX/generated/fonts/include -I../TouchGFX/generated/gui_generated/include -I../TouchGFX/generated/images/include -I../TouchGFX/generated/texts/include -I../TouchGFX/generated/videos/include -I../TouchGFX/gui/include -I"D:/Data/2-Code/15-Stm32H743iit6/3-Code/5-Memory-Game/UserLibrary/Bsp" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-UserLibrary-2f-Bsp

clean-UserLibrary-2f-Bsp:
	-$(RM) ./UserLibrary/Bsp/Bsp_Gt9xx.cyclo ./UserLibrary/Bsp/Bsp_Gt9xx.d ./UserLibrary/Bsp/Bsp_Gt9xx.o ./UserLibrary/Bsp/Bsp_Gt9xx.su ./UserLibrary/Bsp/Bsp_Lcd.cyclo ./UserLibrary/Bsp/Bsp_Lcd.d ./UserLibrary/Bsp/Bsp_Lcd.o ./UserLibrary/Bsp/Bsp_Lcd.su ./UserLibrary/Bsp/Bsp_W25q64.cyclo ./UserLibrary/Bsp/Bsp_W25q64.d ./UserLibrary/Bsp/Bsp_W25q64.o ./UserLibrary/Bsp/Bsp_W25q64.su ./UserLibrary/Bsp/Image.cyclo ./UserLibrary/Bsp/Image.d ./UserLibrary/Bsp/Image.o ./UserLibrary/Bsp/Image.su ./UserLibrary/Bsp/w25q_mem.cyclo ./UserLibrary/Bsp/w25q_mem.d ./UserLibrary/Bsp/w25q_mem.o ./UserLibrary/Bsp/w25q_mem.su

.PHONY: clean-UserLibrary-2f-Bsp

