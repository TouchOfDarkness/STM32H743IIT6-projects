################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../TouchGFX/gui/src/gameoverscreen_screen/GameOverScreenPresenter.cpp \
../TouchGFX/gui/src/gameoverscreen_screen/GameOverScreenView.cpp 

OBJS += \
./TouchGFX/gui/src/gameoverscreen_screen/GameOverScreenPresenter.o \
./TouchGFX/gui/src/gameoverscreen_screen/GameOverScreenView.o 

CPP_DEPS += \
./TouchGFX/gui/src/gameoverscreen_screen/GameOverScreenPresenter.d \
./TouchGFX/gui/src/gameoverscreen_screen/GameOverScreenView.d 


# Each subdirectory must supply rules for building sources it contributes
TouchGFX/gui/src/gameoverscreen_screen/%.o TouchGFX/gui/src/gameoverscreen_screen/%.su TouchGFX/gui/src/gameoverscreen_screen/%.cyclo: ../TouchGFX/gui/src/gameoverscreen_screen/%.cpp TouchGFX/gui/src/gameoverscreen_screen/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../TouchGFX/App -I../TouchGFX/target/generated -I../TouchGFX/target -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/touchgfx/framework/include -I../TouchGFX/generated/fonts/include -I../TouchGFX/generated/gui_generated/include -I../TouchGFX/generated/images/include -I../TouchGFX/generated/texts/include -I../TouchGFX/generated/videos/include -I../TouchGFX/gui/include -I"D:/Data/2-Code/15-Stm32H743iit6/3-Code/5-Memory-Game/UserLibrary/Bsp" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -femit-class-debug-always -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-TouchGFX-2f-gui-2f-src-2f-gameoverscreen_screen

clean-TouchGFX-2f-gui-2f-src-2f-gameoverscreen_screen:
	-$(RM) ./TouchGFX/gui/src/gameoverscreen_screen/GameOverScreenPresenter.cyclo ./TouchGFX/gui/src/gameoverscreen_screen/GameOverScreenPresenter.d ./TouchGFX/gui/src/gameoverscreen_screen/GameOverScreenPresenter.o ./TouchGFX/gui/src/gameoverscreen_screen/GameOverScreenPresenter.su ./TouchGFX/gui/src/gameoverscreen_screen/GameOverScreenView.cyclo ./TouchGFX/gui/src/gameoverscreen_screen/GameOverScreenView.d ./TouchGFX/gui/src/gameoverscreen_screen/GameOverScreenView.o ./TouchGFX/gui/src/gameoverscreen_screen/GameOverScreenView.su

.PHONY: clean-TouchGFX-2f-gui-2f-src-2f-gameoverscreen_screen

