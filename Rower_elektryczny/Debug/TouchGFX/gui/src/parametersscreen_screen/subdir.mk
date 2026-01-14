################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../TouchGFX/gui/src/parametersscreen_screen/ParametersScreenPresenter.cpp \
../TouchGFX/gui/src/parametersscreen_screen/ParametersScreenView.cpp 

OBJS += \
./TouchGFX/gui/src/parametersscreen_screen/ParametersScreenPresenter.o \
./TouchGFX/gui/src/parametersscreen_screen/ParametersScreenView.o 

CPP_DEPS += \
./TouchGFX/gui/src/parametersscreen_screen/ParametersScreenPresenter.d \
./TouchGFX/gui/src/parametersscreen_screen/ParametersScreenView.d 


# Each subdirectory must supply rules for building sources it contributes
TouchGFX/gui/src/parametersscreen_screen/%.o TouchGFX/gui/src/parametersscreen_screen/%.su TouchGFX/gui/src/parametersscreen_screen/%.cyclo: ../TouchGFX/gui/src/parametersscreen_screen/%.cpp TouchGFX/gui/src/parametersscreen_screen/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../TouchGFX/App -I../TouchGFX/target/generated -I../TouchGFX/target -I../Middlewares/ST/touchgfx/framework/include -I../TouchGFX/generated/fonts/include -I../TouchGFX/generated/gui_generated/include -I../TouchGFX/generated/images/include -I../TouchGFX/generated/texts/include -I../TouchGFX/generated/videos/include -I../TouchGFX/gui/include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -femit-class-debug-always -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-TouchGFX-2f-gui-2f-src-2f-parametersscreen_screen

clean-TouchGFX-2f-gui-2f-src-2f-parametersscreen_screen:
	-$(RM) ./TouchGFX/gui/src/parametersscreen_screen/ParametersScreenPresenter.cyclo ./TouchGFX/gui/src/parametersscreen_screen/ParametersScreenPresenter.d ./TouchGFX/gui/src/parametersscreen_screen/ParametersScreenPresenter.o ./TouchGFX/gui/src/parametersscreen_screen/ParametersScreenPresenter.su ./TouchGFX/gui/src/parametersscreen_screen/ParametersScreenView.cyclo ./TouchGFX/gui/src/parametersscreen_screen/ParametersScreenView.d ./TouchGFX/gui/src/parametersscreen_screen/ParametersScreenView.o ./TouchGFX/gui/src/parametersscreen_screen/ParametersScreenView.su

.PHONY: clean-TouchGFX-2f-gui-2f-src-2f-parametersscreen_screen

