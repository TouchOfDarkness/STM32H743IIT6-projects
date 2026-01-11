################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../TouchGFX/generated/images/src/BitmapDatabase.cpp \
../TouchGFX/generated/images/src/SVGDatabase.cpp \
../TouchGFX/generated/images/src/image_Border.cpp \
../TouchGFX/generated/images/src/image_MemoryGameLogo.cpp \
../TouchGFX/generated/images/src/image_Number1.cpp \
../TouchGFX/generated/images/src/image_Number2.cpp \
../TouchGFX/generated/images/src/image_Number3.cpp \
../TouchGFX/generated/images/src/image_PlayButton.cpp \
../TouchGFX/generated/images/src/image_Score.cpp \
../TouchGFX/generated/images/src/image_game-over-game.cpp 

OBJS += \
./TouchGFX/generated/images/src/BitmapDatabase.o \
./TouchGFX/generated/images/src/SVGDatabase.o \
./TouchGFX/generated/images/src/image_Border.o \
./TouchGFX/generated/images/src/image_MemoryGameLogo.o \
./TouchGFX/generated/images/src/image_Number1.o \
./TouchGFX/generated/images/src/image_Number2.o \
./TouchGFX/generated/images/src/image_Number3.o \
./TouchGFX/generated/images/src/image_PlayButton.o \
./TouchGFX/generated/images/src/image_Score.o \
./TouchGFX/generated/images/src/image_game-over-game.o 

CPP_DEPS += \
./TouchGFX/generated/images/src/BitmapDatabase.d \
./TouchGFX/generated/images/src/SVGDatabase.d \
./TouchGFX/generated/images/src/image_Border.d \
./TouchGFX/generated/images/src/image_MemoryGameLogo.d \
./TouchGFX/generated/images/src/image_Number1.d \
./TouchGFX/generated/images/src/image_Number2.d \
./TouchGFX/generated/images/src/image_Number3.d \
./TouchGFX/generated/images/src/image_PlayButton.d \
./TouchGFX/generated/images/src/image_Score.d \
./TouchGFX/generated/images/src/image_game-over-game.d 


# Each subdirectory must supply rules for building sources it contributes
TouchGFX/generated/images/src/%.o TouchGFX/generated/images/src/%.su TouchGFX/generated/images/src/%.cyclo: ../TouchGFX/generated/images/src/%.cpp TouchGFX/generated/images/src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../TouchGFX/App -I../TouchGFX/target/generated -I../TouchGFX/target -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/touchgfx/framework/include -I../TouchGFX/generated/fonts/include -I../TouchGFX/generated/gui_generated/include -I../TouchGFX/generated/images/include -I../TouchGFX/generated/texts/include -I../TouchGFX/generated/videos/include -I../TouchGFX/gui/include -I"D:/Data/2-Code/15-Stm32H743iit6/3-Code/5-Memory-Game/UserLibrary/Bsp" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -femit-class-debug-always -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-TouchGFX-2f-generated-2f-images-2f-src

clean-TouchGFX-2f-generated-2f-images-2f-src:
	-$(RM) ./TouchGFX/generated/images/src/BitmapDatabase.cyclo ./TouchGFX/generated/images/src/BitmapDatabase.d ./TouchGFX/generated/images/src/BitmapDatabase.o ./TouchGFX/generated/images/src/BitmapDatabase.su ./TouchGFX/generated/images/src/SVGDatabase.cyclo ./TouchGFX/generated/images/src/SVGDatabase.d ./TouchGFX/generated/images/src/SVGDatabase.o ./TouchGFX/generated/images/src/SVGDatabase.su ./TouchGFX/generated/images/src/image_Border.cyclo ./TouchGFX/generated/images/src/image_Border.d ./TouchGFX/generated/images/src/image_Border.o ./TouchGFX/generated/images/src/image_Border.su ./TouchGFX/generated/images/src/image_MemoryGameLogo.cyclo ./TouchGFX/generated/images/src/image_MemoryGameLogo.d ./TouchGFX/generated/images/src/image_MemoryGameLogo.o ./TouchGFX/generated/images/src/image_MemoryGameLogo.su ./TouchGFX/generated/images/src/image_Number1.cyclo ./TouchGFX/generated/images/src/image_Number1.d ./TouchGFX/generated/images/src/image_Number1.o ./TouchGFX/generated/images/src/image_Number1.su ./TouchGFX/generated/images/src/image_Number2.cyclo ./TouchGFX/generated/images/src/image_Number2.d ./TouchGFX/generated/images/src/image_Number2.o ./TouchGFX/generated/images/src/image_Number2.su ./TouchGFX/generated/images/src/image_Number3.cyclo ./TouchGFX/generated/images/src/image_Number3.d ./TouchGFX/generated/images/src/image_Number3.o ./TouchGFX/generated/images/src/image_Number3.su ./TouchGFX/generated/images/src/image_PlayButton.cyclo ./TouchGFX/generated/images/src/image_PlayButton.d ./TouchGFX/generated/images/src/image_PlayButton.o ./TouchGFX/generated/images/src/image_PlayButton.su ./TouchGFX/generated/images/src/image_Score.cyclo ./TouchGFX/generated/images/src/image_Score.d ./TouchGFX/generated/images/src/image_Score.o ./TouchGFX/generated/images/src/image_Score.su ./TouchGFX/generated/images/src/image_game-over-game.cyclo ./TouchGFX/generated/images/src/image_game-over-game.d ./TouchGFX/generated/images/src/image_game-over-game.o ./TouchGFX/generated/images/src/image_game-over-game.su

.PHONY: clean-TouchGFX-2f-generated-2f-images-2f-src

