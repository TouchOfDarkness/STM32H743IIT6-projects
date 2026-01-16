################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F7xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := Dzialajace MPU z kalmanem i wysylanie
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Dzialajace\ MPU\ z\ kalmanem\ i\ wysylanie.elf \

MAP_FILES += \
Dzialajace\ MPU\ z\ kalmanem\ i\ wysylanie.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Dzialajace\ MPU\ z\ kalmanem\ i\ wysylanie.list \


# All Target
all: main-build

# Main-build Target
main-build: Dzialajace\ MPU\ z\ kalmanem\ i\ wysylanie.elf secondary-outputs

# Tool invocations
Dzialajace\ MPU\ z\ kalmanem\ i\ wysylanie.elf Dzialajace\ MPU\ z\ kalmanem\ i\ wysylanie.map: $(OBJS) $(USER_OBJS) C:\Users\kamil\STM32CubeIDE\workspace_1.17.0\Dzialajace\ MPU\ z\ kalmanem\ i\ wysylanie\STM32F746ZGTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Dzialajace MPU z kalmanem i wysylanie.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"C:\Users\kamil\STM32CubeIDE\workspace_1.17.0\Dzialajace MPU z kalmanem i wysylanie\STM32F746ZGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Dzialajace MPU z kalmanem i wysylanie.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Dzialajace\ MPU\ z\ kalmanem\ i\ wysylanie.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Dzialajace MPU z kalmanem i wysylanie.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Dzialajace\ MPU\ z\ kalmanem\ i\ wysylanie.elf Dzialajace\ MPU\ z\ kalmanem\ i\ wysylanie.list Dzialajace\ MPU\ z\ kalmanem\ i\ wysylanie.map Dzialajace\\ MPU\\ z\\ kalmanem\\ i\\ wysylanie.elf default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
