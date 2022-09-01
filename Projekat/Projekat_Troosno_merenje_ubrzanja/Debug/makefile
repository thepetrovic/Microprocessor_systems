################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
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


BUILD_ARTIFACT_NAME := Projekat_Troosno_merenje_ubrzanja
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Projekat_Troosno_merenje_ubrzanja.elf \

MAP_FILES += \
Projekat_Troosno_merenje_ubrzanja.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Projekat_Troosno_merenje_ubrzanja.list \

OBJCOPY_HEX += \
Projekat_Troosno_merenje_ubrzanja.hex \


# All Target
all: main-build

# Main-build Target
main-build: Projekat_Troosno_merenje_ubrzanja.elf secondary-outputs

# Tool invocations
Projekat_Troosno_merenje_ubrzanja.elf Projekat_Troosno_merenje_ubrzanja.map: $(OBJS) $(USER_OBJS) C:\Users\Marijana\ Petrovic\Downloads\Microprocessor_systems-main\Microprocessor_systems-main\Projekat\Projekat_Troosno_merenje_ubrzanja\STM32F103C6TX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Projekat_Troosno_merenje_ubrzanja.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\Marijana Petrovic\Downloads\Microprocessor_systems-main\Microprocessor_systems-main\Projekat\Projekat_Troosno_merenje_ubrzanja\STM32F103C6TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Projekat_Troosno_merenje_ubrzanja.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Projekat_Troosno_merenje_ubrzanja.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Projekat_Troosno_merenje_ubrzanja.list"
	@echo 'Finished building: $@'
	@echo ' '

Projekat_Troosno_merenje_ubrzanja.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "Projekat_Troosno_merenje_ubrzanja.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Projekat_Troosno_merenje_ubrzanja.elf Projekat_Troosno_merenje_ubrzanja.hex Projekat_Troosno_merenje_ubrzanja.list Projekat_Troosno_merenje_ubrzanja.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
