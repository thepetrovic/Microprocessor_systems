################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Projekat_Troosno_merenje_ubrzanja.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Projekat_Troosno_merenje_ubrzanja.list \

OBJCOPY_HEX += \
Projekat_Troosno_merenje_ubrzanja.hex \

OBJCOPY_BIN += \
Projekat_Troosno_merenje_ubrzanja.bin \


# All Target
all: Projekat_Troosno_merenje_ubrzanja.elf secondary-outputs

# Tool invocations
Projekat_Troosno_merenje_ubrzanja.elf: $(OBJS) $(USER_OBJS) D:\FAKS\4\ godina\Mikroprocesori\Vezbe\Projekat_Troosno_merenje_ubrzanja\STM32F103C6TX_FLASH.ld
	arm-none-eabi-gcc -o "Projekat_Troosno_merenje_ubrzanja.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"D:\FAKS\4 godina\Mikroprocesori\Vezbe\Projekat_Troosno_merenje_ubrzanja\STM32F103C6TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Projekat_Troosno_merenje_ubrzanja.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Projekat_Troosno_merenje_ubrzanja.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Projekat_Troosno_merenje_ubrzanja.list"
	@echo 'Finished building: $@'
	@echo ' '

Projekat_Troosno_merenje_ubrzanja.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "Projekat_Troosno_merenje_ubrzanja.hex"
	@echo 'Finished building: $@'
	@echo ' '

Projekat_Troosno_merenje_ubrzanja.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Projekat_Troosno_merenje_ubrzanja.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
