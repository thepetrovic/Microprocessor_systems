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
blinkanje_diode.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
blinkanje_diode.list \

OBJCOPY_HEX += \
blinkanje_diode.hex \

OBJCOPY_BIN += \
blinkanje_diode.bin \


# All Target
all: blinkanje_diode.elf secondary-outputs

# Tool invocations
blinkanje_diode.elf: $(OBJS) $(USER_OBJS) D:\FAKS\4\ godina\Mikroprocesori\STM32CubeMX\ zadaci\blinkanje_diode\STM32F103C6TX_FLASH.ld
	arm-none-eabi-gcc -o "blinkanje_diode.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"D:\FAKS\4 godina\Mikroprocesori\STM32CubeMX zadaci\blinkanje_diode\STM32F103C6TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="blinkanje_diode.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

blinkanje_diode.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "blinkanje_diode.list"
	@echo 'Finished building: $@'
	@echo ' '

blinkanje_diode.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "blinkanje_diode.hex"
	@echo 'Finished building: $@'
	@echo ' '

blinkanje_diode.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "blinkanje_diode.bin"
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
