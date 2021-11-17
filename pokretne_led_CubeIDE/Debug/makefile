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
pokretne_led_CubeIDE.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
pokretne_led_CubeIDE.list \

OBJCOPY_HEX += \
pokretne_led_CubeIDE.hex \

OBJCOPY_BIN += \
pokretne_led_CubeIDE.bin \


# All Target
all: pokretne_led_CubeIDE.elf secondary-outputs

# Tool invocations
pokretne_led_CubeIDE.elf: $(OBJS) $(USER_OBJS) D:\FAKS\4\ godina\Mikroprocesori\STM32CubeIDE\ zadaci\pokretne_led_CubeIDE\STM32F103C6TX_FLASH.ld
	arm-none-eabi-gcc -o "pokretne_led_CubeIDE.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"D:\FAKS\4 godina\Mikroprocesori\STM32CubeIDE zadaci\pokretne_led_CubeIDE\STM32F103C6TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="pokretne_led_CubeIDE.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

pokretne_led_CubeIDE.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "pokretne_led_CubeIDE.list"
	@echo 'Finished building: $@'
	@echo ' '

pokretne_led_CubeIDE.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "pokretne_led_CubeIDE.hex"
	@echo 'Finished building: $@'
	@echo ' '

pokretne_led_CubeIDE.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "pokretne_led_CubeIDE.bin"
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
