################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
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
naizmenicno_ukljucivanje_led_diode_CubeIDE.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
naizmenicno_ukljucivanje_led_diode_CubeIDE.list \

OBJCOPY_HEX += \
naizmenicno_ukljucivanje_led_diode_CubeIDE.hex \

OBJCOPY_BIN += \
naizmenicno_ukljucivanje_led_diode_CubeIDE.bin \


# All Target
all: naizmenicno_ukljucivanje_led_diode_CubeIDE.elf secondary-outputs

# Tool invocations
naizmenicno_ukljucivanje_led_diode_CubeIDE.elf: $(OBJS) $(USER_OBJS) D:\FAKS\4\ godina\Mikroprocesori\STM32CubeMX\ zadaci\naizmenicno_ukljucivanje_led_diode_CubeIDE\STM32F103C6TX_FLASH.ld
	arm-none-eabi-gcc -o "naizmenicno_ukljucivanje_led_diode_CubeIDE.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"D:\FAKS\4 godina\Mikroprocesori\STM32CubeMX zadaci\naizmenicno_ukljucivanje_led_diode_CubeIDE\STM32F103C6TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="naizmenicno_ukljucivanje_led_diode_CubeIDE.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

naizmenicno_ukljucivanje_led_diode_CubeIDE.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "naizmenicno_ukljucivanje_led_diode_CubeIDE.list"
	@echo 'Finished building: $@'
	@echo ' '

naizmenicno_ukljucivanje_led_diode_CubeIDE.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "naizmenicno_ukljucivanje_led_diode_CubeIDE.hex"
	@echo 'Finished building: $@'
	@echo ' '

naizmenicno_ukljucivanje_led_diode_CubeIDE.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "naizmenicno_ukljucivanje_led_diode_CubeIDE.bin"
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
