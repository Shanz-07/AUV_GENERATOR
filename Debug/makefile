################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Drivers/BSP/STM32F4xx-Nucleo/subdir.mk
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


<<<<<<< HEAD
BUILD_ARTIFACT_NAME := auv1
=======
BUILD_ARTIFACT_NAME := auv
>>>>>>> a77da31a94716530a4892b19b5d0a66c9cd1352f
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
<<<<<<< HEAD
auv1.elf \

MAP_FILES += \
auv1.map \
=======
auv.elf \

MAP_FILES += \
auv.map \
>>>>>>> a77da31a94716530a4892b19b5d0a66c9cd1352f

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
<<<<<<< HEAD
auv1.list \
=======
auv.list \
>>>>>>> a77da31a94716530a4892b19b5d0a66c9cd1352f


# All Target
all: main-build

# Main-build Target
<<<<<<< HEAD
main-build: auv1.elf secondary-outputs

# Tool invocations
auv1.elf auv1.map: $(OBJS) $(USER_OBJS) E:\auv1\STM32F446RETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "auv1.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"E:\auv1\STM32F446RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="auv1.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
=======
main-build: auv.elf secondary-outputs

# Tool invocations
auv.elf auv.map: $(OBJS) $(USER_OBJS) C:\Users\nouma\OneDrive\Documents\auv\STM32F446RETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "auv.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\nouma\OneDrive\Documents\auv\STM32F446RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="auv.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
>>>>>>> a77da31a94716530a4892b19b5d0a66c9cd1352f
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

<<<<<<< HEAD
auv1.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "auv1.list"
=======
auv.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "auv.list"
>>>>>>> a77da31a94716530a4892b19b5d0a66c9cd1352f
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
<<<<<<< HEAD
	-$(RM) auv1.elf auv1.list auv1.map default.size.stdout
=======
	-$(RM) auv.elf auv.list auv.map default.size.stdout
>>>>>>> a77da31a94716530a4892b19b5d0a66c9cd1352f
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
