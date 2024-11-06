-include ../makefile.init

IAR_PATH = "C:\Program Files (x86)\IAR Systems\Embedded Workbench 8.3\arm"
IAR_BIN_PATH = $(IAR_PATH)\bin
SRC_DIR = Code
LIB_PATH = C:/dev/libs
BRK_CORE_PATH = $(LIB_PATH)/breaker_core
CONFIG_FILE_PATH := Code\config\SB\2_pole\GFCI
UTILS_PATH = $(LIB_PATH)/utilities

# Define the compiler and other tools with full paths
CC := "C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/13.3 rel1/bin/arm-none-eabi-gcc.exe"
#AS := "C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/13.3 rel1/bin/arm-none-eabi-as.exe"
#CP := "C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/13.3 rel1/bin/arm-none-eabi-objcopy.exe"
SZ := "C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/13.3 rel1/bin/arm-none-eabi-size.exe"
RM := del /Q

#CC = $(IAR_BIN_PATH)\iccarm.exe
AS = $(IAR_BIN_PATH)\iasmarm.exe
LN = $(IAR_BIN_PATH)\ilinkarm.exe

CP = $(IAR_BIN_PATH)\ielftool.exe

HEX = $(CP) --ihex --verbose
SREC = $(CP) --srec --verbose

# Project name
TARGET = GN_Project

# Build Directory
BUILD_DIR = Build

# optimization
OPT = -Ol

# cpu
CPU = Cortex-M0+

# cpu default function mode
CPU_MODE = --thumb

# fpu
FPU = --fpu=None

# endianess
ENDIAN = --endian=little

# mcu
MCU = --cpu=$(CPU) $(CPU_MODE) $(FPU) $(ENDIAN)

#Enable IAR language extensions
LANG_EXT_ENB = -e

# Create the build directory if it doesn't exist
create_build_dir:
	@if not exist "$(BUILD_DIR)" mkdir "$(BUILD_DIR)"

C_SOURCES =  \
Code/main.c \
../dev/libs/ST/stm32/stm32g0xx/CMSIS/Device/ST/STM32G0xx/Source/Templates/system_stm32g0xx.c \
../dev/libs/ST/stm32/stm32g0xx/STM32G0xx_HAL_Driver/Src/stm32g0xx_ll_tim.c \
../dev/libs/ST/stm32/stm32g0xx/STM32G0xx_HAL_Driver/Src/stm32g0xx_ll_rcc.c \
../dev/libs/ST/stm32/stm32g0xx/STM32G0xx_HAL_Driver/Src/stm32g0xx_ll_utils.c \
../dev/libs/ST/stm32/stm32g0xx/STM32G0xx_HAL_Driver/Src/stm32g0xx_ll_exti.c \
../dev/libs/ST/stm32/stm32g0xx/STM32G0xx_HAL_Driver/Src/stm32g0xx_ll_gpio.c \
../dev/libs/ST/stm32/stm32g0xx/STM32G0xx_HAL_Driver/Src/stm32g0xx_ll_spi.c

# ASM sources
ASM_SOURCES =  \
../dev/libs/ST/stm32/stm32g0xx/CMSIS/Device/ST/STM32G0xx/Source/Templates/arm/startup_stm32g071xx.s

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# Object files
# Object files (placed in BUILD_DIR)
C_OBJS = $(patsubst %.c,$(BUILD_DIR)/%.o,$(C_LIST))
ASM_OBJS = $(patsubst %.s,$(BUILD_DIR)/%.o,$(ASM_LIST))
OBJS = $(C_OBJS) $(ASM_OBJS)

# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_FULL_LL_DRIVER \
-DHSE_VALUE=8000000 \
-DHSE_STARTUP_TIMEOUT=100 \
-DLSE_STARTUP_TIMEOUT=5000 \
-DLSE_VALUE=32768 \
-DEXTERNAL_CLOCK_VALUE=12288000 \
-DHSI_VALUE=16000000 \
-DLSI_VALUE=32000 \
-DVDD_VALUE=3300 \
-DPREFETCH_ENABLE=0 \
-DINSTRUCTION_CACHE_ENABLE=1 \
-DDATA_CACHE_ENABLE=1 \
-DSTM32G071xx

# AS includes
AS_INCLUDES = 

#dlib
DLIB = --dlib_config $(IAR_PATH)/inc/c/DLib_Config_Full.h

# Include paths
C_INCLUDES := \
-I"C:/GN_Project/Code" \
-I"C:/GN_Project/Code/protection/main" \
-I"C:/GN_Project/Code/protection/sys" \
-I"C:/dev/libs" \
-I"C:/dev/libs/ST/stm32/stm32g0xx/STM32G0xx_HAL_Driver/Inc" \
-I"C:/dev/libs/ST/stm32/stm32g0xx/CMSIS/Device/ST/STM32G0xx/Include" \
-I"C:/dev/libs/ST/stm32/stm32g0xx/CMSIS/Include" \
-I"C:/dev/libs/utilities/utilities-0.0.2/inc"

# Compile flags
#CFLAGS = -mcpu=cortex-m0plus -mthumb -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP
#CFLAGS += -DUSE_FULL_LL_DRIVER -DSTM32G0xx -DUSE_HAL_DRIVER -DSTM32G071xx
#CFLAGS += $(C_INCLUDES)

DEBUG_CFLAGS = $(MCU) $(C_DEFS) $(DLIB) $(C_INCLUDES) $(OPT) $(LANG_EXT_ENB) --debug --f debug.config -DSTATIC="static"

ASFLAGS = -s+ -w+ -r --cpu $(CPU) $(AS_DEFS) $(AS_INCLUDES)

# Linker script
LDSCRIPT = C:\GN_Project\STM32G071CBU6_FLASH.ld

# Linker flags
LDFLAGS = -mcpu=cortex-m0plus -mthumb -T$(LDSCRIPT) -Wl,-Map=$(TARGET).map,--cref -Wl,--gc-sections

# Libraries
LIBS = ---lc -lm ---lnosys 

# Compile rule
$(BUILD_DIR)/%.o: %.c Makefile FORCE
	@echo 'Building file: $<'
	@echo 'Invoking: Compiler'
	$(CC) $(CFLAGS) -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo '	'

#$(BUILD_DIR)/%.o: %.c
#	@echo 'Building file: $<'
#	@echo 'Invoking: Compiler'
#	$(CC) $(CFLAGS) $(C_INCLUDES) -c -o "$@" "$<"
#	@echo 'Finished building: $<'
#	@echo '	'

$(BUILD_DIR)/%.o: %.s Makefile FORCE
	@echo 'Building file: $<'
	@echo 'Invoking: Assembler'
	$(AS) $< -o $@ $(ASFLAGS) 
	@echo 'Finished building: $<'
	@echo '	'

#$(BUILD_DIR)/%.o: %.s
#	@echo 'Building file: $<'
#	@echo 'Invoking: Cross ARM GNU Assembler'
#	$(CC) -mcpu=cortex-m0plus -mthumb -g -c -x assembler-with-cpp -o "$@" "$<"
#	@echo 'Finished building: $<'
#	@echo '	'

# Link rule
$(BUILD_DIR)/$(TARGET).elf: $(C_OBJS) $(ASM_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Linker'
	$(CC) $(LDFLAGS) -o "$@" $^ $(OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(SZ) "$@"

# All target
all: create_build_dir $(OBJS) $(BUILD_DIR)/$(TARGET).elf
	@echo 'Finished building all targets'

# Other targets
clean:
	@echo 'Cleaning up: $(OBJS) $(TARGET).elf $(TARGET).map'
	del /Q Build
	del /Q $(TARGET).map
	@echo 'Cleaned up object files and binaries.'

# Generate binary and hex files
$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(CP) -O binary $< $@
	
# Load hardware configuration
load_hardware_config:
	-@erase /Q $(SRC_DIR)\\sys\\*.pin_config
	-@erase /Q $(SRC_DIR)\\sys\\*firmware.config
	-@erase /Q $(SRC_DIR)\\sys\\*hardware_init.c
	copy $(TARGET_PIN_CONFIG_FILE) $(SRC_DIR)\\sys\\hardware.pin_config
	copy $(TARGET_FIRMWARE_CONFIG_FILE) $(SRC_DIR)\\sys\\firmware.config
	copy $(TARGET_HARDWARE_INIT_FILE) $(SRC_DIR)\\sys\\hardware_init.c
	@echo TARGET_VERSION_FILE = $(TARGET_VERSION_FILE)
	@echo TARGET_VERSION = $(TARGET_VERSION)
	@echo RELEASE_TARGET_VERSION = $(RELEASE_TARGET_VERSION)
	@echo FIRMWARE_BUILD_VERSION = $(FIRMWARE_BUILD_VERSION)
	@echo CFLAGS = $(CFLAGS)
	@echo using breaker-core : $(BRK_CORE_VERSION)
	@echo using utilities : $(UTILS_VERSION)

# Debug target
debug: TARGET_PIN_CONFIG_FILE := $(shell dir /b/s $(CONFIG_FILE_PATH)\\*.pin_config)
debug: TARGET_FIRMWARE_CONFIG_FILE := $(shell dir /b/s $(CONFIG_FILE_PATH)\\*.config)
debug: TARGET_HARDWARE_INIT_FILE := $(shell dir /b/s $(CONFIG_FILE_PATH)\\*hardware_init.c)
debug: TARGET_VERSION_FILE := $(CONFIG_FILE_PATH)\\version.txt
debug: TARGET_VERSION := $(shell type $(TARGET_VERSION_FILE))
debug: RELEASE_TARGET_VERSION := $(word 1, $(TARGET_VERSION))
debug: FIRMWARE_BUILD_VERSION := $(word 2, $(TARGET_VERSION))
debug: BRK_CORE_VERSION := $(word 3, $(TARGET_VERSION))
debug: UTILS_VERSION := $(word 4, $(TARGET_VERSION))
debug: LIBS := $(UTILS_PATH)/$(UTILS_VERSION)/lib/$(UTILS_VERSION).a $(BRK_CORE_PATH)/$(BRK_CORE_VERSION)/lib/$(BRK_CORE_VERSION).a
debug: C_INCLUDES += -I$(BRK_CORE_PATH)/$(BRK_CORE_VERSION)/inc
debug: C_INCLUDES += -I$(UTILS_PATH)/$(UTILS_VERSION)/inc
debug: CFLAGS = $(DEBUG_CFLAGS)
debug: load_hardware_config all

# Phony targets
.PHONY: all clean dependents debug load_h

-include ../makefile.targets