-include ../makefile.init

COMPONENTS = main
DRIVER_COMPONENT := utils exti gpio tim spi rcc

###########################################
### PATHS
###########################################

IAR_PATH = "C:\Program Files\IAR Systems\Embedded Workbench 9.2\arm"
IAR_BIN_PATH = $(IAR_PATH)\bin
SRC_DIR = src
LIB_PATH = C:/dev/libs
LIST_DIR = list

HAL_DRIVER_PATH = $(LIB_PATH)/ST/stm32/stm32g0xx/STM32G0xx_HAL_Driver
CMSIS_DRIVER_PATH = $(LIB_PATH)/ST/stm32/stm32g0xx/CMSIS
BRK_CORE_PATH = $(LIB_PATH)/breaker_core
CONFIG_FILE_PATH := $(SRC_DIR)\config\SB\2_pole\GFCI
UTILS_PATH = $(LIB_PATH)/utilities

# link script
LNSCRIPT = $(SRC_DIR)/sys/stm32g070xx_flash.icf

# Project name
TARGET = GN_Project_G070

# Build Directory
BUILD_DIR = build

SRC_PATH = C:/$(TARGET)/$(SRC_DIR)

###########################################
### TOOLS
###########################################

# Define the compiler and other tools with full paths 
RM := del /Q

CC = $(IAR_BIN_PATH)\iccarm.exe
AS = $(IAR_BIN_PATH)\iasmarm.exe
LN = $(IAR_BIN_PATH)\ilinkarm.exe
CP = $(IAR_BIN_PATH)\ielftool.exe

HEX = $(CP) --ihex --verbose
SREC = $(CP) --srec --verbose

###########################################
### FLAGS SETUP
###########################################
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

###########################################
### SOURCES
###########################################

# Create the build directory if it doesn't exist
create_build_dir:
	@if not exist "$(BUILD_DIR)" mkdir "$(BUILD_DIR)"

C_FILES = $(addsuffix .c, $(COMPONENTS))
C_FILES += $(addsuffix .c, $(addprefix stm32g0xx_ll_,$(DRIVER_COMPONENT)))
C_SRC_DIRS = $(addprefix $(SRC_DIR)/, $(COMPONENTS))
C_SRC_DIRS += $(HAL_DRIVER_PATH)/src

C_SOURCES = $(join $(addsuffix /, $(C_SRC_DIRS)), $(C_FILES))
C_SOURCES += $(CMSIS_DRIVER_PATH)/Device/ST/STM32G0xx/Source/Templates/system_stm32g0xx.c
C_SOURCES += $(HAL_DRIVER_PATH)/Src/stm32g0xx_hal.c
C_SOURCES += ../dev/libs/ST/stm32/stm32g0xx/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_cortex.c

# ASM sources
ASM_SOURCES = $(SRC_DIR)/sys/startup_stm32g070xx.s

###########################################
### OBJECTS
###########################################

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

###########################################
### DEFINITIONS
###########################################

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
-DSTM32G070xx

###########################################
### INCLUDES
###########################################

# AS includes
AS_INCLUDES = 

#dlib
DLIB = --dlib_config $(IAR_PATH)/inc/c/DLib_Config_Full.h

# Include paths
C_INCLUDES = -I$(SRC_PATH)
C_INCLUDES += -I$(SRC_PATH)/protection/main
C_INCLUDES += -I$(SRC_PATH)/protection/sys
C_INCLUDES += -I$(LIB_PATH)
C_INCLUDES += -I$(HAL_DRIVER_PATH)/Inc
C_INCLUDES += -I$(CMSIS_DRIVER_PATH)/Device/ST/STM32G0xx/Include
C_INCLUDES += -I$(CMSIS_DRIVER_PATH)/Include
C_INCLUDES += -I$(UTILS_PATH)/utilities-0.0.2/inc

###########################################
### FLAGS
###########################################

DEBUG_CFLAGS = $(MCU) $(C_DEFS) $(DLIB) $(C_INCLUDES) $(OPT) $(LANG_EXT_ENB) --debug --f debug.config -DSTATIC="static"

ASFLAGS = -s+ -w+ -r --cpu $(CPU) $(AS_DEFS) $(AS_INCLUDES)

###########################################
### LINK AND COMPILE
###########################################

# Linker flags
LNFLAGS = --redirect _//printf=_//printfFull --redirect _Scanf=_ScanfFull --no_out_extension --semihosting --entry __iar_program_start --vfe --text_out locale 
LNFLAGS += --place_holder __build_checksum,4,.checksum,16 --keep=__build_checksum 
LNFLAGS += --place_holder __version,8,.version,16 --keep=__version 

# Libraries
LIBS = -lc -lm -lnosys 

# Compile rule
$(BUILD_DIR)/%.o: %.c
	@echo **** building $< ****
	$(CC) $< $(CFLAGS) -o $@
	@echo ' '

$(BUILD_DIR)/%.i: %.c
	@echo **** building $< ****
	$(CC) $< $(CFLAGS) --preprocess=n $(BUILD_DIR) -o $@
	@echo ' '
	
$(BUILD_DIR)/%.o: %.s
	@echo **** building $< ****
	$(AS) $< -o $@ $(ASFLAGS) 
	@echo ' '

# Link rule
$(BUILD_DIR)/$(TARGET).elf: $(BUILD_DIR) $(LIST_DIR) $(OBJECTS) Makefile
	@echo 'Invoking: Linker'
	@echo 'OBJECTS = ' $(OBJECTS)
	$(LN) $(OBJECTS) $(LIBS) $(LNFLAGS) --config $(LNSCRIPT) -o $(BUILD_DIR)/$(TARGET).elf --map $(BUILD_DIR)/$(TARGET).map
	$(CP) $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).elf --fill 0xFF;0x08000000-__build_checksum --checksum __build_checksum:4,crc=0x04C11DB7,0x0;0x8000000-__build_checksum --checksum __version:8,crc=0x04C11DB7:i,0x$(FIRMWARE_BUILD_VERSION);

$(LIST_DIR):
	mkdir $@

###########################################
### BUILD TARGETS
###########################################

# All target
all: create_build_dir $(OBJS) $(BUILD_DIR)/$(TARGET).elf
	@echo 'Finished building all targets'

# Other targets
clean:
	@echo 'Cleaning up: $(OBJS) $(TARGET).elf $(TARGET).map'
	del /Q Build
	@echo 'Cleaned up object files and binaries.'

# Generate binary and hex files
$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(HEX) $< $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
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