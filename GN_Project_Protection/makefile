####################################################################################################
#
#
#                      Eaton Electrical
#
#                      Proprietary Information
#                      (C) Copyright 2020
#                      All rights reserved
#
#**************************************************************************************************
#  Written by:         Aaron Joseph
#                      Eaton Electrical
#                      1000 Cherrington Parkway
#                      Pittsburgh, PA  15108-4312
#                      (412) 893-3470
#
####################################################################################################

.EXPORT_ALL_VARIABLES:

######################################
# components
######################################
COMPONENTS := main adc iwdg gpio timer sys btn trip micro_test flash led trip_log self_test shunt_cal secondary_solenoid open_fdbk usart primary_switch power_loss_detection temperature_control spi
DRIVER_COMPONENT := utils exti gpio adc dma rcc pwr tim usart spi

IAR_PATH = "C:\Program Files (x86)\IAR Systems\Embedded Workbench 8.3\arm"
IAR_BIN_PATH = $(IAR_PATH)\bin
CYGWIN_DIR = C:\cygwin64\bin
LIB_PATH = C:/dev/libs

HAL_DRIVER_PATH = $(LIB_PATH)/ST/stm32/stm32g0xx/STM32G0xx_HAL_Driver
CMSIS_DRIVER_PATH = $(LIB_PATH)/ST/stm32/stm32g0xx/CMSIS
UTILS_PATH = $(LIB_PATH)/utilities
BRK_CORE_PATH = $(LIB_PATH)/breaker_core

DoxygenBinPath="C:\Program Files\doxygen\bin"

#### May not be used ################################################
IAR_BUILD_CMD = $(IAR_BIN_PATH)/iccarm.exe
ASM_BUILD_CMD = $(IAR_BIN_PATH)/iasmarm.exe
LNK_CMD       = $(IAR_BIN_PATH)/ilinkarm.exe

######################################
# target
###################################### 
TARGET = GN_Project_Protection

######################################
# configuration file path 
###################################### 
# this is a default value, should be updated once a target is chosen. 
# for ex. if compiling release ch1p gfci, should call 
# build target : release CONFIG_FILE_PATH=src\config\CH\1_pole\GFCI
CONFIG_FILE_PATH := src\config\SB\2_pole\GFCI

######################################
# building variables
######################################

# optimization
OPT = -Ol


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build
LIST_DIR = list
RELEASE_DIR = release
SRC_DIR = src
######################################
# source
######################################
# C sources
C_FILES = $(addsuffix .c, $(COMPONENTS))
C_FILES += $(addsuffix .c, $(addprefix stm32g0xx_ll_,$(DRIVER_COMPONENT)))
C_SRC_DIRS = $(addprefix $(SRC_DIR)/, $(COMPONENTS))
C_SRC_DIRS += src/interrupt# interrupt doesn't have a specific pattern 
C_SRC_DIRS += $(HAL_DRIVER_PATH)/src

C_SOURCES = $(join $(addsuffix /, $(C_SRC_DIRS)), $(C_FILES))
C_SOURCES += src/sys/system_stm32g0xx.c# this does not have specific name patern. 
C_SOURCES += src/sys/hardware_init.c# this does not have specific name patern. 
C_SOURCES += src/interrupt/stm32g0xx_it.c# this does not have specific name patern. 
C_SOURCES += src/self_test/hf_sense.c# this does not have specific name patern. 
C_SOURCES += src/self_test/interrupt_check.c# this does not have specific name patern. 
C_SOURCES += src/self_test/sequence_number_check.c# this does not have specific name patern. 
C_SOURCES += src/self_test/data_overrun_check.c# this does not have specific name patern. 
C_SOURCES += src/self_test/line_current_bias_check.c# this does not have specific name patern. 
C_SOURCES += src/self_test/ground_fault_bias_check.c# this does not have specific name patern. 
C_SOURCES += src/self_test/ptt.c# this does not have specific name patern. 
C_SOURCES += src/self_test/auto_monitor.c# this does not have specific name patern. 
C_SOURCES += src/sys/dataAcq.c# this does not have specific name patern. 
C_SOURCES += src/sys/acqTrigger.c# this does not have specific name patern. 
C_SOURCES += ../dev/libs/ST/stm32/stm32g0xx/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal.c
C_SOURCES += ../dev/libs/ST/stm32/stm32g0xx/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_cortex.c

HEADER_DIRS += $(HAL_DRIVER_PATH)/Inc

API_H += $(foreach dir,$(HEADER_DIRS),$(wildcard $(dir)/*.h))
API_H += $(foreach dir,$(C_SRC_DIRS),$(wildcard $(dir)/*api.h))

# ASM sources
ASM_SOURCES =  \
src/sys/startup_stm32g071xx.s\
src/micro_test/continuous_cpu_check.s

#dlib
DLIB = --dlib_config $(IAR_PATH)/inc/c/DLib_Config_Full.h

# compile flags
ASFLAGS = -s+ -w+ -r --cpu $(CPU) $(AS_DEFS) $(AS_INCLUDES)

######################################
# libraries
######################################

LIBS = 

#######################################
# binaries
#######################################

CC = $(IAR_BIN_PATH)\iccarm.exe
AS = $(IAR_BIN_PATH)\iasmarm.exe
LN = $(IAR_BIN_PATH)\ilinkarm.exe

CP = $(IAR_BIN_PATH)\ielftool.exe

HEX = $(CP) --ihex --verbose
SREC = $(CP) --srec --verbose

#######################################
# CFLAGS
#######################################
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

# macros

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

# C includes
C_INCLUDES = $(addprefix -I$(SRC_DIR)/, $(COMPONENTS))
#C_INCLUDES += -Isrc/sys
C_INCLUDES += -Isrc/interrupt
C_INCLUDES += -I$(HAL_DRIVER_PATH)/Inc
C_INCLUDES += -I$(CMSIS_DRIVER_PATH)/Device/ST/STM32G0xx/Include
C_INCLUDES += -I$(CMSIS_DRIVER_PATH)/Include
#C_INCLUDES += -I$(UTILS_PATH)/utilities-0.0.2/inc

#dlib
DLIB = --dlib_config $(IAR_PATH)/inc/c/DLib_Config_Full.h

# compile gcc flags
ASFLAGS = -s+ -w+ -r --cpu $(CPU) $(AS_DEFS) $(AS_INCLUDES)


#CFLAGS += --diag_suppress Pa183  #suppress user added warnings
LNFLAGS = --redirect _//printf=_//printfFull --redirect _Scanf=_ScanfFull --no_out_extension --semihosting --entry __iar_program_start --vfe --text_out locale 
LNFLAGS += --place_holder __build_checksum,4,.checksum,16 --keep=__build_checksum 
LNFLAGS += --place_holder __version,8,.version,16 --keep=__version 


RELEASE_CFLAGS = $(MCU) $(C_DEFS) $(DLIB) $(C_INCLUDES) $(OPT) $(LANG_EXT_ENB) --f release.config -DSTATIC="static"
DEBUG_CFLAGS = $(MCU) $(C_DEFS) $(DLIB) $(C_INCLUDES) $(OPT) $(LANG_EXT_ENB) --debug --f debug.config -DSTATIC="static"

#ifeq ($(DEBUG), 1)
#CFLAGS += -g -gdwarf-2
#endif


# Generate dependency information
#CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LNSCRIPT = src/sys/stm32g071xx_flash.icf


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

PREPROCESS_OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.i)))
vpath %.c $(sort $(dir $(C_SOURCES)))


.PHONY: preprocess
preprocess: $(BUILD_DIR) $(PREPROCESS_OBJECTS) Makefile

#delete old version, copy new version from build and rename. 
release_project:
	@echo **** copying hex files for release ****
	-@erase /Q $(RELEASE_DIR)\hex\*
	copy $(BUILD_DIR)\$(TARGET).hex $(RELEASE_DIR)\hex
	rename  $(RELEASE_DIR)\hex\$(TARGET).hex $(RELEASE_TARGET_VERSION).hex

# release will enable watchdog	
.PHONY: release
release: TARGET_PIN_CONFIG_FILE := $(shell dir /b/s $(CONFIG_FILE_PATH)\*.pin_config)
release: TARGET_FIRMWARE_CONFIG_FILE := $(shell dir /b/s $(CONFIG_FILE_PATH)\*.config)
release: TARGET_HARDWARE_INIT_FILE := $(shell dir /b/s $(CONFIG_FILE_PATH)\*hardware_init.c)
release: TARGET_VERSION_FILE := $(CONFIG_FILE_PATH)\version.txt
release: TARGET_VERSION := $(shell type $(TARGET_VERSION_FILE))
release: RELEASE_TARGET_VERSION := $(word 1, $(TARGET_VERSION))
release: FIRMWARE_BUILD_VERSION := $(word 2, $(TARGET_VERSION))
release: BRK_CORE_VERSION := $(word 3, $(TARGET_VERSION))
release: UTILS_VERSION := $(word 4, $(TARGET_VERSION))
release: LIBS := $(UTILS_PATH)/$(UTILS_VERSION)/lib/$(UTILS_VERSION).a $(BRK_CORE_PATH)/$(BRK_CORE_VERSION)/lib/$(BRK_CORE_VERSION).a
release: C_INCLUDES += -I$(BRK_CORE_PATH)/$(BRK_CORE_VERSION)/inc
release: C_INCLUDES += -I$(UTILS_PATH)/$(UTILS_VERSION)/inc
release: CFLAGS = $(RELEASE_CFLAGS)
release: load_hardware_config $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex release_project
# end release

# default action: debug
.PHONY: debug
debug: TARGET_PIN_CONFIG_FILE := $(shell dir /b/s $(CONFIG_FILE_PATH)\*.pin_config)
debug: TARGET_FIRMWARE_CONFIG_FILE := $(shell dir /b/s $(CONFIG_FILE_PATH)\*.config)
debug: TARGET_HARDWARE_INIT_FILE := $(shell dir /b/s $(CONFIG_FILE_PATH)\*hardware_init.c)
debug: TARGET_VERSION_FILE := $(CONFIG_FILE_PATH)\version.txt
debug: TARGET_VERSION := $(shell type $(TARGET_VERSION_FILE))
debug: RELEASE_TARGET_VERSION := $(word 1, $(TARGET_VERSION))
debug: FIRMWARE_BUILD_VERSION := $(word 2, $(TARGET_VERSION))
debug: BRK_CORE_VERSION := $(word 3, $(TARGET_VERSION))
debug: UTILS_VERSION := $(word 4, $(TARGET_VERSION))
debug: LIBS := $(UTILS_PATH)/$(UTILS_VERSION)/lib/$(UTILS_VERSION).a $(BRK_CORE_PATH)/$(BRK_CORE_VERSION)/lib/$(BRK_CORE_VERSION).a
debug: C_INCLUDES += -I$(BRK_CORE_PATH)/$(BRK_CORE_VERSION)/inc
debug: C_INCLUDES += -I$(UTILS_PATH)/$(UTILS_VERSION)/inc
debug: CFLAGS = $(DEBUG_CFLAGS)
debug: load_hardware_config $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex  

$(BUILD_DIR)/%.o: %.c Makefile FORCE
	@echo **** building $< ****
	$(CC) $< $(CFLAGS) -o $@
	@echo ' '

	
$(BUILD_DIR)/%.i: %.c Makefile FORCE
	@echo **** building $< ****
	$(CC) $< $(CFLAGS) --preprocess=n $(BUILD_DIR) -o $@
	@echo ' '
	

	
#this loads the correct hardware/pin configuration from the config/ folder into the sys folder
#that configuration is what is used during the build
load_hardware_config:
	-@erase /Q $(SRC_DIR)\sys\*.pin_config
	-@erase /Q $(SRC_DIR)\sys\*firmware.config
	-@erase /Q $(SRC_DIR)\sys\*hardware_init.c
	copy $(TARGET_PIN_CONFIG_FILE) $(SRC_DIR)\sys\hardware.pin_config
	copy $(TARGET_FIRMWARE_CONFIG_FILE) $(SRC_DIR)\sys\firmware.config
	copy $(TARGET_HARDWARE_INIT_FILE) $(SRC_DIR)\sys\hardware_init.c
	@echo TARGET_VERSION_FILE = $(TARGET_VERSION_FILE)
	@echo TARGET_VERSION = $(TARGET_VERSION)
	@echo RELEASE_TARGET_VERSION = $(RELEASE_TARGET_VERSION)
	@echo FIRMWARE_BUILD_VERSION = $(FIRMWARE_BUILD_VERSION)
	@echo CFLAGS = $(CFLAGS)
	@echo using breaker-core : $(BRK_CORE_VERSION)
	@echo using utilities : $(UTILS_VERSION)
	
	
#FORCE target just forces any targets that list it as a dependency to be run
FORCE:

echo_vars:
	@echo MAKECMDGOALS =      $(MAKECMDGOALS)
	@echo BUILD_DIR =         $(BUILD_DIR)
	@echo RELEASE_DIR =       $(RELEASE_DIR)
	@echo TARGET =            $(TARGET)
	@echo C_FILES =           $(C_FILES)
	@echo C_SRC_DIRS =        $(C_SRC_DIRS)
	@echo C_SOURCES =         $(C_SOURCES)
	@echo HEADER_DIRS =       $(HEADER_DIRS)
	@echo API_H =             $(API_H)
	@echo C_INCLUDES =        $(C_INCLUDES)
	@echo OBJECTS =           $(OBJECTS)
	@echo BRK_CORE_PATH =     $(BRK_CORE_PATH)
	@echo LIBS =              $(LIBS)
	@echo BUILD_DIR\TARGET =  $(BUILD_DIR)/$(TARGET)
	@echo PATH =              %PATH%
	@echo SHELL =             %SHELL%


$(BUILD_DIR)/%.o: %.s Makefile
	@echo **** building $< ****
	$(AS) $< -o $@ $(ASFLAGS) 
	@echo ' '

#Using ielftool to generate checksum after linking from the begining of the flash to where __build_checksum is stored.
#It then uses the checksum feature again to generate the firmware build version and store it in __version.(calculating 0 bytes)
#This is a work around because there's no tool(function) that can specifically write and change __version varriable. 
$(BUILD_DIR)/$(TARGET).elf: $(BUILD_DIR) $(LIST_DIR) $(OBJECTS) Makefile
	$(LN) $(OBJECTS) $(LIBS) $(LNFLAGS) --config $(LNSCRIPT) -o $(BUILD_DIR)/$(TARGET).elf --map $(BUILD_DIR)/$(TARGET).map
	$(CP) $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).elf --fill 0xFF;0x08000000-__build_checksum --checksum __build_checksum:4,crc=0x04C11DB7,0x0;0x8000000-__build_checksum --checksum __version:8,crc=0x04C11DB7:i,0x$(FIRMWARE_BUILD_VERSION);


$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(HEX) $< $@
	
$(BUILD_DIR):
	mkdir $@		

$(RELEASE_DIR):
	$(CYGWIN_DIR)\mkdir.exe -p $@
	
$(LIST_DIR):
	mkdir $@
	
$(RELEASE_DIR)\hex: $(RELEASE_DIR)
	$(CYGWIN_DIR)\mkdir.exe -p $(RELEASE_DIR)\hex
	
	
#######################################
# clean up
#######################################
clean:
	-@erase /Q $(BUILD_DIR)\*


#######################################
# Documentation
#######################################
documentation: $(RELEASE_DIR)
documentation: DOXY_OPT := $(SRC_DIR)/doxygen/internal/DoxyOptions
documentation: DOXY_INC := $(C_SRC_DIRS)
documentation: DIA_DIRS = $(C_SRC_DIRS)
documentation:
	@echo $(DOXY_OPT)
	@echo $(DOXY_INC)
	@echo $(DIA_DIRS)
#	@erase $(DOXY_OPT)
#	@touch $(DOXY_OPT)
	@echo PROJECT_NAME = $(PROD_NAME) > $(DOXY_OPT)
	@echo OUTPUT_DIRECTORY = $(RELEASE_DIR)/doxygen >> $(DOXY_OPT)
	@echo DIAFILE_DIRS = $(DIA_DIRS) >> $(DOXY_OPT)
	$(DoxygenBinPath)\doxygen $(SRC_DIR)/doxygen/internal/doxy_config


#######################################
# githash
#######################################
git_hash:
	git describe --abbrev=40 --dirty --always --long
  
  
#######################################
# dependencies
#######################################
#-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***

