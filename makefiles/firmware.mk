#
# author    : huang li long <huanglilongwk@outlook.com>
# time      : 2016/08/08
# brief     : deal with user's code
# ref       : https://github.com/hll4fork/Firmware-old/blob/master/makefiles/firmware.mk
# 

# get firmware.mk file's directory
MK_DIR :=$(realpath $(dir $(lastword $(MAKEFILE_LIST))))
$(info % MK_DIR = $(MK_DIR))

# set PATH_BASE variable
ifeq ($(PATH_BASE),) # not set
export PATH_BASE  := $(abspath $(MK_DIR)/..)
endif

# module build dir
#BUILD_SRC_DIR := $(BUILD_DIR)/src

# common
include $(MK_DIR)/setup.mk

# include configure makefile 
include $(MK_DIR)/config_stm32f4.mk

# include nuttx makefile for unzip nuttx-export
include $(MK_DIR)/nuttx.mk

# include toolchain makefile
include $(MK_DIR)/toolchain-arm.mk

# modules search path
MODULE_SEARCH_DIRS += $(MODULE_SRC_DIR)

# collect module makfiles
MODULE_MAKEFILES := $(foreach module,$(MODULES),$(MODULE_SRC_DIR)/$(module)/module.mk)

# module objects
MODULE_OBJS      := $(foreach module,$(MODULES),$(BUILD_SRC_DIR)/$(module)/module.o)

# rules to build modules
# relpath : module real path     --> $(MODULE_SRC_DIR)/buttons
# mkfile  : module's makefile    --> $(MODULE_SRC_DIR)/buttons/module.mk
# workdir : build directory      --> $(BUILD_SRC_DIR)/buttons
.PHONY:$(MODULE_OBJS)
$(MODULE_OBJS):		srcpath = $(patsubst $(BUILD_SRC_DIR)%,$(MODULE_SRC_DIR)%,$@)
$(MODULE_OBJS):		mkfile  = $(patsubst %module.o,%module.mk,$(srcpath))
$(MODULE_OBJS):		workdir = $(@D)
$(MODULE_OBJS):		$(GLOBAL_DEPS) $(NUTTX_CONFIG_HEADER)
	@echo %
	@echo % Build modules $(MODULE_OBJS)
	@echo %
	mkdir -p $(workdir)
	$(MAKE) -r -f $(MK_DIR)/module.mk \
	-C $(workdir) \
	MODULE_WORK_DIR=$(workdir) \
	MODULE_OBJ=$@ \
	MODULE_MK=$(mkfile) \
	MODULE_NAME=$(lastword $(subst /, ,$(workdir))) \
	module


# build firmware
.PHONY:			 firmware
FIRMWARE_BIN	 = $(BUILD_DIR)/firmware.bin
FIRMWARE_ELF     = $(BUILD_DIR)/firmware.elf

# unzip Nuttx's export
firmware:$(FIRMWARE_BIN)

$(FIRMWARE_BIN):		$(FIRMWARE_ELF)
	$(call SYM_TO_BIN,$<,$@)

$(FIRMWARE_ELF):		$(MODULE_OBJS) $(GLOBAL_DEPS) $(LINK_DEPS) $(MODULE_MKFILES)
	$(call LINK,$@,$(MODULE_OBJS))

-include $(DEP_INCLUDES)
