#
# author	: huang li long <huanglilongwk@outlook.com>
# time		: 2016/08/02
# brief		: configure and build nuttx
#

.PHONY: all clean upload

all:archive firmware

# get current top makefile's absolute path 
PATH_BASE := $(realpath $(dir $(lastword $(MAKEFILE_LIST))))

# board
BOARD := stm32f429discovery

# nuttx path
NUTTX_SRC := $(PATH_BASE)/nuttx/nuttx

# nuttx export directory path
BUILD_DIR := $(PATH_BASE)/build

# nuttx board export 
NUTTX_EXPORT = $(BOARD).zip

# firmware file
FIRMWARE_BIN := $(PATH_BASE)/firmware.bin

# jobs
J ?= 4

# build nuttx and export
archive:$(NUTTX_EXPORT) 

# config and export nuttx
$(NUTTX_EXPORT) : $(NUTTX_SRC)
	@echo %
	@echo % First configure Nuttx for $(BOARD)
	@echo %
	cd $(NUTTX_SRC)/configs && cp -rf $(PATH_BASE)/nuttx-configs/$(BOARD) .
	cd $(NUTTX_SRC)/tools && ./configure.sh $(BOARD)/nsh_romfs
	@echo %
	@echo % Second build Nuttx for $(BOARD)
	@echo %
	$(MAKE) -r -j$(J) -C $(NUTTX_SRC) -r CONFIG_ARCH_BOARD=$(BOARD) export
	mkdir -p $(BUILD_DIR)
	cp -rf $(NUTTX_SRC)/nuttx-export.zip $(BUILD_DIR)/$@
	cd $(NUTTX_SRC)/configs && rm -rf $(BOARD)

include $(PATH_BASE)/makefiles/toolchain-arm.mk
include $(PATH_BASE)/makefiles/nuttx.mk

# unzip Nuttx's export
firmware:$(NUTTX_CONFIG_HEADER) firmware.elf firmware.bin

# build user's src
SRCS			 = main.c
OBJS			 = $(addsuffix .o,$(SRCS))
DEPS			 = $(addsuffix .d,$(SRCS))

$(OBJS):$(SRCS)
	$(call COMPILE,$<,$@)

firmware.bin:		firmware.elf
	$(call SYM_TO_BIN,$<,$@)

firmware.elf:		$(OBJS) $(LINK_DEPS)
	$(call LINK,$@,$(OBJS))

clean:
	@rm -rf build/*
	@rm -f *.elf
	@rm -f *.bin
	@rm -f *.d
	@rm -f *.o
	@rm -f *.map
	$(MAKE) -r -j$(J) -C $(NUTTX_SRC) distclean

upload:$(FIRMWARE_BIN)
	st-flash write $(FIRMWARE_BIN) 0x8000000

-include $(DEPS)