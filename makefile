#
# author	: huang li long <huanglilongwk@outlook.com>
# time		: 2016/08/02
# brief		: configure and build nuttx
#

.PHONY: all clean romfs

# get current top makefile's absolute path 
PATH_BASE := $(realpath $(dir $(lastword $(MAKEFILE_LIST))))

# board
BOARD := stm32f429discovery

# nuttx path
NUTTX_SRC := $(PATH_BASE)/nuttx/nuttx

# romfs file
ROMFSIMG  := $(PATH_BASE)/nuttx-configs/$(BOARD)/include/nsh_romfsimg.h

# romfs directory
ROMFS_DIR := $(PATH_BASE)/etc

# nuttx export directory path
BUILD_DIR := $(PATH_BASE)/build

# jobs
J ?= 4

NUTTX_BIN = $(NUTTX_SRC)/nuttx.bin

all: firmware upload

firmware:romfs $(NUTTX_BIN)

# config and export nuttx
$(NUTTX_BIN) : $(NUTTX_SRC)
	@echo %
	@echo % First configure Nuttx for $(BOARD)
	@echo %
	cd $(NUTTX_SRC)/configs && cp -rf $(PATH_BASE)/nuttx-configs/$(BOARD) .
	cd $(NUTTX_SRC)/tools && ./configure.sh $(BOARD)/nsh_romfs
	@echo %
	@echo % Second build Nuttx for $(BOARD)
	@echo %
	$(MAKE) -r -j$(J) -C $(NUTTX_SRC)
	cd $(NUTTX_SRC)/configs && rm -rf $(BOARD)

clean:
	rm -rf build/*
	$(MAKE) -r -j$(J) -C $(NUTTX_SRC) distclean

upload:$(NUTTX_BIN)
	st-flash write $(NUTTX_BIN) 0x8000000

romfs:$(ROMFS_DIR) $(ROMFSIMG)
	@echo %
	@echo % generate romfs
	@echo %
	genromfs -f $(PATH_BASE)/romfs_img -d $(PATH_BASE)/etc
	xxd -i $(PATH_BASE)/romfs_img > $@
	rm -f $(PATH_BASE)/romfs_img
	rm -f $(PATH_BASE)/romfs