#
# author	: huang li long <huanglilongwk@outlook.com>
# time		: 2016/08/02
# brief		: configure and build nuttx
#

.PHONY: all clean

# get current top makefile's absolute path 
PATH_BASE := $(realpath $(dir $(lastword $(MAKEFILE_LIST))))

# board
BOARD := stm32f429i-disco

# nuttx path
NUTTX_SRC := $(PATH_BASE)/nuttx/nuttx

# nuttx export directory path
BUILD_DIR := $(PATH_BASE)/build

# jobs
J ?= 4

NUTTX_BIN = $(NUTTX_SRC)/nuttx.bin

all: firmware upload

firmware:$(NUTTX_BIN)

# config and export nuttx
$(NUTTX_BIN) : $(NUTTX_SRC)
	@echo %
	@echo % First configure Nuttx for $(BOARD)
	@echo %
	cd $(NUTTX_SRC)/tools && ./configure.sh $(BOARD)/nsh
	@echo %
	@echo % Second build Nuttx for $(BOARD)
	@echo %
	$(MAKE) -r -j$(J) -C $(NUTTX_SRC)

clean:
	rm -rf build/*
	$(MAKE) -r -j$(J) -C $(NUTTX_SRC) distclean

upload:$(NUTTX_BIN)
	st-flash write $(NUTTX_BIN) 0x8000000
