#
# author	: huang li long <huanglilongwk@outlook.com>
# time		: 2016/08/02
# brief		: configure and build nuttx
#

.PHONY: all

# get current top makefile's absolute path 
PATH_BASE := $(realpath $(dir $(lastword $(MAKEFILE_LIST))))

# board
BOARD := stm32f429discovery

# nuttx path
NUTTX_SRC := $(PATH_BASE)/nuttx/nuttx

# nuttx export directory path
NUTTX_BUILD_DIR := $(PATH_BASE)/build

# nuttx board export 
NUTTX_EXPORT = $(BOARD).export

# jobs
J ?= 4

all:$(NUTTX_EXPORT)

# config and export nuttx
$(NUTTX_EXPORT) : $(NUTTX_SRC)
	@echo %
	@echo % First configure Nuttx for $(BOARD)
	@echo %
	cd $(NUTTX_SRC)/configs && cp -rf $(PATH_BASE)/nuttx-configs/$(BOARD) .
	cd $(NUTTX_SRC)/tools && ./configure.sh $(BOARD)/nsh_romfs
	@echo %
	@echo % Second export Nuttx for $(BOARD)
	@echo %
	$(MAKE) -r -j$(J) -C $(NUTTX_SRC) -r CONFIG_ARCH_BOARD=$(BOARD) export
	mkdir -p $(NUTTX_BUILD_DIR)
	cp -rf $(NUTTX_SRC)/nuttx-export.zip $(NUTTX_BUILD_DIR)/$@
	cd $(NUTTX_SRC)/configs && rm -rf $(BOARD)

