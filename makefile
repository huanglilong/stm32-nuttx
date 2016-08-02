#
# author	: huang li long <huanglilongwk@outlook.com>
# time		: 2016/08/02
# brief		: configure and build nuttx
#

# get current top makefile's absolute path 
PATH_BASE := $(realpath $(dir $(lastword $(MAKEFILE_LIST))))

# board
BOARD := stm32f429discovery

# nuttx path
NUTTX_SRC := $(PATH_BASE)/nuttx/nuttx

# nuttx export directory path
NUTTX_EXPORT_DIR := $(PATH_BASE)/export

# nuttx board export 
NUTTX_EXPORT = $(BOARD).export

# jobs
J ?= 4

# config and export nuttx
$(NUTTX_EXPORT) : $(NUTTX_SRC)
	@echo First configure Nuttx for $(BOARD)
	cd $(NUTTX_SRC)/configs && cp -rf $(PATH_BASE)/nuttx-configs/$(BOARD) .
	cd $(NUTTX_SRC)/tools && ./configure.sh $(BOARD)/nsh_romfs
	@echo Second export Nuttx for $(BOARD)
	$(MAKE) -r -j$(J) -C $(NUTTX_SRC) -r CONFIG_ARCH_BOARD=$(BOARD) export
	mkdir -p $(NUTTX_EXPORT_DIR)
	cp -rf $(NUTTX_SRC)/nuttx-export.zip $(NUTTX_EXPORT_DIR)/$@
	cd $(NUTTX_SRC)/configs && rm -rf $(BOARD)

