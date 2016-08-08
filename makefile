#
# author	: huang li long <huanglilongwk@outlook.com>
# time		: 2016/08/02
# brief		: configure and build nuttx
#

.PHONY: firmware clean distclean upload

# get current top makefile's absolute path 
export PATH_BASE := $(realpath $(dir $(lastword $(MAKEFILE_LIST))))

# include setup makefile
include $(PATH_BASE)/makefiles/setup.mk

# board
export BOARD := stm32f429discovery

# config
export CONGIF := nsh_romfs

# firmware file
FIRMWARE_BIN := $(BUILD_DIR)/firmware.bin
FIRMWARE_ELF := $(BUILD_DIR)/firmware.elf

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
	cd $(NUTTX_SRC)/tools && ./configure.sh $(BOARD)/$(CONGIF)
	@echo %
	@echo % Second build Nuttx for $(BOARD)
	@echo %
	$(MAKE) -r -j$(J) -C $(NUTTX_SRC) -r CONFIG_ARCH_BOARD=$(BOARD) export
	mkdir -p $(BUILD_DIR)
	cp -rf $(NUTTX_SRC)/nuttx-export.zip $@
	cd $(NUTTX_SRC)/configs && rm -rf $(BOARD)

firmware:
	@echo %%%%
	@echo %%%% Building firmware
	@echo %%%%
	mkdir -p $(BUILD_DIR)
	$(MAKE) -r -C $(BUILD_DIR) \
		-f $(MAKEFILE_DIR)/firmware.mk \
		firmware

distclean:
	rm -rf build/*
	$(MAKE) -r -j$(J) -C $(NUTTX_SRC) distclean

upload:
	st-flash write $(FIRMWARE_BIN) 0x8000000
