#
# author	: huang li long <huanglilongwk@outlook.com>
# time		: 2016/08/02
# brief		: configure and build nuttx
#

.PHONY: archive firmware clean distclean upload

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
	@echo % Configure Nuttx for $(BOARD)
	@echo %
	cd $(NUTTX_SRC)/configs && cp -rf $(PATH_BASE)/nuttx-configs/$(BOARD) .
	cd $(NUTTX_SRC)/tools && ./configure.sh $(BOARD)/$(CONGIF)
	@echo %
	@echo % Build Nuttx for $(BOARD)
	@echo %
	$(MAKE) -r -j$(J) -C $(NUTTX_SRC) -r CONFIG_ARCH_BOARD=$(BOARD) export
	mkdir -p $(BUILD_DIR)
	cp -rf $(NUTTX_SRC)/nuttx-export.zip $@
	cd $(NUTTX_SRC)/configs && rm -rf $(BOARD)

# build firmware
firmware:
	@echo %%%%
	@echo %%%% Building firmware
	@echo %%%%
	mkdir -p $(BUILD_DIR)
	$(MAKE) -r -C $(BUILD_DIR) \
	-f $(MAKEFILE_DIR)/firmware.mk \
	firmware

# clean firmware
clean:
	@echo %
	@echo % Clean firmware
	@echo %
	@rm -rf $(BUILD_SRC_DIR)

# clean firmware, archive and configure
distclean:
	@echo %
	@echo % Clean firmware, archive and configure
	@echo %
	@rm -rf $(BUILD_DIR)
	$(MAKE) -r -j$(J) -C $(NUTTX_SRC) distclean

# upload firmware
upload:
	st-flash write $(FIRMWARE_BIN) 0x8000000
