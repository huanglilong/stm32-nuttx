#
# author	: huang li long <huanglilongwk@outlook.com>
# time		: 2016/08/02
# brief		: configure and build nuttx
#

.PHONY: all

all: archive firmware

# get current top makefile's absolute path 
PATH_BASE := $(realpath $(dir $(lastword $(MAKEFILE_LIST))))

# board
BOARD := stm32f429discovery
CONFIG_BOARD := stm32f429discovery

# nuttx path
export NUTTX_SRC := $(PATH_BASE)/nuttx/nuttx

# nuttx export directory path
BUILD_DIR := $(PATH_BASE)/build

# nuttx board export 
NUTTX_EXPORT = $(BOARD).zip

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
	@echo % Second export Nuttx for $(BOARD)
	@echo %
	$(MAKE) -r -j$(J) -C $(NUTTX_SRC) -r CONFIG_ARCH_BOARD=$(BOARD) export
	mkdir -p $(BUILD_DIR)
	cp -rf $(NUTTX_SRC)/nuttx-export.zip $(BUILD_DIR)/$@
	cd $(NUTTX_SRC)/configs && rm -rf $(BOARD)

include $(PATH_BASE)/makefiles/toolchain-arm.mk
include $(PATH_BASE)/makefiles/nuttx.mk

# unzip Nuttx's export
firmware:$(NUTTX_CONFIG_HEADER) firmware.elf firmware.bin
#FIRMWARE_ELF := $(BUILD_DIR)/firmware.elf
#FIRMWARE_BIN := $(BUILD_DIR)/firmware.bin 

SRCS			 = main.c
OBJS			 = $(addsuffix .o,$(SRCS))
DEPS			 = $(addsuffix .d,$(SRCS))

$(OBJS):$(SRCS)
	$(call COMPILE,$<,$@)

# FIRMWARE_ELF:$(NUTTX_EXPORT) $(NUTTX_CONFIG_HEADER) main.o
# 	$(Q) $(LD) $(LDFLAGS) -Map firmware.map -o $@ --start-group main.o $(LIBS) --end-group

# FIRMWARE_BIN : FIRMWARE_ELF
# 	$(Q) $(OBJCOPY) -O binary $< $@

# firmware.elf:  main.o $(NUTTX_CONFIG_HEADER)
# 	$(CC) $(CFLAGS) main.o $(LIBS) --output $@ $(LDFLAGS)

firmware.bin:		firmware.elf
	$(call SYM_TO_BIN,$<,$@)

firmware.elf:		$(OBJS) $(LINK_DEPS)
	$(call LINK,$@,$(OBJS))

clean:
	$(MAKE) -C $(NUTTX_SRC) distclean
	rm -rf build
	rm -f *.elf
	rm -f *.bin
	rm -f *.d
	rm -f *.o
	rm -f *.map
-include $(DEPS)
