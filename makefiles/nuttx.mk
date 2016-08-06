#
# author	: huang li long <huanglilongwk@outlook.com>
# time		: 2016/08/04
# brief		: deal with $(BOARD).zip
#

# unzip directory
NUTTX_EXPORT_DIR	 = $(BUILD_DIR)/nuttx-export
NUTTX_CONFIG_HEADER	 = $(NUTTX_EXPORT_DIR)/include/nuttx/config.h

GLOBAL_DEPS			 += $(NUTTX_CONFIG_HEADER)

# NuttX's linker script
LDSCRIPT			 += $(NUTTX_EXPORT_DIR)/build/ld.script

# Add directories from the NuttX export to the relevant search paths
INCLUDE_DIRS   		 += $(NUTTX_EXPORT_DIR)/include     \
			   			$(NUTTX_EXPORT_DIR)/include/cxx \
			   			$(NUTTX_EXPORT_DIR)/arch/chip   \
			   			$(NUTTX_EXPORT_DIR)/arch/common

LIB_DIRS			 += $(NUTTX_EXPORT_DIR)/libs
LIBS				 += -lapps -lnuttx
NUTTX_LIBS			  = $(NUTTX_EXPORT_DIR)/libs/libapps.a \
			    		$(NUTTX_EXPORT_DIR)/libs/libnuttx.a
LINK_DEPS			 += $(NUTTX_LIBS)

$(NUTTX_CONFIG_HEADER):	$(NUTTX_EXPORT)
	@echo %% Unpacking $(NUTTX_EXPORT)
	unzip -q -o -d $(BUILD_DIR) $(BUILD_DIR)/$(NUTTX_EXPORT)
	touch $@

 #$(LDSCRIPT): $(NUTTX_CONFIG_HEADER)
 #$(NUTTX_LIBS): $(NUTTX_CONFIG_HEADER)
