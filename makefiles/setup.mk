#
# author    : huang li long <huanglilongwk@outlook.com>
# time      : 2016/08/08
# brief     : common setups
# ref       : https://github.com/hll4fork/Firmware-old/blob/master/makefiles/setup.mk
#

export MODULE_SRC_DIR   = $(abspath $(PATH_BASE)/src)
export MAKEFILE_DIR	    = $(abspath $(PATH_BASE)/makefiles)
export NUTTX_SRC	    = $(abspath $(PATH_BASE)/nuttx/nuttx)
export BUILD_DIR	    = $(abspath $(PATH_BASE)/build)
export NUTTX_EXPORT 	= $(BUILD_DIR)/$(BOARD).zip

# include path 
export INCLUDE_DIRS	:=  $(MODULE_SRC_DIR)