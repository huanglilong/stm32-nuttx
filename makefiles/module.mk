#
# author    : huang li long <huanglilongwk@outlook.com>
# time      : 2016/08/08
# brief     : deal with modules
# ref       : https://github.com/hll4fork/Firmware-old/blob/master/makefiles/module.mk
#

# checking 
ifeq ($(MODULE_MK),)
$(error No module makefile specified)
endif
$(info %% MODULE_MK     = $(MODULE_MK))

# include toolchain makefile
include $(MAKEFILE_DIR)/toolchain-arm.mk

# include each module makefile
include $(MODULE_MK)
MODULE_SRC		:= $(dir $(MODULE_MK))
$(info %  MODULE_NAME         = $(MODULE_NAME))
$(info %  MODULE_SRC          = $(MODULE_SRC))
$(info %  MODULE_OBJ          = $(MODULE_OBJ))
$(info %  MODULE_WORK_DIR     = $(MODULE_WORK_DIR))

# Things that, if they change, might affect everything
GLOBAL_DEPS		+= $(MAKEFILE_LIST)

# module build rules
module:			$(MODULE_OBJ)

# Object files we will generate from sources
OBJS			 = $(addsuffix .o,$(SRCS))

# Dependency files that will be auto-generated
DEPS			 = $(addsuffix .d,$(SRCS))

$(OBJS):		$(GLOBAL_DEPS)

vpath %.c $(MODULE_SRC)
$(filter %.c.o,$(OBJS)): %.c.o: %.c $(GLOBAL_DEPS)
	$(call COMPILE,$<,$@)

vpath %.cpp $(MODULE_SRC)
$(filter %.cpp.o,$(OBJS)): %.cpp.o: %.cpp $(GLOBAL_DEPS)
	$(call COMPILEXX,$<,$@)

vpath %.S $(MODULE_SRC)
$(filter %.S.o,$(OBJS)): %.S.o: %.S $(GLOBAL_DEPS)
	$(call ASSEMBLE,$<,$@)

# Built product rules
$(MODULE_OBJ):		$(OBJS) $(GLOBAL_DEPS)
	$(call PRELINK,$@,$(OBJS))

-include $(DEPS)