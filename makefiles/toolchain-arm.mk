#
# Definitions for a generic GNU ARM-EABI toolchain
#

# Toolchain commands. Normally only used inside this file.
# Note: 
#		1. $(TOOLCHAIN)gcc with -E option, nothing is done except preprocessing.
#		2. $(TOOLCHAIN)ar rcs; r: Insert the files member... into archive (with replacement);
#							   c: Create the archive
#							   a: Add new files after an existing member of the archive

TOOLCHAIN	 = arm-none-eabi-

CC			 = $(TOOLCHAIN)gcc
CXX			 = $(TOOLCHAIN)g++
CPP			 = $(TOOLCHAIN)gcc -E
LD			 = $(TOOLCHAIN)ld
AR			 = $(TOOLCHAIN)ar rcs
NM			 = $(TOOLCHAIN)nm
OBJCOPY		 = $(TOOLCHAIN)objcopy
OBJDUMP		 = $(TOOLCHAIN)objdump


# Cortex-M4 CPU flags
FLAGS_CORTEXM4F	 = -mcpu=cortex-m4 \
			       -mthumb \
			       -march=armv7e-m \
			       -mfpu=fpv4-sp-d16 \
			       -mfloat-abi=hard

# optimisation flags
OPTIMIZATION	 = 	-g3 \
					-fno-strict-aliasing \
					-fno-strength-reduce \
					-fomit-frame-pointer \
					-funsafe-math-optimizations \
					-fno-builtin-printf \
					-ffunction-sections \
					-fdata-sections

# Language-specific flags
ARCHCFLAGS		 = -std=gnu99
ARCHCXXFLAGS	 = -fno-exceptions -fno-rtti -std=gnu++0x -fno-threadsafe-statics

# Generic warnings
ARCHWARNINGS    = 	-Wall \
					-Wextra \
					-Werror \
					-Wdouble-promotion \
					-Wshadow \
					-Wfloat-equal \
					-Wframe-larger-than=1024 \
					-Wpointer-arith \
					-Wlogical-op \
					-Wmissing-declarations \
					-Wpacked \
					-Wno-unused-parameter \
					-Werror=format-security \
					-Werror=array-bounds \
					-Wfatal-errors \
					-Wformat=1 \
					-Werror=unused-but-set-variable \
					-Werror=unused-variable \
					-Werror=double-promotion \
					-Werror=reorder \
					-Werror=uninitialized \
					-Werror=init-self
#   -Werror=float-conversion - works, just needs to be phased in with some effort and needs GCC 4.9+
#   -Wcast-qual  - generates spurious noreturn attribute warnings, try again later
#   -Wconversion - would be nice, but too many "risky-but-safe" conversions in the code
#   -Wcast-align - would help catch bad casts in some cases, but generates too many false positives

# C-specific warnings
ARCHCWARNINGS	= 	$(ARCHWARNINGS) \
					-Wbad-function-cast \
					-Wstrict-prototypes \
					-Wold-style-declaration \
					-Wmissing-parameter-type \
					-Wmissing-prototypes \
					-Wnested-externs

# C++-specific warnings
#
ARCHWARNINGSXX	= $(ARCHWARNINGS) \
			   	  -Wno-missing-field-initializers

# pull in *just* libm from the toolchain ... this is grody
#LIBM			:= $(shell $(CC) $(ARCHCPUFLAGS) -print-file-name=libm.a)
#EXTRA_LIBS		+= $(LIBM)

# Flags we pass to the C compiler
CFLAGS		   = 	$(ARCHCFLAGS) \
					$(ARCHCWARNINGS) \
					$(OPTIMIZATION) \
					$(FLAGS_CORTEXM4F) \
					$(ARCHINCLUDES) \
					-fno-common \
					$(addprefix -I,$(INCLUDE_DIRS))

# Flags we pass to the C++ compiler
#
CXXFLAGS	   = $(ARCHCXXFLAGS) \
				 $(ARCHWARNINGSXX) \
				 $(OPTIMIZATION) \
				 $(FLAGS_CORTEXM4F) \
				 $(ARCHXXINCLUDES) \
				 $(ARCHDEFINES) \
				 -DCONFIG_WCHAR_BUILTIN \
				 $(EXTRADEFINES) \
				 $(EXTRACXXFLAGS) \
				 $(addprefix -I,$(INCLUDE_DIRS))

# Flags we pass to the assembler
#
AFLAGS		   = $(CFLAGS) -D__ASSEMBLY__ \
				 $(EXTRADEFINES) \
				 $(EXTRAAFLAGS)

# Flags we pass to the linker
#
# LDFLAGS		   += --warn-common \
# 				  --gc-sections \
# 				  $(EXTRALDFLAGS) \
# 				  $(addprefix -T,$(LDSCRIPT)) \
# 				  $(addprefix -L,$(LIB_DIRS))

LDFLAGS		   += $(addprefix -T,$(LDSCRIPT)) \
				  $(addprefix -L,$(LIB_DIRS))

# Compiler support library
#
#LIBGCC			:= $(shell $(CC) $(ARCHCPUFLAGS) -print-libgcc-file-name)

# Files that the final link depends on
#
LINK_DEPS		+= $(LDSCRIPT)

# Files to include to get automated dependencies
#
DEP_INCLUDES		 = $(subst .o,.d,$(OBJS))

# Compile C source $1 to object $2
# as a side-effect, generate a dependency file
#
define COMPILE
	@echo "CC:      $1"
	@mkdir -p $(dir $2)
	$(Q) $(CCACHE) $(CC) -MD -c $(CFLAGS) $(abspath $1) -o $2
endef

# Compile C++ source $1 to $2
# as a side-effect, generate a dependency file
#
define COMPILEXX
	@echo "CXX:     $1"
	@mkdir -p $(dir $2)
	$(Q) $(CCACHE) $(CXX) -MD -c $(CXXFLAGS) $(abspath $1) -o $2
endef

# Assemble $1 into $2
#
define ASSEMBLE
	@echo "AS:      $1"
	@mkdir -p $(dir $2)
	$(Q) $(CC) -c $(AFLAGS) $(abspath $1) -o $2
endef

# Produce partially-linked $1 from files in $2
#
define PRELINK
	@echo "PRELINK: $1"
	@mkdir -p $(dir $1)
	$(Q) $(LD) -Ur -Map $1.map -o $1 $2 && $(OBJCOPY) --localize-hidden $1
endef

# Update the archive $1 with the files in $2
#
define ARCHIVE
	@echo "AR:      $2"
	@mkdir -p $(dir $1)
	$(Q) $(AR) $1 $2
endef

# Link the objects in $2 into the binary $1
#
define LINK
	@echo "LINK:    $1"
	@mkdir -p $(dir $1)
	$(Q) $(LD) $(LDFLAGS) -Map $1.map -o $1 --start-group $2 $(LIBS) $(LIBGCC) --end-group
endef

# Convert $1 from a linked object to a raw binary in $2
#
define SYM_TO_BIN
	@echo "BIN:     $2"
	@mkdir -p $(dir $2)
	$(Q) $(OBJCOPY) -O binary $1 $2
endef

# Take the raw binary $1 and make it into an object file $2.
# The symbol $3 points to the beginning of the file, and $3_len
# gives its length.
#
# - compile an empty file to generate a suitable object file
# - relink the object and insert the binary file
# - extract the length
# - create const unsigned $3_len with the extracted length as its value and compile it to an object file
# - link the two generated object files together
# - edit symbol names to suit
#
# NOTE: exercise caution using this with absolute pathnames; it looks
#       like the MinGW tools insert an extra _ in the binary symbol name; e.g.
#	the path:
#
#	/d/px4/firmware/Build/px4fmu_default.build/romfs.img
#
#	is assigned symbols like:
#
#	_binary_d__px4_firmware_Build_px4fmu_default_build_romfs_img_size
#
#	when we would expect
#
#	_binary__d_px4_firmware_Build_px4fmu_default_build_romfs_img_size
#
define BIN_SYM_PREFIX
	_binary_$(subst /,_,$(subst .,_,$1))
endef
define BIN_TO_OBJ
	@echo "OBJ:     $2"
	@mkdir -p $(dir $2)
	$(Q) echo > $2.c
	$(call COMPILE,$2.c,$2.c.o)
	$(Q) $(LD) -r -o $2.bin.o $2.c.o -b binary $1
	$(Q) echo "const unsigned int $3_len = 0x`$(NM) -p --radix=x $2.bin.o | $(GREP) $(call BIN_SYM_PREFIX,$1)_size$$ | $(GREP) -o ^[0-9a-fA-F]*`;" > $2.c
	$(call COMPILE,$2.c,$2.c.o)
	$(Q) $(LD) -r -o $2 $2.c.o $2.bin.o
	$(Q) $(OBJCOPY) $2 \
		--redefine-sym $(call BIN_SYM_PREFIX,$1)_start=$3 \
		--strip-symbol $(call BIN_SYM_PREFIX,$1)_size \
		--strip-symbol $(call BIN_SYM_PREFIX,$1)_end \
		--rename-section .data=.rodata
	$(Q) $(REMOVE) $2.c $2.c.o $2.bin.o
endef
