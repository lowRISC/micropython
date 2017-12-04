# Enable/disable modules and 3rd-party libs to be included in interpreter

# Build 32-bit binaries on a 64-bit host
MICROPY_FORCE_32BIT = 0

# This variable can take the following values:
#  0 - no readline, just simple stdin input
#  1 - use MicroPython version of readline
MICROPY_USE_READLINE = 1

# btree module using Berkeley DB 1.xx
MICROPY_PY_BTREE = 0

# _thread module using pthreads
MICROPY_PY_THREAD = 0

# Subset of CPython termios module
MICROPY_PY_TERMIOS = 0

# Subset of CPython socket module
MICROPY_PY_SOCKET = 0

# ffi module requires libffi (libffi-dev Debian package)
MICROPY_PY_FFI = 0

# ussl module requires one of the TLS libraries below
MICROPY_PY_USSL = 0
# axTLS has minimal size and fully integrated with MicroPython, but
# implements only a subset of modern TLS functionality, so may have
# problems with some servers.
MICROPY_SSL_AXTLS = 0
# mbedTLS is more up to date and complete implementation, but also
# more bloated. Configuring and building of mbedTLS should be done
# outside of MicroPython, it can just link with mbedTLS library.
MICROPY_SSL_MBEDTLS = 0

# jni module requires JVM/JNI
MICROPY_PY_JNI = 0

# Avoid using system libraries, use copies bundled with MicroPython
# as submodules (currently affects only libffi).
MICROPY_STANDALONE = 0

include ../../py/mkenv.mk

ifneq ($(CROSS_COMPILE),)
CC = $(CROSS_COMPILE)gcc -MMD -MP -Wall -Werror -D__NO_INLINE__ -mcmodel=medany -O2 -std=gnu99 -Wno-unused -Wno-attributes -Wno-error=multichar -fno-delete-null-pointer-checks  -march=rv64imafdc -mabi=lp64d
AR = $(CROSS_COMPILE)ar rc micropython.a
LDFLAGS_ARCH =
OBJ = $(PY_O) setjmp.o
else
CC = gcc
LD = gcc
LDFLAGS_ARCH = -Wl,-Map=$@.map,--cref -Wl,--gc-sections
OBJ = $(PY_O)
endif

# qstr definitions (must come before including py.mk)
QSTR_DEFS = qstrdefsport.h

# OS name, for simple autoconfig
UNAME_S := $(shell uname -s)

# include py core make definitions
include $(TOP)/py/py.mk

INC +=  -I.
INC +=  -I$(TOP)
INC += -I$(BUILD)

# compiler settings
CWARN = -Wall -Werror
CWARN += -Wpointer-arith -Wuninitialized
CFLAGS = $(INC) $(CWARN) -std=gnu99 -DUNIX $(CFLAGS_MOD) $(COPT) $(CFLAGS_EXTRA) -Dclose=_close_ -Dclosedir=_closedir_ -D_exit=__exit_ -Dfsync=_fsync_ -Dgetenv=_getenv_ -Dgettimeofday=_gettimeofday_ -Disatty=_isatty_ -Dlseek=_lseek_ -Dmkdir=_mkdir_ -Dopen=_open_ -Dopendir=_opendir_ -Dread=_read_ -Dreaddir=_readdir_ -Drealpath=_realpath_ -Dsbrk=_sbrk_ -Dsigemptyset=_sigemptyset_ -Dsignal=_signal_ -Dstat=_stat_ -Dstrerror=_strerror_ -Dsystem=_system_ -Dtcgetattr=_tcgetattr_ -Dtcsetattr=_tcsetattr_ -Dunlink=_unlink_ -Dwrite=_write_ -Dmalloc=_malloc_ -Drealloc=_realloc_ -Dfree=_free_ -Dstrchr=_strchr_


# Debugging/Optimization
ifdef DEBUG
CFLAGS += -g
COPT = -O0
else
COPT = -Os -fdata-sections -ffunction-sections -DNDEBUG
# _FORTIFY_SOURCE is a feature in gcc/glibc which is intended to provide extra
# security for detecting buffer overflows. Some distros (Ubuntu at the very least)
# have it enabled by default.
#
# gcc already optimizes some printf calls to call puts and/or putchar. When
# _FORTIFY_SOURCE is enabled and compiling with -O1 or greater, then some
# printf calls will also be optimized to call __printf_chk (in glibc). Any
# printfs which get redirected to __printf_chk are then no longer synchronized
# with printfs that go through mp_printf.
#
# In MicroPython, we don't want to use the runtime library's printf but rather
# go through mp_printf, so that stdout is properly tied into streams, etc.
# This means that we either need to turn off _FORTIFY_SOURCE or provide our
# own implementation of __printf_chk. We've chosen to turn off _FORTIFY_SOURCE.
# It should also be noted that the use of printf in MicroPython is typically
# quite limited anyways (primarily for debug and some error reporting, etc
# in the unix version).
#
# Information about _FORTIFY_SOURCE seems to be rather scarce. The best I could
# find was this: https://securityblog.redhat.com/2014/03/26/fortify-and-you/
# Original patchset was introduced by
# https://gcc.gnu.org/ml/gcc-patches/2004-09/msg02055.html .
#
# Turning off _FORTIFY_SOURCE is only required when compiling with -O1 or greater
CFLAGS += -U _FORTIFY_SOURCE
endif

# Use gcc syntax for map file
LDFLAGS = $(LDFLAGS_MOD) $(LDFLAGS_ARCH) $(LDFLAGS_EXTRA) # -lm

ifeq ($(MICROPY_FORCE_32BIT),1)
# Note: you may need to install i386 versions of dependency packages,
# starting with linux-libc-dev:i386
ifeq ($(MICROPY_PY_FFI),1)
ifeq ($(UNAME_S),Linux)
CFLAGS_MOD += -I/usr/include/i686-linux-gnu
endif
endif
endif

ifeq ($(MICROPY_USE_READLINE),1)
INC +=  -I$(TOP)/lib/mp-readline
CFLAGS_MOD += -DMICROPY_USE_READLINE=1
LIB_SRC_C_EXTRA += mp-readline/readline.c
endif
ifeq ($(MICROPY_PY_TERMIOS),1)
CFLAGS_MOD += -DMICROPY_PY_TERMIOS=1
SRC_MOD += modtermios.c
endif
ifeq ($(MICROPY_PY_SOCKET),1)
CFLAGS_MOD += -DMICROPY_PY_SOCKET=1
SRC_MOD += modusocket.c
endif
ifeq ($(MICROPY_PY_THREAD),1)
CFLAGS_MOD += -DMICROPY_PY_THREAD=1 -DMICROPY_PY_THREAD_GIL=0
LDFLAGS_MOD += -lpthread
endif

ifeq ($(MICROPY_PY_FFI),1)

ifeq ($(MICROPY_STANDALONE),1)
LIBFFI_CFLAGS_MOD := -I$(shell ls -1d $(TOP)/lib/libffi/build_dir/out/lib/libffi-*/include)
 ifeq ($(MICROPY_FORCE_32BIT),1)
  LIBFFI_LDFLAGS_MOD = $(TOP)/lib/libffi/build_dir/out/lib32/libffi.a
 else
  LIBFFI_LDFLAGS_MOD = $(TOP)/lib/libffi/build_dir/out/lib/libffi.a
 endif
else
LIBFFI_CFLAGS_MOD := $(shell pkg-config --cflags libffi)
LIBFFI_LDFLAGS_MOD := $(shell pkg-config --libs libffi)
endif

ifeq ($(UNAME_S),Linux)
LIBFFI_LDFLAGS_MOD += -ldl
endif

CFLAGS_MOD += $(LIBFFI_CFLAGS_MOD) -DMICROPY_PY_FFI=1
LDFLAGS_MOD += $(LIBFFI_LDFLAGS_MOD)
SRC_MOD += modffi.c
endif

ifeq ($(MICROPY_PY_JNI),1)
# Path for 64-bit OpenJDK, should be adjusted for other JDKs
CFLAGS_MOD += -I/usr/lib/jvm/java-7-openjdk-amd64/include -DMICROPY_PY_JNI=1
SRC_MOD += modjni.c
endif

# source files
SRC_C = \
	main.c \
	malloc.c \
	stubs.c \
	memcmp.c \
	memcpy.c \
	memset.c \
	memmove.c \
	wordcopy.c \
	strchr.c \
	strrchr.c \
	strcmp.c \
	strcpy.c \
	strncmp.c \
	strlen.c \
	unix_mphal.c \
	input.c \
	modos.c \
	modmachine.c \
	moduos_vfs.c \
	fatfs_port.c \
	sd_diskio.c \
	spi.c \
	$(SRC_MOD)

LIB_SRC_C = $(addprefix lib/,\
	$(LIB_SRC_C_EXTRA) \
	timeutils/timeutils.c \
	)

# FatFS VFS support
LIB_SRC_C += $(addprefix lib/,\
	oofatfs/ff.c \
	oofatfs/option/unicode.c \
	)

OBJ += $(addprefix $(BUILD)/, $(SRC_C:.c=.o))
OBJ += $(addprefix $(BUILD)/, $(LIB_SRC_C:.c=.o))
OBJ += $(addprefix $(BUILD)/, $(STMHAL_SRC_C:.c=.o))

# List of sources for qstr extraction
SRC_QSTR += $(SRC_C) $(LIB_SRC_C)
# Append any auto-generated sources that are needed by sources listed in
# SRC_QSTR
SRC_QSTR_AUTO_DEPS +=

all: micropython

include $(TOP)/py/mkrules.mk

ifneq ($(CROSS_COMPILE),)
micropython: $(OBJ)
	$(ECHO) "LINK $@"
	$(Q)$(AR) $^ $(LIB) $(LDFLAGS)
else
micropython: $(OBJ)
	$(ECHO) "LINK $@"
	$(Q)$(LD) -o $@ $^ $(LIB) $(LDFLAGS)
endif

# Value of configure's --host= option (required for cross-compilation).
# Deduce it from CROSS_COMPILE by default, but can be overridden.
ifneq ($(CROSS_COMPILE),)
CROSS_COMPILE_HOST = --host=$(patsubst %-,%,$(CROSS_COMPILE))
else
CROSS_COMPILE_HOST =
endif

