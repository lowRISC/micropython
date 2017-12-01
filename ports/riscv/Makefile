export PATH=/local/scratch/jrrk2/lowrisc-chip-update-jrrk2/riscv/bin:/usr/bin:/bin
include ../../py/mkenv.mk

# qstr definitions (must come before including py.mk)
QSTR_DEFS = qstrdefsport.h

# include py core make definitions
include $(TOP)/py/py.mk

CROSS_COMPILE = riscv64-unknown-elf-

INC += -I.
INC += -I$(TOP)
INC += -I$(BUILD)

CFLAGS_RISCV = -MMD -MP -Wall -Werror -D__NO_INLINE__ -mcmodel=medany -O2 -std=gnu99 -Wno-unused -Wno-attributes -Wno-error=multichar -fno-delete-null-pointer-checks -march=rv64imafdc -mabi=lp64d
CFLAGS = $(INC) -Wall -Werror -std=c99 $(CFLAGS_RISCV) $(COPT)
LDFLAGS = -T riscv.ld -nostdlib

# Tune for Debugging or Optimization
ifeq ($(DEBUG), 1)
CFLAGS += -O0 -ggdb
else
CFLAGS += -Os -DNDEBUG
CFLAGS += -fdata-sections -ffunction-sections
endif

LIBS =

SRC_C = \
	main.c \
	uart_core.c \
	lib/utils/stdout_helpers.c \
	lib/utils/pyexec.c \
	lib/libc/string0.c \
	lib/mp-readline/readline.c \
	$(BUILD)/_frozen_mpy.c \

OBJ = $(PY_O) $(addprefix $(BUILD)/, $(SRC_C:.c=.o)) setjmp.o

all: $(BUILD)/firmware.a

$(BUILD)/_frozen_mpy.c: frozentest.mpy $(BUILD)/genhdr/qstrdefs.generated.h
	$(ECHO) "MISC freezing bytecode"
	$(Q)$(TOP)/tools/mpy-tool.py -f -q $(BUILD)/genhdr/qstrdefs.preprocessed.h -mlongint-impl=none $< > $@

$(BUILD)/firmware.a: $(OBJ)
	$(ECHO) "LINK $@"
	$(CROSS_COMPILE)ar rc $@ $^ $(LIBS)


test: $(BUILD)/firmware.elf
	$(Q)/bin/echo -e "print('hello world!', list(x+1 for x in range(10)), end='eol\\\\n')\\r\\n\\x04" | $(BUILD)/firmware.elf | tail -n2 | grep "^hello world! \\[1, 2, 3, 4, 5, 6, 7, 8, 9, 10\\]eol"

include $(TOP)/py/mkrules.mk
