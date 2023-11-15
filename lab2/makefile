SRCS = $(wildcard *.s)
OBJS = $(patsubst %.s,$(OBJDIR)/%.o,$(SRCS))
ELFS = $(patsubst %.s,$(OBJDIR)/%.elf,$(SRCS))
BINS = $(patsubst %.s,$(OBJDIR)/%.bin,$(SRCS))
LSTS = $(patsubst %.s,$(OBJDIR)/%.lst,$(SRCS))

LINKER_SCRIPT = stm32.ld
OBJDIR = Debug

CFLAGS  = -mcpu=cortex-m0plus -mthumb
CFLAGS += -mfloat-abi=soft
CFLAGS += -g3

LDFLAGS = -mcpu=cortex-m0plus -mthumb
LDFLAGS += -mfloat-abi=soft
LDFLAGS += -nostdlib
LDFLAGS += -T$(LINKER_SCRIPT)

CROSS_COMPILE = arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJDUMP = $(CROSS_COMPILE)objdump
OBJCOPY = $(CROSS_COMPILE)objcopy
SIZE = $(CROSS_COMPILE)size

all: clean build size

build: $(BINS) $(LSTS)

$(OBJDIR)/%.o: %.s
	@mkdir -p $(OBJDIR)
	@echo "Compiling" $<
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR)/%.elf: $(OBJDIR)/%.o
	@echo "Linking" $@
	$(CC) $< $(LDFLAGS) -o $@

$(OBJDIR)/%.bin: $(OBJDIR)/%.elf
	@echo "Creating binary" $@
	@$(OBJCOPY) -O binary $< $@

$(OBJDIR)/%.lst: $(OBJDIR)/%.elf
	@echo "Creating list file" $@
	@$(OBJDUMP) -x -S $< > $@

size: $(ELFS)
	@echo "Displaying size"
	@$(SIZE) $^

clean:
	@echo "Cleaning up..."
	@rm -rf $(OBJDIR)/

.PHONY: all build size clean
