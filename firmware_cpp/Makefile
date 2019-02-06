
SUBDIRS = drivers
PROJ = proj

### Machine flags
#
CC_CMACH	= -mmcu=msp430f5510
CC_DMACH	= -D__MSP430F5510__
### Build flags
#
# -fdata-sections, -ffunction-sections and -Wl,--gc-sections -Wl,-s
# are used for dead code elimination, see:
# http://gcc.gnu.org/ml/gcc-help/2003-08/msg00128.html
#
CFLAGS		+= $(CC_CMACH) $(CC_DMACH) -Wall -Wno-switch
CFLAGS		+= -fno-force-addr -finline-limit=1 -fno-schedule-insns
CFLAGS      += -fno-exceptions -fno-rtti -fno-stack-protector -fno-math-errno -ffast-math
CFLAGS		+= -fshort-enums -Wl,-Map=output.map
LDFLAGS		= -lm

CFLAGS_REL	+= -Os -s -fdata-sections -ffunction-sections -fomit-frame-pointer
LDFLAGS_REL	+= -Wl,--gc-sections -Wl,-s -dead_strip -Wl,--strip-all

CFLAGS_DBG	+= -O1 -g
LDFLAGS_DBG	+= -Wl,--gc-sections

# linker flags and include directories
INCLUDES	+= -I./ -Igcc/ -Idrivers/ -Ilibraries/ISR_safe_memory/ -Ilibraries/UHS_host/ -Ilibraries/UHS_ByteBuffer/
### Build tools
# 
CC		= msp430-elf-gcc
CXX		= msp430-elf-g++
LD		= msp430-elf-ld
AS		= msp430-elf-as
AR		= msp430-elf-ar

BASH := $(shell which bash || which bash)

.PHONY: all
.PHONY: clean
.PHONY: install
.PHONY: config
.PHONY: depend
.PHONY: doc
.PHONY: tags
.PHONY: force

all: depend config.h tags $(PROJ).elf

#
# Build list of sources and objects to build
SRCS := $(wildcard *.c)
$(foreach subdir,$(SUBDIRS), \
	$(eval SRCS := $(SRCS) $(wildcard $(subdir)/*.c)) \
)
SRCSPP := $(wildcard *.cpp)
$(foreach subdir,$(SUBDIRS), \
	$(eval SRCS := $(SRCS) $(wildcard $(subdir)/*.cpp)) \
)
OBJS := $(patsubst %.c,%.o,$(SRCS)) $(patsubst %.cpp,%.o,$(SRCSPP))

#
# Dependencies rules
depend: $(PROJ).dep

$(PROJ).dep: $(SRCS)
	@echo "Generating dependencies.."
	@touch $@
	@makedepend $(INCLUDES) -Y -f $@ $^ &> /dev/null
	@rm -f $@.bak

#
# Append specific CFLAGS/LDFLAGS
DEBUG := $(shell grep "^\#define CONFIG_DEBUG" config.h)
ifeq ($(DEBUG),)
TARGET	:= RELEASE
CFLAGS	+= $(CFLAGS_REL)
LDFLAGS	+= $(LDFLAGS_REL)
else
TARGET	:= DEBUG
CFLAGS	+= $(CFLAGS_DBG)
LDFLAGS	+= $(LDFLAGS_DBG)
endif

# rebuild if CFLAGS changed, as suggested in:
# http://stackoverflow.com/questions/3236145/force-gnu-make-to-rebuild-objects-affected-by-compiler-definition/3237349#3237349
$(PROJ).cflags: force
	@echo "$(CFLAGS)" | cmp -s - $@ || echo "$(CFLAGS)" > $@

$(OBJS): $(PROJ).cflags
#
# Top rules

$(PROJ).elf: $(OBJS)
	@echo -e "\n>> Building $@ as target $(TARGET)"
	$(CXX) $(CFLAGS) $(LDFLAGS) $(INCLUDES) -o $@ $+ && size $@ && bash tools/setver.sh
	@#@python tools/msp430-ram-usage $@

modinit.o: modinit.c
	@echo "CXX $<"
	@$(CXX) $(CFLAGS) -Wno-implicit-function-declaration \
		$(INCLUDES) -c $< -o $@

%.o: %.c
	@echo "CXX $<"
	@$(CXX) $(CFLAGS) $(INCLUDES) -c $< -o $@

%.o: %.cpp
	@echo "CXX $<"
	@$(CXX) $(CFLAGS) $(INCLUDES) -c $< -o $@

modinit.c:
	@echo "Please do a 'make config' first!" && false

config.h:
	@echo "Please do a 'make config' first!" && false

tags: $(SRCS)
	@echo "Generating tags .."
	@exuberant-ctags -R

install: $(PROJ).elf
	@bash ~/bin/burn.sh
	@#mspdebug olimex-iso-mk2 "prog $<"

clean: $(SUBDIRS)
	@for subdir in $(SUBDIRS); do \
		echo "Cleaning $$subdir .."; rm -f $$subdir/*.o; \
	done
	@rm -f *.o $(PROJ).{elf,txt,cflags,dep} output.map

doc:
	rm -rf doc/*
	doxygen Doxyfile
	
-include $(PROJ).dep
