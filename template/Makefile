TARGET = template
OPT = -O2
CPP_STANDARD = -std=gnu++17

CPP_SOURCES = \
    main.cpp \
    grain_player.cpp \
    tape_echo.cpp \
    motor.cpp \
    tape_format.cpp \
    record_head.cpp \
    diag.cpp \
    cpu_test.cpp \
    controls.cpp \
    itcm.cpp \
    ../touch/knobs.cpp \
    ../touch/pads.cpp \
    ../touch/switches.cpp

C_INCLUDES = \
    -I. \
    -I.. \
    -I../touch

LIBDAISY_DIR ?= ../lib/libDaisy
DAISYSP_DIR ?= ../lib/DaisySP

SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core

# Test build (like the core Makefile's `ifdef DEBUG`): make PAD_TEST=1 ...
ifdef PAD_TEST
C_DEFS += -DPAD_TEST
endif

# Hot audio code in ITCM: libDaisy's flash linker script + the .itcm_text section.
LDSCRIPT = texture_machine_flash.lds
include $(SYSTEM_FILES_DIR)/Makefile

libs:
	$(MAKE) -C $(LIBDAISY_DIR) -j4
	$(MAKE) -C $(DAISYSP_DIR) -j4

clean-libs:
	$(MAKE) -C $(LIBDAISY_DIR) clean
	$(MAKE) -C $(DAISYSP_DIR) clean
