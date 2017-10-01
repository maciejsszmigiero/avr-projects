ifndef SRCDIR
$(error Please give bootloader source dir as SRCDIR= parameter to make)
endif

DESTDIR=$(patsubst %/,%,$(dir $(realpath $(firstword $(MAKEFILE_LIST)))))

.PHONY: default
default: all

%: force
	@$(MAKE) -C "$(SRCDIR)" DESTDIR="$(DESTDIR)" MCU=atmega1284 F_CPU=7372800 BOOTLOADER_ADDRESS=0x01F800 CONFIG_FILE="$(DESTDIR)/config.h" $@

force: ;
