PRG            = smartupsaddon
OBJ            = fan.o main.o serial-base.o serial.o temp.o lib-debug.o lib-i2c.o lib-tc74.o lib-timekeeping.o
MCU_TARGET     = atmega1284
OPTIMIZE       = -O2
CSTD           = gnu11
DEFS           = -DF_CPU=7372800
LIBS           =

CC             = avr-gcc
OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump

CFLAGS_STD     = -std=$(CSTD) -pipe -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS)
LDFLAGS        = -Wl,-Map,$(PRG).map

DEPDIR         = .dep
$(shell mkdir -p $(DEPDIR) >/dev/null)
DEPFLAGS = -MT $@ -MD -MP -MF $(DEPDIR)/$(basename $@).Td
DEP_POSTCOMPILE = @mv -f $(DEPDIR)/$(basename $@).Td $(DEPDIR)/$(basename $@).d

all: $(PRG).elf lst text eeprom

$(PRG).elf: $(OBJ)
	$(CC) $(CFLAGS_STD) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

lib-%.o : ../lib/%.c $(DEPDIR)/lib-%.d Makefile
	$(CC) $(CPPFLAGS) $(CFLAGS_STD) $(CFLAGS) -c $(DEPFLAGS) -o $@ $<
	$(DEP_POSTCOMPILE)

%.o : %.c
%.o : %.c $(DEPDIR)/%.d Makefile
	$(CC) $(CPPFLAGS) $(CFLAGS_STD) $(CFLAGS) -c $(DEPFLAGS) -o $@ $<
	$(DEP_POSTCOMPILE)

$(DEPDIR)/%.d: ;
.PRECIOUS: $(DEPDIR)/%.d

include $(wildcard $(patsubst %,$(DEPDIR)/%.d,$(basename $(OBJ))))

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

# Rules for building the .text rom images

text: hex bin srec

hex:  $(PRG).hex
bin:  $(PRG).bin
srec: $(PRG).srec

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

# Rules for building the .eeprom rom images

eeprom: ehex ebin esrec

ehex:  $(PRG)_eeprom.hex
ebin:  $(PRG)_eeprom.bin
esrec: $(PRG)_eeprom.srec

%_eeprom.hex: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@ \
	|| { echo empty $@ not generated; exit 0; }

%_eeprom.srec: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O srec $< $@ \
	|| { echo empty $@ not generated; exit 0; }

%_eeprom.bin: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O binary $< $@ \
	|| { echo empty $@ not generated; exit 0; }
