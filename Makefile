##############################################################################
#                                                                            #
# ESC - Electronic Speed Controller for R/C cars and robots                  #
#                                                                            #
# by Triffid Hunter                                                          #
#                                                                            #
#                                                                            #
# This firmware is Copyright (C) 2009-2010 Michael Moon aka Triffid_Hunter   #
#                                                                            #
# This program is free software; you can redistribute it and/or modify       #
# it under the terms of the GNU General Public License as published by       #
# the Free Software Foundation; either version 2 of the License, or          #
# (at your option) any later version.                                        #
#                                                                            #
# This program is distributed in the hope that it will be useful,            #
# but WITHOUT ANY WARRANTY; without even the implied warranty of             #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              #
# GNU General Public License for more details.                               #
#                                                                            #
# You should have received a copy of the GNU General Public License          #
# along with this program; if not, write to the Free Software                #
# Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA #
#                                                                            #
##############################################################################

##############################################################################
#                                                                            #
# Change these to suit your hardware                                         #
#                                                                            #
##############################################################################

# MCU_TARGET = atmega168
 MCU_TARGET = atmega328p
# MCU_TARGET = atmega644p
#MCU_TARGET = atmega1280
# MCU_TARGET = atmega2560
# MCU_TARGET = at90usb1287

# CPU clock rate
#F_CPU = 20000000L
#F_CPU = 16000000L
F_CPU = 8000000L

# application baud
APPBAUD = 38400

DEFS = -DF_CPU=$(F_CPU) -DBAUD=$(APPBAUD)

##############################################################################
#                                                                            #
# Programmer settings for "make program"                                     #
#                                                                            #
##############################################################################

AVRDUDE = avrdude
AVRDUDECONF = /etc/avrdude.conf

##############################################################################
#                                                                            #
# udev rule for /dev/arduino (insert into /etc/udev/rules.d/99-local.rules)  #
# SUBSYSTEMS=="usb", ATTRS{idProduct}=="6001", ATTRS{idVendor}=="0403",      #
#     NAME="%k", SYMLINK+="arduino", SYMLINK+="arduino_$attr{serial}",       #
#     MODE="0660"                                                            #
#                                                                            #
##############################################################################

PROGPORT = /dev/arduino_A1001Ncz
PROGBAUD = 9600
# at least mega2560 needs stk500v2
PROGID = avrisp

##############################################################################
#                                                                            #
# These defaults should be ok, change if you need to                         #
#                                                                            #
##############################################################################

PROGRAM = esc

SOURCES = $(wildcard *.c)

ARCH = avr-
CC = $(ARCH)gcc
OBJDUMP = $(ARCH)objdump
OBJCOPY = $(ARCH)objcopy

OPTIMIZE = -Os -ffunction-sections -finline-functions-called-once -mcall-prologues
# OPTIMIZE = -O0
CFLAGS = -g -Wall -Wstrict-prototypes $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS) -std=gnu99 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -save-temps
LDFLAGS = -Wl,--as-needed -Wl,--gc-sections -Wl,-L,/usr/avr/lib/avr5
LIBS = -lm
LIBDEPS =
SUBDIRS =

ifneq (,$(findstring usb,$(MCU_TARGET)))
LDFLAGS += -Llufa_serial
LIBS += -llufa_serial
SUBDIRS += lufa_serial
LIBDEPS += lufa_serial/liblufa_serial.a
else
SOURCES += serial.c
endif

OBJ = $(patsubst %.c,%.o,${SOURCES})

.PHONY: all program clean size subdirs program-fuses doc functionsbysize
.PRECIOUS: %.o %.elf

all: config.h subdirs $(PROGRAM).hex $(PROGRAM).lst $(PROGRAM).sym size

$(PROGRAM).elf: $(LIBDEPS)

subdirs:
	@for dir in $(SUBDIRS); do \
	  $(MAKE) -C $$dir; \
	done

program: $(PROGRAM).hex config.h
	@#stty $(PROGBAUD) raw ignbrk hup < $(PROGPORT)
	@sleep 0.1
	@#@stty $(PROGBAUD) raw ignbrk hup < $(PROGPORT)
	$(AVRDUDE) -c$(PROGID) -b$(PROGBAUD) -p$(MCU_TARGET) -P$(PROGPORT) -C$(AVRDUDECONF) -U flash:w:$<
	@#stty $(APPBAUD) raw ignbrk -hup -echo ixoff < $(PROGPORT)

.lfuse: $(PROJECT).o
	avr-objcopy -j .fuse -I elf32-avr -O binary esc.o /dev/stdout | perl -ne '/(.)(.)(.)/ && printf "%c", ord $$1' > $@

.hfuse: $(PROJECT).o
	avr-objcopy -j .fuse -I elf32-avr -O binary esc.o /dev/stdout | perl -ne '/(.)(.)(.)/ && printf "%c", ord $$2' > $@

.efuse: $(PROJECT).o
	avr-objcopy -j .fuse -I elf32-avr -O binary esc.o /dev/stdout | perl -ne '/(.)(.)(.)/ && printf "%c", ord $$3' > $@

.lock:  $(PROJECT).o
	avr-objcopy -j .lock -I elf32-avr -O binary esc.o /dev/stdout | perl -ne '/(.)/ && printf "%c", ord $$1' > $@

program-fuses: .lfuse .hfuse .efuse .lock
# 	avr-objdump -s -j .fuse $^ | perl -ne '/\s0000\s([0-9a-f]{2})/ && print "$$1\n"' > lfuse
# 	avr-objdump -s -j .fuse $^ | perl -ne '/\s0000\s..([0-9a-f]{2})/ && print "$$1\n"' > hfuse
# 	avr-objdump -s -j .fuse $^ | perl -ne '/\s0000\s....([0-9a-f]{2})/ && print "$$1\n"' > efuse
# 	echo $(AVRDUDE) -c$(PROGID) -b$(PROGBAUD) -p$(MCU_TARGET) -P$(PROGPORT) -C$(AVRDUDECONF) -U lfuse:w:lfuse -U hfuse:w:hfuse -U efuse:w:efuse
# 	echo $(AVRDUDE) -c$(PROGID) -b$(PROGBAUD) -p$(MCU_TARGET) -P$(PROGPORT) -C$(AVRDUDECONF)
# 	echo $(AVRDUDE) -c$(PROGID) -b$(PROGBAUD) -p$(MCU_TARGET) -P$(PROGPORT) -C$(AVRDUDECONF)
	$(AVRDUDE) -c$(PROGID) -b$(PROGBAUD) -p$(MCU_TARGET) -P$(PROGPORT) -C$(AVRDUDECONF) -U lfuse:w:.lfuse -U hfuse:w:.hfuse -U efuse:w:.efuse -U lock:w:.lock

clean: clean-subdirs
	rm -rf *.o *.elf *.lst *.map *.sym *.lss *.eep *.srec *.bin *.hex *.al *.i *.s *~ *fuse

clean-subdirs:
	@for dir in $(SUBDIRS); do \
	  $(MAKE) -C $$dir clean; \
	done

size: $(PROGRAM).elf
	@echo
	@echo $$'            \033[1;4m SIZE        Atmega168        Atmega328p       Atmega644         Atmega1280        Atmega2560 \033[0m'
	@$(OBJDUMP) -h $^ | perl -MPOSIX -ne '/.(text)\s+([0-9a-f]+)/     && do { $$a += eval "0x$$2" }; END { printf "    FLASH : %5d bytes  %2d%% of %2dkb      %2d%% of %2dkb      %2d%% of %2dkb      %2d%% of %3dkb      %2d%% of %3dkb\n", $$a, ceil($$a * 100 / (15 * 1024)), 15, ceil($$a * 100 / (31 * 1024)), 31, ceil($$a * 100 / (63 * 1024)), 63, ceil($$a * 100 / (127 * 1024)), 127, ceil($$a * 100 / (255 * 1024)), 255 }'
	@$(OBJDUMP) -h $^ | perl -MPOSIX -ne '/.(data|bss)\s+([0-9a-f]+)/ && do { $$a += eval "0x$$2" }; END { printf "    RAM   : %5d bytes  %2d%% of %2dkb      %2d%% of %2dkb      %2d%% of %2dkb      %2d%% of %3dkb      %2d%% of %3dkb\n", $$a, ceil($$a * 100 / ( 1 * 1024)),  1, ceil($$a * 100 / ( 2 * 1024)),  2, ceil($$a * 100 / ( 4 * 1024)),  4, ceil($$a * 100 / (  8 * 1024)),   8, ceil($$a * 100 / (  8 * 1024)),   8 }'
	@$(OBJDUMP) -h $^ | perl -MPOSIX -ne '/.(eeprom)\s+([0-9a-f]+)/   && do { $$a += eval "0x$$2" }; END { printf "    EEPROM: %5d bytes  %2d%% of %2dkb      %2d%% of %2dkb      %2d%% of %2dkb      %2d%% of %3dkb      %2d%% of %3dkb\n", $$a, ceil($$a * 100 / ( 1 * 1024)),  1, ceil($$a * 100 / ( 2 * 1024)),  2, ceil($$a * 100 / ( 2 * 1024)),  2, ceil($$a * 100 / (  4 * 1024)),   4, ceil($$a * 100 / (  4 * 1024)),   4 }'

doc: Doxyfile *.c *.h
	doxygen $<

functionsbysize: $(OBJ)
	@avr-objdump -h $^ | grep '\.text\.' | perl -ne '/\.text\.(\S+)\s+([0-9a-f]+)/ && printf "%u\t%s\n", eval("0x$$2"), $$1;' | sort -n

%.o: %.c config.h Makefile
	@echo "  CC        $@"
	@$(CC) -c $(CFLAGS) -Wa,-adhlns=$(<:.c=.al) -o $@ $(subst .o,.c,$@)

%.elf: $(OBJ)
	@echo "  LINK      $@"
	@$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

%.lst: %.elf
	@echo "  OBJDUMP   $@"
	@$(OBJDUMP) -h -S $< > $@

%.hex: %.elf
	@echo "  OBJCOPY   $@"
	@$(OBJCOPY) -j .text -j .data -O ihex -R .eeprom -R .fuse -R .lock $< $@

%.bin: %.elf
	@echo "  OBJCOPY   $@"
	@$(OBJCOPY) -j .text -j .data -O binary $< $@

%.sym: %.elf
	@echo "  SYM       $@"
	@$(OBJDUMP) -t $< | perl -ne 'BEGIN { printf "  ADDR  NAME                  SIZE\n"; } /([0-9a-f]+)\s+(\w+)\s+O\s+\.(bss|data)\s+([0-9a-f]+)\s+(\w+)/ && printf "0x%04x  %-20s +%d\n", eval("0x$$1") & 0xFFFF, $$5, eval("0x$$4")' | sort -k1 > $@
