DEVICE=attiny85
AVRDUDE = avrdude -c usbtiny -p $(DEVICE)

CC = avr-gcc
CFLAGS = -Wall -Os -I. -Iusbdrv -mmcu=$(DEVICE) -DF_CPU=16500000 -DDEBUG_LEVEL=0
LDFLAGS = -Os -mmcu=$(DEVICE)
# NEVER compile the final product with debugging! Any debug output will
# distort timing so that the specs can't be met.


# symbolic targets:
all:	main.hex

%.o:	%.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o:	%.S
	$(CC) $(CFLAGS) -x assembler-with-cpp -c $< -o $@

%.o:	%.s
	$(CC) $(CFLAGS) -S $< -o $@

%.bin:	%.o 
	$(CC) $(LDFLAGS) -o $@ $^

%.hex:	%.bin
	avr-objcopy -j .text -j .data -O ihex $< $@
	./checksize $< 8192 512
# do the checksize script as our last action to allow successful compilation
# on Windows with WinAVR where the Unix commands will fail.

flash: main.hex
	$(AVRDUDE) -U flash:w:$<:i

# FUSES ARE DEFAULT HIGH, SET LOW
fuse:
	$(AVRDUDE) -U hfuse:w:0xdd:m -U lfuse:w:0xc1:m

clean:
	rm -f *.hex *.bin *.o usbdrv/*.o

main.bin: usbdrv/usbdrv.o usbdrv/usbdrvasm.o usbdrv/oddebug.o

