CC = msp430-elf-gcc
CPU = msp430fr5739
CFLAGS = -mmcu=$(CPU) -O2 -g

all: main.hex

main.elf: AgNP04b-main.c
	$(CC) $(CFLAGS) -o main.elf AgNP04b-main.c

main.hex: main.elf
	msp430-elf-objcopy -O ihex main.elf main.hex