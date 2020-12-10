TARGET ?= boot

CFLAGS=-Os -Wall -march=rv32im -nostartfiles -mstrict-align -DPRINTF_DISABLE_SUPPORT_FLOAT -DPRINTF_DISABLE_SUPPORT_EXPONENTIAL
LDSCRIPT = linker_rom.ld
LDFLAGS += -nostartfiles -ffreestanding -Wl,-Bstatic,-T,$(LDSCRIPT),-Map,rom.map,--print-memory-usage

all: $(TARGET).vh $(TARGET).bin

boot.elf: crt.o rom.o tinyprintf.o
	riscv32-unknown-elf-gcc $(CFLAGS) $^ -o $@ $(LDFLAGS)
	#riscv32-unknown-elf-strip -s -R .comment -R .gnu.version --strip-unneeded $@
	riscv32-unknown-elf-size $@

%.vh: %.bin
	python ./wb_rom_gen.py $< > $@
	#riscv32-unknown-elf-objcopy $< -O verilog $@

%.bin: %.elf
	riscv32-unknown-elf-objcopy -O binary $< $@

%.o: %.c
	riscv32-unknown-elf-gcc $(CFLAGS) -c $< -o $@

%.o: %.S
	riscv32-unknown-elf-gcc $(CFLAGS) -c $< -o $@

clean:
	rm -f *.o *.bin *.elf $(TARGET).vh

