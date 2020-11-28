BOOTROM_ADDR ?= 0xf0000000

TARGET ?= boot
all: $(TARGET).vh

boot.elf: boot.S
	riscv32-unknown-elf-gcc -march=rv32i -nostdlib $< -o $@

%.vh: %.bin
	python ./wb_rom_gen.py $< > $@

%.bin: %.elf
	riscv32-unknown-elf-objcopy -O binary $< $@

%.elf: %.S
	riscv32-unknown-elf-gcc -nostdlib $< -o $@

clean:
	rm *.elf $(TARGET).vh

