#!/bin/sh

riscv32-unknown-elf-gcc -march=rv32i -c blink.S
riscv32-unknown-elf-objcopy -O binary blink.o blink.bin
python ./wb_rom_gen.py blink.bin
