from struct import unpack
import sys
infile = sys.argv[1]
offset = 64

outfile = sys.stdout

BLOCK = """@00000000
{rom_contents}	 
"""

with open(infile, "rb") as f:
    b = f.read(4)
    addr = 0
    rom_contents = ""
    while b:
        rom_contents += "{data:02X}\n".format(data=b[3])
        rom_contents += "{data:02X}\n".format(data=b[2])
        rom_contents += "{data:02X}\n".format(data=b[1])
        rom_contents += "{data:02X}\n".format(data=b[0])
        b = f.read(4)
        addr += 1
    outfile.write(BLOCK.format(rom_contents=rom_contents))
