* Not performing 8 bit reads properly. No status register from 16550 UART
* Swapping the endian of the WB SEL lines from the RISCV core allow 8 bit writes to the 16550
    * Must not be swapped from RAM (need to maintain correct word byte ordering on reads after writing a byte)
* Updating wb_intercon based on patch from https://github.com/Oxore/fusesoc-demos/blob/master/cores/wb_intercon.patch

These lines need to be removed from the `VexRiscv` core, as they cause all SEL bits to be set on byte reads
```
    if((! dBus_cmd_halfPipe_payload_wr))begin
      dBusWishbone_SEL = (4'b1111);
    end
```

TODO: Look into how SpinalHDL generates this and fix it
