# Cyclone 10 FuseSoC RISC-V SoC

* WIP Development on `Intel Cyclone 10 LP Evaluation Kit`.
* The included ROM writes a counter to the 4 LEDs and echos 'A' to the UART.
* JTAG of the VexRiscv core via Virtual TAP is not working yet.
* Uses coreport <https://github.com/wtfuzz/coreport> GPIO peripheral


## RV32I Toolchain

```
sudo apt-get install autoconf automake autotools-dev curl python3 libmpc-dev libmpfr-dev libgmp-dev gawk build-essential bison flex texinfo gperf libtool patchutils bc zlib1g-dev libexpat-dev
```

```
sudo mkdir -p /opt/riscv32i
sudo chown $USER:$USER /opt/riscv32i

git clone --recursive https://github.com/riscv/riscv-gnu-toolchain

cd riscv-gnu-toolchain && mkdir build && cd build
../configure --with-arch=rv32i --prefix=/opt/riscv32i
make -j16 && make install
```

## VexRiscv Core

```
echo "deb https://dl.bintray.com/sbt/debian /" | sudo tee -a /etc/apt/sources.list.d/sbt.list
curl -sL "https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x2EE0EA64E40A89B84B2DF73499E82A75642AC823" | sudo apt-key add
sudo apt-get update
sudo apt-get install sbt
```

```
git clone https://github.com/SpinalHDL/VexRiscv.git
cd VexRiscv
sbt "runMain vexriscv.demo.GenFuseSoc"

# Copy generated VexRiscv Verilog to core
cp VexRiscvWishbone.v ../rtl/verilog/
```

## JTAG Debugger

```
sudo mkdir /opt/openocd
sudo chown $USER:$USER /opt/openocd

git clone https://github.com/SpinalHDL/openocd_riscv.git
cd openocd_riscv && ./bootstrap && ./configure --prefix=/opt/openocd
make -j12 && make install
```
