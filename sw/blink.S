#define GPIO_BASE 0x91000000
#define UART_BASE 0x90000000

#define ON_DELAY 0x10ffff
#define OFF_DELAY 0x20ffff

.section  .start_jump,"ax",@progbits

start:
  lui a2, %hi(GPIO_BASE)
  addi a2, a2, %lo(GPIO_BASE)

  // Make pins outputs
  li t0, 0xff
  sb t0, 0x4(a2)

loop:
  // Turn outputs on
  li t0, 0xff
  sb t0, 0x0(a2)

  lui t0, %hi(ON_DELAY)
  addi t0, t0, %lo(ON_DELAY)
  li t1, 1
delay1:
  sub t0, t0, t1
  bne t0, zero, delay1

  // Turn outputs off
  li t0, 0x00
  sb t0, 0x0(a2)

  lui t0, %hi(OFF_DELAY)
  addi t0, t0, %lo(OFF_DELAY)
  li t1, 1
delay2:
  sub t0, t0, t1
  bne t0, zero, delay2

  j loop
