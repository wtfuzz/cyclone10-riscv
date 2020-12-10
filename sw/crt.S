#define GPIO_BASE 0x91000000
#define UART_BASE 0x90000000

.global crtStart
.global main
.global trap_callback

.section	.start_jump,"ax",@progbits

crtStart:
  lui a2, %hi(GPIO_BASE)
  addi a2, a2, %lo(GPIO_BASE)

  // Make pins outputs
  li t0, 0xff
  sb t0, 0x4(a2)

  // Invert output pins
  sb t0, 0x14(a2)

  // Turn one LED on
  li t0, 0x1
  sb t0, 0x0(a2)

  //long jump to allow crtInit to be anywhere
  //do it always in 12 bytes
  lui x2,       %hi(crtInit)
  addi x2, x2,  %lo(crtInit)
  jalr x1,x2
  nop

.section .text

.global  trap_entry
.align 8
trap_entry:
  sw x1,  - 1*4(sp)
  sw x5,  - 2*4(sp)
  sw x6,  - 3*4(sp)
  sw x7,  - 4*4(sp)
  sw x10, - 5*4(sp)
  sw x11, - 6*4(sp)
  sw x12, - 7*4(sp)
  sw x13, - 8*4(sp)
  sw x14, - 9*4(sp)
  sw x15, -10*4(sp)
  sw x16, -11*4(sp)
  sw x17, -12*4(sp)
  sw x28, -13*4(sp)
  sw x29, -14*4(sp)
  sw x30, -15*4(sp)
  sw x31, -16*4(sp)
  addi sp,sp,-16*4
  call trap_callback
  lw x1 , 15*4(sp)
  lw x5,  14*4(sp)
  lw x6,  13*4(sp)
  lw x7,  12*4(sp)
  lw x10, 11*4(sp)
  lw x11, 10*4(sp)
  lw x12,  9*4(sp)
  lw x13,  8*4(sp)
  lw x14,  7*4(sp)
  lw x15,  6*4(sp)
  lw x16,  5*4(sp)
  lw x17,  4*4(sp)
  lw x28,  3*4(sp)
  lw x29,  2*4(sp)
  lw x30,  1*4(sp)
  lw x31,  0*4(sp)
  addi sp,sp,16*4
  mret
  .text


crtInit:
  .option push
  .option norelax

  la gp, __global_pointer$
  .option pop
  la sp, _stack_start
  
  lui a2, %hi(GPIO_BASE)
  addi a2, a2, %lo(GPIO_BASE)

  li t0, 0x2
  sb t0, 0x0(a2)


bss_init:
  la a0, _bss_start
  la a1, _bss_end
bss_loop:
  beq a0,a1,bss_done
  sw zero,0(a0)
  add a0,a0,4
  j bss_loop
bss_done:
  li t0, 0x3
  sb t0, 0x0(a2)

ctors_init:
  la a0, _ctors_start
  addi sp,sp,-4
ctors_loop:
  la a1, _ctors_end
  beq a0,a1,ctors_done
  lw a3,0(a0)
  add a0,a0,4
  sw a0,0(sp)
  jalr  a3
  lw a0,0(sp)
  j ctors_loop
ctors_done:
  li t0, 0x4
  sb t0, 0x0(a2)
  addi sp,sp,4

  /*
  li a0, 0x880     //880 enable timer + external interrupts
  csrw mie,a0
  li a0, 0x1808     //1808 enable interrupts
  csrw mstatus,a0
  */

  li t0, 0x5
  sb t0, 0x0(a2)

init_uart:
  /* Initialize the UART */
  lui a1, %hi(UART_BASE)
  addi a1, a1, %lo(UART_BASE)

  /* Enable divisor latch */
  li t0, 0x83
  sb t0, 3(a1)

  // Write divisors
  li t0, 0 
  sb t0, 1(a1)
  li t0, 27
  sb t0, 0(a1)

  // 8 bits per character, no parity, 1 stop bit
  li t0, 0x3
  sb t0, 3(a1)

  li t2, '.'
  sb t2, 0(a1)

  call main

infinitLoop:
  j infinitLoop