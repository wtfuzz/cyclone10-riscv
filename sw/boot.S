#define GPIO_BASE 0x91000000
#define DELAY 0x100000

#define UART_BASE 0x90000000

.section .text
.global _start

_start:
  /* Load GPIO base address to a0 */
	lui a0, %hi(GPIO_BASE)
	addi a0, a0, %lo(GPIO_BASE)

  /** Set GPIO to output */
  li t0, 0xff
  sb t0, 4(a0)

  /** Invert GPIO data */
  sb t0, 0x14(a0) 

  /** Turn LEDs on */
  li t0, 0x03
  sb t0, 0(a0)

  /* Initialize the UART */
  lui a1, %hi(UART_BASE)
  addi a1, a1, %lo(UART_BASE)

init_uart:

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

  li t2, 'A'
  sb t2, 0(a1)

  li t1, DELAY

  li t3, 0

bl1:
	/* Write to LEDs */
	sb t3, 0(a0)
  addi t3, t3, 1

  /** Write 'A' to UART */
  li t2, 'A'
  sb t2, 0(a1)

  /*
	xori t0, t0,1
  ori t0, t0, 2
  */

	/* Reset timer */
	and t2, zero, zero

	/* Delay loop */
time1:
	addi t2, t2, 1
	bne t1, t2, time1
	j bl1 