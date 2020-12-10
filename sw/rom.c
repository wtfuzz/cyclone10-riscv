#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include "tinyprintf.h"
#include "riscv.h"

typedef struct _evt_t
{
    uint8_t set;
    uint16_t mod;
    void (*callback)(void *data);
    void *data;
} evt_t;

#define MAX_TIMERS 8
static evt_t timers[MAX_TIMERS];

#define MCAUSE_INT_MASK     (0x80000000)
#define MCAUSE_CODE_MASK    (0x7FFFFFFF)
#define IRQ_SOFT            (3)
#define IRQ_TIMER           (7)
#define IRQ_EXTERNAL        (11)

// trap_entry in crt.S
extern void trap_entry();
extern const uint32_t _stack_start;

#define GPIO_BASE ((uint32_t *)0x91000000)
#define UART_BASE ((uint8_t *)0x90000000)

// Tick timer incremented every millisecond by the timer interrupt
uint64_t tick;

enum {
    UART_RBR      = 0x00,  /* Receive Buffer Register */
    UART_THR      = 0x00,  /* Transmit Hold Register */   
    UART_IER      = 0x01,  /* Interrupt Enable Register */
    UART_DLL      = 0x00,  /* Divisor LSB (LCR_DLAB) */
    UART_DLM      = 0x01,  /* Divisor MSB (LCR_DLAB) */
    UART_FCR      = 0x02,  /* FIFO Control Register */
    UART_LCR      = 0x03,  /* Line Control Register */ 
    UART_MCR      = 0x04,  /* Modem Control Register */
    UART_LSR      = 0x05,  /* Line Status Register */ 
    UART_MSR      = 0x06,  /* Modem Status Register */
    UART_SCR      = 0x07,  /* Scratch Register */
    
    UART_LCR_DLAB = 0x80,  /* Divisor Latch Bit */
    UART_LCR_8BIT = 0x03,  /* 8-bit */
    UART_LCR_PODD = 0x08,  /* Parity Odd */
    
    UART_LSR_DA   = 0x01,  /* Data Available */
    UART_LSR_OE   = 0x02,  /* Overrun Error */
    UART_LSR_PE   = 0x04,  /* Parity Error */ 
    UART_LSR_FE   = 0x08,  /* Framing Error */  
    UART_LSR_BI   = 0x10,  /* Break indicator */
    UART_LSR_RE   = 0x20,  /* THR is empty */
    UART_LSR_RI   = 0x40,  /* THR is empty and line is idle */
    UART_LSR_EF   = 0x80,  /* Erroneous data in FIFO */
};


static volatile uint32_t *gpio = GPIO_BASE;
static volatile uint8_t *uart = UART_BASE;

static void __attribute__((optimize("O0"))) _delay(int x)
{
    x *= (5000000/1000);
    while(x)
    {
        x--;
    }
}

static void gpio_write(uint8_t x)
{
    *((uint8_t *)gpio) = x;
}

void timer_callback()
{
    tick++;

    for(int i=0;i<MAX_TIMERS;i++)
    {
        if(timers[i].set)
        {
            if(tick % timers[i].mod == 0)
            {
                (*timers[i].callback)(timers[i].data);
            }
        }
    }
}

void trap_callback() {
    uint32_t cause = csr_read(mcause);
    uint32_t code = cause & MCAUSE_CODE_MASK;

    if(cause & MCAUSE_INT_MASK)
    {
        switch(code)
        {
            case IRQ_TIMER:
                timer_callback();
                break;
            case IRQ_EXTERNAL:
                printf("External interrupt\n");
                break;
            default:
                printf("Unhandled IRQ %ld\n", code);
        }
    } else {
        printf("Trap code %ld\n", code);
    }

    //csr_set(sip, MIP_STIP);
    //csr_clear(mie, MIE_MTIE);

    //csr_clear(mstatus, MSTATUS_MIE);
}

//void _putchar(char ch)
void _putc(void *p, char ch)
{
    while ((uart[UART_LSR] & UART_LSR_RI) == 0);
    uart[UART_THR] = ch;
}

int init_trap()
{
    // Set mtvec to trap_entry in crt.S
    uint32_t sp = (uint32_t) (&_stack_start);
    printf("Stack start: %p\n", &_stack_start);
    printf("Trap Entry: %p\n", trap_entry);
    tick = 0;
    csr_write(mtvec, trap_entry);
    csr_write(mscratch, sp -32*4);
    //csr_write(mstatus, 0x0800 | MSTATUS_MPIE);
    //csr_write(mie, 0x880);

    csr_write(mie, 0x880);
    csr_write(mstatus, 0x1808);

    return 0;
}

void status_timer(void *data)
{
    printf("Status Timer. Uptime: %d\n", (int)(tick/1000));
}

int main()
{
    int c = 0;

    gpio_write(0x6);
    init_printf(NULL, _putc);

    printf("\r\nRISC-V Booting\r\n");

    init_trap();

    gpio_write(0x7);

    timers[0].set = 1;
    timers[0].mod = 1000;
    timers[0].callback = status_timer;
    timers[0].data = NULL;

    while(1)
    {
        c++;

        printf("Count: %d Tick: %lld\n", c, tick);

        gpio_write(0x1);
        _delay(200);
        gpio_write(0x0);
        _delay(200);
    }
}