#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include "tinyprintf.h"

#define GPIO_BASE ((uint32_t *)0x91000000)
#define UART_BASE ((uint8_t *)0x90000000)

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

void irqCallback(){
}

//void _putchar(char ch)
void _putc(void *p, char ch)
{
    while ((uart[UART_LSR] & UART_LSR_RI) == 0);
    uart[UART_THR] = ch;
}

int main()
{
    char buf[32];
    //void *p = malloc(128);
    int c = 0;

    init_printf(NULL, _putc);

    uart[UART_THR] = '=';

    printf("\r\nRISC-V Booting\r\n");
    printf("buf=%p\r\n", buf);


    gpio_write(0x6);


    while(1)
    {
        c++;
        memset(buf, 'a', 8);

        printf("buf=%s\n", buf);
        buf[0] += 1;
        buf[1] += 2;
        buf[2] += 3;
        buf[3] += 4;
        printf("buf=%s\n", buf);

        printf("%x\r\n", uart[0]);
        printf("%x\r\n", uart[1]);
        printf("%x\r\n", uart[2]);
        printf("%x\r\n", uart[3]);
        printf("%x\r\n", uart[4]);
        printf("%x\r\n", uart[5]);
        printf("%x\r\n", uart[6]);

        printf("Count: %d\n", c);

        gpio_write(0x1);
        _delay(200);
        gpio_write(0x0);
        _delay(200);
    }
}