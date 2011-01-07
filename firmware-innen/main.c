/*
 * vim:ts=4:sw=4:expandtab
 *
 */
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#define BAUD 9600
#include <util/setbaud.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>

/* seite 84 */
static char pin[11];
static uint8_t pincnt = 0;
static volatile char serbuf[64];
static volatile uint8_t sercnt = 0;

ISR(USART0_RX_vect) {
    uint8_t byte;
    uint8_t usr;

    usr = UCSR0A;
    byte = UDR0;

    if (serbuf[8] == '$')
        return;

    if (sercnt == 0 && byte != '^')
        return;

    if (sercnt == 8 && byte != '$')
        return;

    serbuf[sercnt] = byte;
    sercnt++;

    if (sercnt == 9)
        sercnt = 0;
}

static void uart_puts(const char *str) {
    int c;

    for (c = 0; c < strlen(str); c++) {
        while ( !( UCSR0A & (1<<UDRE0)) );
        UDR0 = (unsigned char)str[c];
    }
}

static void handle_command(const char *buffer) {
    if (strncmp(buffer, "^PAD ", strlen("^PAD ")) == 0) {
        char c = buffer[5];
        if (pincnt < 10) {
            pin[pincnt] = c;
            pincnt++;
        }
        uart_puts("^LED 2 1$\n");
        uart_puts("^BEEP 1 $\n");
        char *str;
        if (pincnt == 7 && strncmp(pin, "777#", 5) == 0) {
            uart_puts("^LED 2 2$");
            uart_puts("^BEEP 2 $\n");
            pincnt = 0;
            memset(pin, '\0', sizeof(pin));
            PORTA &= ~(1 << PA2);
            _delay_ms(500);
            PORTA |= (1 << PA2);
        } else if (pincnt == 4 && strncmp(pin, "666#", 5) == 0) {
            uart_puts("^LED 2 2$");
            uart_puts("^BEEP 2 $\n");
            pincnt = 0;
            memset(pin, '\0', sizeof(pin));
            PORTA &= ~(1 << PA3);
            _delay_ms(500);
            PORTA |= (1 << PA3);
        } else if (c == '#') {
            uart_puts("^LED 1 2$^BEEP 2 $");
            //uart_puts("^BEEP 2 $\n");
            pincnt = 0;
            memset(pin, '\0', sizeof(pin));
        }
            
    }
}

int main() {
    char bufcopy[10];

    DDRA = (1 << PA2) | (1 << PA3);
    PORTA = (1 << PA2) | (1 << PA3);


    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
    UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);

    UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);


    /* disable watchdog */
    MCUSR &= ~(1 << WDRF);
    WDTCSR &= ~(1 << WDE);
    wdt_disable();

    sei();

    /* enable LED so that the user knows the controller is active */
    DDRC = (1 << PC7);
    PORTC = (1 << PC7);

    char *str = "^AEEP 1 $\n";
    int c;
    for (;;) {
        if (serbuf[8] == '$') {
            strncpy(bufcopy, serbuf, sizeof(bufcopy));
            serbuf[8] = '\0';

            handle_command(bufcopy);
        }
        //_delay_ms(250);
#if 0
        for (c = 0; c < strlen(str); c++) {
            while ( !( UCSR0A & (1<<UDRE0)) );
            UDR0 = (unsigned char)str[c];
        }
#endif
    }
}
