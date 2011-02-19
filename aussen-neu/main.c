/*
 * vim:ts=4:sw=4:expandtab
 *
 * ATmega644P (4096 Bytes SRAM)
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdint.h>
#define BAUD 38400
#include <util/setbaud.h>

#include "lcd.h"

/* Nach 40ms erkanntem Druck wird eine Taste als gedrückt erkannt */
#define DEBOUNCE_MS 40

static struct lookup_entry {
    uint8_t state;
    uint8_t debounce;
    char key;
} lookup_table[] = {
    { 0xED, 0, '1' },
    { 0xEB, 0, '2' },
    { 0xAF, 0, '3' },
    { 0x7D, 0, '4' },
    { 0x7B, 0, '5' },
    { 0x3F, 0, '6' },
    { 0xDD, 0, '7' },
    { 0xDB, 0, '8' },
    { 0x9F, 0, '9' },
    { 0xFC, 0, '*' },
    { 0xFA, 0, '0' },
    { 0xBE, 0, '#' }
};

/* buffer for UART input */
static volatile char ibuffer[10];
static volatile uint8_t ucnt = 0;

static volatile uint16_t beepcnt = 0;

static void uart2_init() {
    /* activate second uart */
    UBRR1H = UBRRH_VALUE;
    UBRR1L = UBRRL_VALUE;

    /* Generate an interrupt on incoming data, enable receiver/transmitter */
    UCSR1B = (1 << RXCIE1) | (1 << RXEN1) | (1 << TXEN1);

    /* frame format: 8N1 */
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
}

static void uart2_puts(char *str) {
    char *walk;
    for (walk = str; *walk != '\0'; walk++) {
        while ( !( UCSR1A & (1<<UDRE1)) );
        UDR1 = (unsigned char)*walk;
    }
}

/*
 * Timer-Interrupt. Wird alle 1ms getriggert.
 *
 */
ISR(TIMER0_OVF_vect) {
    TCNT0 = 5;
    if (beepcnt > 0 && --beepcnt == 0)
        /* Summer deaktivieren */
        PORTB &= ~(1 << PB4);

    /* Debouncing */
    uint8_t sample = (PINA | (1 << 3));
    uint8_t c;
    for (c = 0; c < sizeof(lookup_table) / sizeof(struct lookup_entry); c++)
        if (sample == lookup_table[c].state) {
            if (lookup_table[c].debounce < DEBOUNCE_MS)
                lookup_table[c].debounce++;
        } else lookup_table[c].debounce = 0;
}

/*
 * Interrupt für Aktivität auf dem UART
 *
 */
ISR(USART1_RX_vect) {
    uint8_t byte = UDR1;
    UDR1 = byte;

    /* If the current buffer is not processed yet, we cannot accept the packet
     * (this should rarely happen as the main loop checks for complete buffers,
     * copies them and handles them) */
    if (ibuffer[8] == '$') {
        PORTB |= (1 << PB0);
        return;
    }
    PORTB &= ~(1 << PB0);

    /* For the first byte of the buffer, we expect '^' (start of packet) */
    if (ucnt == 0 && byte != '^')
        return;

    /* For the last byte of the buffer, we expect '$' (end of packet) */
    if (ucnt == 8 && byte != '$')
        return;

    /* Save the byte */
    ibuffer[ucnt] = byte;
    ucnt++;

    if (ucnt == 9)
        ucnt = 0;
}

static void handle_command(char *buffer) {
    if (strncmp(buffer, "^BEEP", strlen("^BEEP")) == 0) {
        /* Summer-Ausschalten auf 100ms bzw. 500ms setzen */
        if (buffer[6] == '1')
            beepcnt = 100;
        else beepcnt = 500;

        /* Summer aktivieren */
        PORTB |= (1 << PB4);
        return;
    }

    if (strncmp(buffer, "^PING", strlen("^PING")) == 0) {
        char buf[12];
        strncpy(buf, "^PONG cc$\r\n\0", strlen("^PONG cc$\r\n") + 1);
        buf[6] = buffer[6];
        buf[7] = buffer[7];
        uart2_puts(buf);
        return;
    }

    if (strncmp(buffer, "^LED", strlen("^LED")) == 0) {
        //int idx = (buffer[5] == '1' ? 0 : 1);
        //int state;
        //if (buffer[7] == '1')
        //    state = 1;
        //else if (buffer[7] == '2')
        //    state = 2;
        //else state = 3;
        //led_state[idx] = state;

        //DDRC |= (1 << (PC1 + idx));
        //PORTC |= (1 << (PC1 + idx));
    }
}

int main() {
    /* receive enable aktivieren, data deaktivieren */
    DDRD = (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7);
    PORTD = (1 << PD6);

    /* Ziffernfeld-pins (PA0-PA7) als Eingang schalten */
    DDRA = 0;
    PORTA = 0xFF;

    /* LEDs und Summer als Ausgang schalten */
    DDRB = (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB4);

    /* Selbst-Test: Alle LEDs hintereinander für 250 ms aktivieren, ebenso den Summer */
    PORTB = 0;
    _delay_ms(250);
    PORTB = (1 << PB0);
    _delay_ms(250);
    PORTB = (1 << PB1);
    _delay_ms(250);
    PORTB = (1 << PB2);
    _delay_ms(250);
    PORTB = (1 << PB3);
    _delay_ms(250);

    PORTB = (1 << PB4);
    _delay_ms(250);
    PORTB = 0;

    ibuffer[8] = '\0';
    ibuffer[9] = '\0';

    uart2_init();
    uart2_puts("Initializing RS485\r\n");

    uart2_puts("Done, entering mainloop2\r\n");

    lcd_init(LCD_DISP_ON);
    uart2_puts("initialized LCD\r\n");
    lcd_clrscr();
    lcd_puts("pinpad ready\nself-test ok");
    uart2_puts("done.\r\n");

    /* Timer aufsetzen: nach 1 ms soll der Interrupt ausgelöst werden. */
    /* 8 bit counter (TIMER0) */
    /* normal mode */
    TCCR0A = 0;
    /* CLK/64 */
    TCCR0B = (1 << CS01) | (1 << CS00);
    /* timer ticks: 250 */
    TCNT0 = 5;
    TIMSK0 = (1 << TOIE0);
    TIFR0 = (1 << TOV0);

    sei();

    int c;
    char buffer[12];
    buffer[11] = '\0';
    char bufcopy[10];
    for (;;) {
        /* Handle commands received on the UART */
        if (ibuffer[8] == '$') {
            strncpy(bufcopy, ibuffer, sizeof(bufcopy));
            /* change the end of packet marker in memory so that the next
             * packet will be accepted by the RX interrupt handler */
            ibuffer[8] = '\0';

            handle_command(bufcopy);
        }

        uint8_t sample = (PINA | (1 << 3));

        for (c = 0; c < sizeof(lookup_table) / sizeof(struct lookup_entry); c++) {
            if (sample != lookup_table[c].state ||
                lookup_table[c].debounce != DEBOUNCE_MS)
                continue;
            strncpy(buffer, "^PAD c  $\r\n", strlen("^PAD c  $\r\n"));
            buffer[5] = lookup_table[c].key;
            uart2_puts(buffer);
            lookup_table[c].debounce = DEBOUNCE_MS+1;
        }
    }
}