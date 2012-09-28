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

static uint16_t led_state[4] = { 0, 0, 0, 0 };

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

/* Buffer for UART input. This is 39 bytes because the longest command is the
 * LCD command which has 2 bytes for header/footer, 3 bytes for "LCD", one
 * space and 32 bytes of payload, followed by a trailing 0-byte for
 * printability. */
//#define COMMAND_BUFFER_SIZE (strlen("^$") + strlen("LCD ") + 32 + 1)
#define COMMAND_BUFFER_SIZE (2 + 4 + 32 + 1)
static volatile char ibuffer[COMMAND_BUFFER_SIZE];
static volatile uint8_t ucnt = 0;

static volatile uint16_t beepcnt = 0;

static void uart1_init() {
    /* activate second uart */
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

    /* Generate an interrupt on incoming data, enable receiver/transmitter */
    UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);

    /* frame format: 8N1 */
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

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

static void uart_puts(char *str) {
    char *walk;
    for (walk = str; *walk != '\0'; walk++) {
        while ( !( UCSR0A & (1<<UDRE0)) );
        UDR0 = (unsigned char)*walk;
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

    int idx;
    for (idx = 0; idx < 4; idx++) {
        if (led_state[idx] > 0 && --(led_state[idx]) == 0)
            PORTB &= ~(1 << idx);
    }

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
 * Auf UART 1 (zum Debuggen) haben wir ein Line Echo.
 *
 */
ISR(USART1_RX_vect) {
    uint8_t byte = UDR1;
    UDR1 = byte;
}

/*
 * Interrupt für Aktivität auf dem UART
 *
 */
ISR(USART0_RX_vect) {
    uint8_t byte = UDR0;

    /* If the current buffer is not processed yet, we cannot accept the packet
     * (this should rarely happen as the main loop checks for complete buffers,
     * copies them and handles them) */
    if (ibuffer[sizeof(ibuffer) - 2] == '$') {
        return;
    }

    /* For the first byte of the buffer, we expect '^' (start of packet) */
    if (ucnt == 0 && byte != '^')
        return;

    /* For the last byte of the buffer, we expect '$' (end of packet) */
    if (ucnt == (sizeof(ibuffer) - 2) && byte != '$')
        return;

    /* Save the byte */
    ibuffer[ucnt] = byte;
    ucnt++;

    if (ucnt == (sizeof(ibuffer) - 1))
        ucnt = 0;
}

static void handle_command(char *buffer) {
    if (strncmp(buffer, "^BEEP", strlen("^BEEP")) == 0) {
        /* Summer-Ausschalten auf 100ms bzw. 500ms setzen */
        if (buffer[6] == '1')
            beepcnt = 100;
        else if (buffer[6] == '2')
            beepcnt = 25;
        else beepcnt = 500;

        /* Summer aktivieren */
        PORTB |= (1 << PB4);
        return;
    }

    if (strncmp(buffer, "^PING", strlen("^PING")) == 0) {
        /* Modify the buffer in place and send it back */
        buffer[2] = 'O';
        uart_puts(buffer);
        return;
    }

    // example:
    // ^LED 0 250                           $ (rot oben)
    // ^LED 1 250                           $ (grün oben)
    // (beides zusammen ergibt orange)
    // ^LED 2 250                           $ (rot unten)
    // ^LED 3 250                           $ (grün unten)
    // ^BEEP 1                              $
    if (strncmp(buffer, "^LED", strlen("^LED")) == 0) {
        int idx = (buffer[5] - '0');
        if (idx < 0 || idx > 3)
            return;

        if (sscanf(buffer + strlen("^LED 0 "), "%u", &(led_state[idx])) != 1)
            return;

        PORTB |= (1 << idx);
        return;
    }

    // ^LCD Herzlich Willkommen im RZL      $
    // ^LCD PIN: *                          $
    // ^LCD PIN: **                         $
    if (strncmp(buffer, "^LCD", strlen("^LCD")) == 0) {
        lcd_clrscr();
        char *walk = buffer + strlen("^LCD ") + 31;
        while (*walk == ' ')
            walk--;
        *(++walk) = '\0';
        lcd_puts(buffer + strlen("^LCD "));
        return;
    }

    // ^LCH *                               $
    if (strncmp(buffer, "^LCH", strlen("^LCH")) == 0) {
        lcd_putc(buffer[strlen("^LCD ")]);
        return;
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

    memset((void*)ibuffer, '\0', sizeof(ibuffer));

    uart2_init();
    uart2_puts("Initializing RS485\r\n");
    uart1_init();

    uart2_puts("Waiting for LCD...\r\n");

    lcd_init(LCD_DISP_ON);
    uart2_puts("initialized LCD\r\n");
    lcd_clrscr();
    lcd_puts("pinpad ready\nself-test ok");
    uart2_puts("done. now accepting cmds\r\n");

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
    char keypress_buffer[COMMAND_BUFFER_SIZE + 2] =
        "^PAD c                               $\r\n";
    char bufcopy[COMMAND_BUFFER_SIZE];
    for (;;) {
        /* Handle commands received on the UART */
        if (ibuffer[sizeof(ibuffer)-2] == '$') {
            strncpy(bufcopy, (const char *)ibuffer, sizeof(bufcopy));
            /* change the end of packet marker in memory so that the next
             * packet will be accepted by the RX interrupt handler */
            ibuffer[sizeof(ibuffer)-2] = '\0';

            handle_command(bufcopy);
        }

        uint8_t sample = (PINA | (1 << 3));

        for (c = 0; c < sizeof(lookup_table) / sizeof(struct lookup_entry); c++) {
            if (sample != lookup_table[c].state ||
                lookup_table[c].debounce != DEBOUNCE_MS)
                continue;
            keypress_buffer[5] = lookup_table[c].key;
            uart_puts(keypress_buffer);
            lookup_table[c].debounce = DEBOUNCE_MS+1;
        }
    }
}
