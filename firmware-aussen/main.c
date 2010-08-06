/*
 * vim:ts=4:sw=4:expandtab
 *
 */
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#define BAUD 9600
#include <util/setbaud.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>

/* debounce counters */
uint8_t d_cnt[3][4] = {
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0}
};

char chars[3][4] = {
    {'1', '4', '7', '*'},
    {'2', '5', '8', '0'},
    {'3', '6', '9', '#'}
};

uint8_t led_state[2] = {0, 0};

/* buffer for UART input */
char ibuffer[10];
uint8_t ucnt = 0;

static void uart_puts(const char *str) {
    const char *walk;
    for (walk = str; *walk != '\0'; walk++) {
        while ( !( UCSR0A & (1<<UDRE0)) );
        UDR0 = *walk;
    }
}

/*
 * This interrupt is triggered 61 times per second
 * == every 0.016s == every 16 msec
 *
 */
ISR(TIMER0_OVF_vect) {
    uint8_t c;
    for (c = PC5; c >= PC3; c--) {
        /* set PC5, PC4, PC3 as output, low (col) and check each row */
        DDRC |= (1 << c);
        PORTC &= ~(1 << c);
        /* ensure that the output is set properly by waiting a bit */
        _delay_us(500);

#define update_cnt(pin, pd, cnt) \
    do { \
        if ((pin & (1 << pd)) == 0) { \
            if (d_cnt[PC5-c][cnt] < 4) \
                d_cnt[PC5-c][cnt]++; \
        } else d_cnt[PC5-c][cnt] = 0; \
    } while (0)

        update_cnt(PIND, PD5, 0);
        update_cnt(PIND, PD6, 1);
        update_cnt(PIND, PD7, 2);
        update_cnt(PINB, PB0, 3);

        /* set the pin as input again */
        DDRC &= ~(1 << c);
    }
}

#define count_to(varname, cnt) \
    do { \
        static uint8_t varname = 0; \
        if (varname++ < cnt) \
            return; \
        varname = 0; \
    } while (0)

#define led_state(pin, idx) \
    do { \
        if (led_state[idx] == 0) \
            break; \
        if (led_state[idx] == 1) { \
            DDRC &= ~(1 << pin); \
            led_state[idx] = 0; \
            break; \
        } \
        if (led_state[idx] == 2) { \
            count_to(longdelay, 10); \
            DDRC &= ~(1 << pin); \
            led_state[idx] = 0; \
            break; \
        } \
        count_to(toggledelay, 5); \
        static uint8_t toggle = 1; \
        if ((toggle % 2) == 0) { \
            DDRC |= (1 << pin); \
            PORTC |= (1 << pin); \
        } else { \
            DDRC &= ~(1 << pin); \
        } \
        toggle++; \
    } while (0)

ISR(TIMER2_OVF_vect) {
    /* Minimum delay because the prescaler cannot scale enough for our purpose */
    count_to(delay, 5);

    led_state(PC1, 0);
    led_state(PC2, 1);
}

ISR(USART_RX_vect) {
    uint8_t byte = UDR0;

    /* If the current buffer is not processed yet, we cannot accept the packet
     * (this should rarely happen as the main loop checks for complete buffers,
     * copies them and handles them) */
    if (ibuffer[8] == '$')
        return;

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

static void beep(const uint16_t ms) {
    TCCR1A = 0; // Stop all PWM on Timer 1 when setting up
    TCNT1H = 0;
    TCNT1L = 0;
    TCCR1A |= (1<<WGM13) | (1<<WGM12) | (1<<WGM11); // 16 bit Fast PWM using ICR1 for TOP
    TCCR1B = 0;
    TCCR1B |= (1<<CS11);  // pre-scaler = 1
    TCCR1A |= (1<<COM1A1); // enable channel A in non-inverting mode

    ICR1 = 320;  // 4kHz PWM

    OCR1A = ICR1 / 1; // 50% duty cycle
    /* XXX: don't delay but set the timer back to normal in the LED timer */
    _delay_ms(ms);
    TCCR1A = 0;
}

static void handle_command(char *buffer) {
    if (strncmp(buffer, "^BEEP", strlen("^BEEP")) == 0) {
        if (buffer[6] == '1')
            beep(100);
        else beep(500);
        return;
    }

    if (strncmp(buffer, "^PING", strlen("^PING")) == 0) {
        char buf[12];
        strncpy(buf, "^PONG cc$\r\n\0", strlen("^PONG cc$\r\n") + 1);
        buf[6] = buffer[6];
        buf[7] = buffer[7];
        uart_puts(buf);
        return;
    }

    if (strncmp(buffer, "^LED", strlen("^LED")) == 0) {
        int idx = (buffer[5] == '1' ? 0 : 1);
        int state;
        if (buffer[7] == '1')
            state = 1;
        else if (buffer[7] == '2')
            state = 2;
        else state = 3;
        led_state[idx] = state;

        DDRC |= (1 << (PC1 + idx));
        PORTC |= (1 << (PC1 + idx));
    }
}

int main() {
    ibuffer[8] = '\0';
    ibuffer[9] = '\0';

    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

    UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);

    UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);

    /* PD5, PD6, PD7 und PB0 sind die anderen */
    DDRD = 0;
    PORTD = (1 << PD5) | (1 << PD6) | (1 << PD7);

    DDRB = 0;
    PORTB = (1 << PB0);

    /* set up timer for LEDs */
    TCCR2A = 0;
    TCCR2B = (1 << CS02) | (1 << CS01) | (1 << CS00);
    TIMSK2 = (1 << TOIE2);

    /* set up timer for debouncing */
    TCCR0A = 0;
    TCCR0B = (1 << CS02) | (1 << CS00);
    TIMSK0 = (1 << TOIE0);

    /* set PB1 as an output for PWM (speaker) */
    DDRB |= (1 << PB1);

    /* enable interrupts for the timer */
    sei();

    uint8_t col, row;
    /* buffer for formatting output */
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

        for (col = 0; col < 3; col++) {
            for (row = 0; row < 4; row++) {
                if (d_cnt[col][row] != 4)
                    continue;

                /* button at (col, row) was pressed (and debounced) */

                /* increase counter so we get the next event for this button
                 * only after the user released it */
                d_cnt[col][row]++;

                /* we don't use snprintf() because it's rather heavy and we
                 * only deal with formatting here, so it's not worth it */
                strncpy(buffer, "^PAD c  $\r\n", strlen("^PAD c  $\r\n"));
                buffer[5] = chars[col][row];
                uart_puts(buffer);
            }
        }
    }
}
