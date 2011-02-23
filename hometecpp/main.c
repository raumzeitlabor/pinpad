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

enum {
    DIR_OPEN = 0,
    DIR_CLOSE
};

uint8_t admux_mask;
uint8_t pc_drehen;
uint8_t pc_gegendrehen;
uint8_t current_direction;

enum {
    STATE_NOP = 0,
    STATE_BROKEN, /* alles kaputt, motor kriegt keinen strom */
    STATE_OPEN_EINKOPPELN, /* Motor soll eingekoppelt werden */
    STATE_OPENING, /*  Motor soll in Öffnungsrichtung gedreht werden */
    STATE_ADC_OPEN_START, /* ADC4 (öffnen) soll ausgelesen werden */
    STATE_ADC_OPEN_RESTART, /* ADC auslesen, noch nicht genug messwerte */
    STATE_ADC_OPEN_WAIT, /* ADC4 (öffnen) soll ausgelesen werden */
    STATE_ADC_OPEN_RESULT, /* ADC4 hat ein ergebnis */
    STATE_OPENED, /* ADC ist 3 */
    STATE_OPENED_BACK, /* Motor in Schließrichtung für 250ms gedreht werden */
    STATE_OPEN_AUSKOPPELN
};

/* Sensoren-Stati */
enum {
    SENSOR_LOCKED = 0,
    SENSOR_NOT_LOCKED,
    SENSOR_BROKEN
};

uint8_t last_lockstate = 0;
uint8_t lockstate_cnt = 0;
uint8_t lockstate_stable = 0;

volatile uint32_t last_adc = 0;

volatile uint8_t state = 0;

static void uart2_init() {
    /* activate second uart */
    UBRR1H = UBRRH_VALUE;
    UBRR1L = UBRRL_VALUE;

    /* Enable transmitter */
    UCSR1B = /*(1 << RXEN1) | (1 << RXCIE1) | */(1 << TXEN1);

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


static uint8_t get_lockstate() {
    bool pb0 = (PINB & (1 << PB2));
    bool pb1 = (PINB & (1 << PB3));

    if (pb0 && !pb1)
        return SENSOR_LOCKED;
    else if (!pb0 && pb1)
        return SENSOR_NOT_LOCKED;
    else return SENSOR_BROKEN;
}

static void update_pwm() {
    uint8_t fuckedup = 0;

    /* Aktion ist:
     * 0 - nichts tun - STATE_NOP
     * 1 - öffnen - !STATE_NOP && !STATE_BROKEN && current_direction = DIR_OPEN
     * 2 - schließen - !STATE_NOP && !STATE_BROKEN && current_direction = DIR_CLOSE
     */
    uint8_t aktion = 0;
    if (state == STATE_NOP)
        aktion = 0;
    else if (state != STATE_NOP &&
             state != STATE_BROKEN &&
             current_direction == DIR_OPEN)
        aktion = 1;
    else if (state != STATE_NOP &&
             state != STATE_BROKEN &&
             current_direction == DIR_CLOSE)
        aktion = 2;
    else fuckedup = 1;

    /* Sensorwerte */
    uint8_t sensors;
    if (lockstate_stable == SENSOR_LOCKED)
        sensors = 2;
    else if (lockstate_stable == SENSOR_NOT_LOCKED)
        sensors = 1;
    else fuckedup = 1;

    uint8_t combined = (fuckedup << 4) | (aktion << 2) | sensors;
    uint16_t pulse = combined * 3855;

#if 0
    char buf[64];
    snprintf(buf, sizeof(buf), "combined: %u, pulse = %u\r\n", combined, pulse);
    uart2_puts(buf);
#endif

    OCR1BH = (pulse & 0xFF00) >> 8;
    OCR1BL = (pulse & 0x00FF);
}

ISR(USART1_RX_vect) {
    uint8_t byte = UDR1;

    if (byte == 's') {
        PORTC = 0;
        uart2_puts("Stop\r\n");
        state = 0;
    }

    if (byte == '1') {
        uart2_puts("oeffnen!\r\n");
        current_direction = DIR_OPEN;
        pc_drehen = (1 << PC4);
        pc_gegendrehen = (1 << PC3);
        admux_mask = (1 << MUX1) | (1 << MUX0);
        state = STATE_OPEN_EINKOPPELN;
    }

    if (byte == '2') {
        uart2_puts("schliessen\r\n");
        current_direction = DIR_CLOSE;
        pc_drehen = (1 << PC3);
        pc_gegendrehen = (1 << PC4);
        admux_mask = (1 << MUX2);
        state = STATE_OPEN_EINKOPPELN;
    }

    if (byte == '0') {
        bool sensor1 = (PINB & (1 << PB2));
        bool sensor2 = (PINB & (1 << PB3));
        char buf[64];
        snprintf(buf, sizeof(buf), "sensor1: %d, sensor2: %d\r\n", sensor1, sensor2);
        uart2_puts(buf);
    }
}

volatile uint8_t ms_passed = 0;

/*
 * Timer-Interrupt. Wird alle 1ms getriggert.
 *
 */
ISR(TIMER0_OVF_vect) {
    TCNT0 = 5;
    ms_passed++;
}


volatile uint16_t lowvalue;
volatile uint16_t highvalue;
volatile uint16_t pwmvalue;

ISR(TIMER1_CAPT_vect) {
    /* Schauen, bei welcher Flanke wir gerade sind */
    TIMSK1 &= ~(1 << ICIE1);

    /* Steigende Flanke */
    if (TCCR1B & (1 << ICES1)) {
        /* Wert wegspeichern */
        lowvalue = ICR1L;
        lowvalue |= (ICR1H << 8);

        /* flanke ändern */
        TCCR1B &= ~(1 << ICES1);
    } else {
        /* Fallende Flanke */
        highvalue = ICR1L;
        highvalue |= (ICR1H << 8);

        /* Zählerüberlauf */
        if (highvalue < lowvalue) {
            pwmvalue = 0xFFFF - lowvalue + highvalue;
        } else {
            pwmvalue = highvalue - lowvalue;
        }

        /* flanke ändern */
        TCCR1B |= (1 << ICES1);
    }

    TIMSK1 |= (1 << ICIE1);
}

static void update_state(uint8_t new_state) {
    uint8_t sensor1;
    uint8_t sensor2;
    uint8_t wanted_state;

    /* Das letzte Bit gibt den Status von Sensor 2 an */
    sensor2 = (new_state & (1 << 0));
    /* Das vorletzte Bit den Status von Sensor 1 */
    sensor1 = (new_state & (1 << 1));
    /* Das dritte und vierte Bit geben den Aktionscode an */
    wanted_state = (new_state & ((1 << 2) | (1 << 3))) >> 2;
    /* Wenn das erste Bit gesetzt ist (fuckup), wird die Statemachine resettet */
    if (new_state & (1 << 4)) {
        uart2_puts("Force reset\r\n");
        state = STATE_NOP;
        return;
    }

    char buf[128];
    snprintf(buf, sizeof(buf), "wanted_state: %u, sensor1 = %u, sensor2 = %u\r\n",
            wanted_state, sensor1, sensor2);
    uart2_puts(buf);

    if (state == STATE_NOP && wanted_state == 1) {
        uart2_puts("oeffnen(pwm)\r\n");
        current_direction = DIR_OPEN;
        pc_drehen = (1 << PC4);
        pc_gegendrehen = (1 << PC3);
        admux_mask = (1 << MUX1) | (1 << MUX0);
        state = STATE_OPEN_EINKOPPELN;
    }

    if (state == STATE_NOP && wanted_state == 2) {
        uart2_puts("schliessen(pwm)\r\n");
        current_direction = DIR_CLOSE;
        pc_drehen = (1 << PC3);
        pc_gegendrehen = (1 << PC4);
        admux_mask = (1 << MUX2);
        state = STATE_OPEN_EINKOPPELN;
    }
}

static void output_pwm() {
    uint16_t snap = pwmvalue;
    uint8_t c;
    char buf[128];
    for (c = 1; c < 17; c++) {
        if (snap > ((3855 * c) - 500) &&
            snap < ((3855 * c) + 500)) {
            update_state(c);

//    snprintf(buf, sizeof(buf), "pwm: %u, div = %u, high = %u, low = %u\r\n", snap, c, highvalue, lowvalue);
//    uart2_puts(buf);
    return;
        }
    }
    snprintf(buf, sizeof(buf), "Error: PWM %u out of range, high = %u, low = %u\r\n", snap, highvalue, lowvalue);
    uart2_puts(buf);
}

uint8_t messanzahl = 10;
uint16_t werte[10];
volatile uint8_t messung;

ISR(ADC_vect) {
    char buf[128];
    uint16_t l = ADCL;
    uint16_t h = ADCH;

    werte[messung] = (h << 8) | l;
    messung++;
    if (messung == messanzahl-1) {
        if (state == STATE_ADC_OPEN_WAIT) {
            uint8_t j;
            last_adc = 0;
            for (j = 0; j < messung; j++) {
                last_adc += werte[j];
            }
            last_adc /= messung;
            messung = 0;
            state = STATE_ADC_OPEN_RESULT;
            snprintf(buf, sizeof(buf), "ADC: %u\r\n", last_adc);
            uart2_puts(buf);
        }
    } else {
        if (state == STATE_ADC_OPEN_WAIT) {
        state = STATE_ADC_OPEN_RESTART;
        }
    }
}

int main() {
    /* PC0 - Chip Enable 1 - Kupplungsmotor
     * PC1 - Input 1 - Kupplungsmotor
     * PC2 - Input 2 - Kupplungsmotor
     * PC3 - Input 3 - Drehmotor
     * PC4 - Input 4 - Drehmotor
     * PC5 - Chip Enable 2 - Drehmotor
     *
     * PA0 (ADC) - Strom, der durch beide Motoren zusammen fließt
     * PA1 (ADC) - Sense 1 - Kupplungsmotor
     * PA2 (ADC) - Sense 2 - Kupplungsmotor
     * PA3 (ADC) - Sense 3 - Drehmotor
     * PA4 (ADC) - Sense 4 - Drehmotor
     */

    /* auf PB0 und PB1 sind die sensoren angeschlossen */
    DDRB = 0;
    PORTB = 0xFF;

    DDRC = 0xFF;
    //PORTC = (1 << PC0) | (1 << PC1);
    //PORTC = (1 << PC0) | (1 << PC2);
    //PORTC = (1 << PC5) | (1 << PC3);
    //PORTC = (1 << PC5) | (1 << PC4);
    PORTC = 0;

    DDRA = 0;

    uart2_init();
    uart2_puts("moin\r\n");

    DDRD = (1 << PD4);
    PORTD = 0;
    /* PWM-Frequenz setzen */
    TCCR1A = (1 << COM1B1) | (1 << WGM11) | (1 << WGM10);
    TCCR1B = (1 << ICNC1) | (1 << ICES1) | (1 << WGM13) | (1 << WGM12) | (1 << CS11) | (1 << CS10);
    OCR1AH = 0xFF;
    OCR1AL = 0xFF;
    TCNT1H = 0;
    TCNT1L = 0;

    TIMSK1 = (1 << ICIE1);

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

    uint16_t freq = 3855;
    uint8_t c;
    uint8_t j;
    char buf[64];
    uint8_t lockstate;
    for (;;) {
        lockstate = get_lockstate();
        if (lockstate != last_lockstate) {
            last_lockstate = lockstate;
            lockstate_cnt = 0;
        } else {
            if (lockstate_cnt == 3) {
                lockstate_stable = lockstate;
                /* Hier ist der lockstate stabil bekannt */

            } else lockstate_cnt++;
        }

        snprintf(buf, sizeof(buf), "state = %u (wait=%u), curdir = %u(dirclose=%u), lss = %u(locked=%u)\r\n",
                state, STATE_ADC_OPEN_WAIT, current_direction, DIR_CLOSE, lockstate_stable, SENSOR_LOCKED);
        uart2_puts(buf);
                if ((state == STATE_ADC_OPEN_WAIT || state == STATE_ADC_OPEN_RESULT) && current_direction == DIR_OPEN && lockstate_stable == SENSOR_NOT_LOCKED) {
                    state = STATE_OPENED_BACK;
                    uart2_puts("State-change durch sensor\r\n");
                }
                if ((state == STATE_ADC_OPEN_WAIT || state == STATE_ADC_OPEN_RESULT) && current_direction == DIR_CLOSE && lockstate_stable == SENSOR_LOCKED) {
                    state = STATE_OPENED_BACK;
                    uart2_puts("State-change durch sensor\r\n");
                }

        /* Einkoppeln: Einkoppel-Motor starten (läuft 250ms, Drehmotor läuft sofort danach an) */
        if (state == STATE_OPEN_EINKOPPELN) {
            uart2_puts("Einkoppeln\r\n");
            PORTC = (1 << PC0) | (1 << PC1);
            ms_passed = 0;
            state = STATE_OPENING;
        }

        if (state == STATE_OPENING) {
            uart2_puts("Drehen\r\n");
            PORTC = (1 << PC5) | pc_drehen;
            _delay_ms(250);
            state = STATE_ADC_OPEN_START;
        }

        if (state == STATE_ADC_OPEN_START) {
            messung = 0;
            state = STATE_ADC_OPEN_RESTART;
        }

        if (state == STATE_ADC_OPEN_RESTART) {
            uart2_puts("ADC messen\r\n");
            /* ADC messen */
            ADMUX = (1 << REFS1) | admux_mask;
            ADCSRA = (1 << ADEN) | (1 << ADIF) | (1 << ADIE) | (1 << ADPS2);
            DIDR0 = 0xFF;
            /* start */
            ADCSRA |= (1 << ADSC);
            state = STATE_ADC_OPEN_WAIT;
        }
        if (state == STATE_ADC_OPEN_RESULT) {
            uart2_puts("ADC ergebnis\r\n");
            if (last_adc > 580) {
                /* Am ende angekommen */
                state = STATE_OPENED;
            } else {
                state = STATE_ADC_OPEN_START;
            }
        }
        if (state == STATE_OPENED) {
            uart2_puts("geoffnet, zurueckdrehen\r\n");
            PORTC = (1 << PC5) | pc_gegendrehen;
            _delay_ms(150);
            state = STATE_OPENED_BACK;
        }
        if (state == STATE_OPENED_BACK) {
            uart2_puts("zurueck, auskoppeln\r\n");
            /* Wir sind zurückgedreht, Auskoppeln */

            for (j = 0; j < 5; j++) {
                PORTC = (1 << PC0) | (1 << PC2);
                _delay_ms(250);
                PORTC = 0;
                _delay_ms(250);
            }
            state = STATE_NOP;
        }

        if (state == STATE_ADC_OPEN_WAIT && ms_passed >= 250) {
            ms_passed = 0;
            if (PORTC & (1 << PC0)) {
                PORTC &= ~((1 << PC0) | (1 << PC1));
            } else {
                PORTC |= (1 << PC0) | (1 << PC1);
            }
            uart2_puts("250 ms passed, koppeln\r\n");
        }

        update_pwm();
        output_pwm();
    }
}
