#ifndef F_CPU
#define F_CPU 1000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdint.h>

/* ---------- Pin / LED mapping ---------- */
#define LED0_PA0 PA0   /* sensor 0 (PD2) -> this LED / motor direction */
#define LED1_PA7 PA7   /* sensor 1 (PD3) -> this LED / motor direction */
#define LED2_PB0 PB0   /* spare, kept off (reserve for a 3rd sensor later) */

/* ---------- Tunables ---------- */
#define DEBOUNCE_MS    80u   /* a sensor's pin must read continuously HIGH this long before it's trusted */
#define REFRACTORY_MS 500u   /* once switched, an LED is guaranteed to stay on at least this long */

/* ---------- Shared with the timer ISR ---------- */
volatile uint32_t ms = 0;   /* free-running millisecond counter -- the ONLY thing the ISR touches */

ISR(TIMER1_COMPA_vect) {
    ms++;
}

/* ---------- Helpers ---------- */
static uint32_t read_ms(void) {
    uint32_t v;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        v = ms;
    }
    return v;
}

static inline uint8_t pin_high(uint8_t s) {
    if (s == 0) return (PIND & (1 << PD2)) ? 1 : 0;
    return (PIND & (1 << PD3)) ? 1 : 0;
}

static void set_leds(uint8_t s) {
    if (s == 0) {
        PORTA |=  (1 << LED0_PA0);
        PORTA &= ~(1 << LED1_PA7);
    } else {
        PORTA &= ~(1 << LED0_PA0);
        PORTA |=  (1 << LED1_PA7);
    }
    PORTB &= ~(1 << LED2_PB0); /* spare, always off for now */
}

int main(void) {
    /* LEDs as outputs, start low */
    DDRA  |= (1 << LED0_PA0) | (1 << LED1_PA7);
    DDRB  |= (1 << LED2_PB0);
    PORTA &= ~((1 << LED0_PA0) | (1 << LED1_PA7));
    PORTB &= ~(1 << LED2_PB0);

    /* PIR inputs (PIR modules drive the line themselves, no pull-up needed) */
    DDRD  &= ~((1 << PD2) | (1 << PD3));
    PORTD &= ~((1 << PD2) | (1 << PD3));

    /* Timer1 CTC, 1 ms tick -- interrupts are only used for this clock now,
       INT0/INT1 are not used at all, so there's nothing left for an ISR
       and the main loop to race over. */
    TCCR1A = 0;
    OCR1A  = (uint16_t)(F_CPU / 8UL / 1000UL - 1UL);
    TCCR1B = (1 << WGM12) | (1 << CS11);   /* CTC, prescaler = 8 */
    TIMSK |= (1 << OCIE1A);

    sei();

    uint8_t  last_sensor      = 0;        /* starts at 0 so an LED is always lit */
    uint32_t rising_since[2]  = {0, 0};   /* ms timestamp each pin last went LOW->HIGH */
    uint8_t  prev_level[2]    = {0, 0};   /* previous polled level, to detect that edge */
    uint32_t refractory_until = 0;

    for (;;) {
        uint32_t now = read_ms();

        /* track how long each pin has been continuously HIGH */
        for (uint8_t s = 0; s < 2; s++) {
            uint8_t level = pin_high(s);
            if (level && !prev_level[s]) {
                rising_since[s] = now;   /* just went high: (re)start the debounce clock */
            }
            prev_level[s] = level;
        }

        /* switch only to a DIFFERENT sensor that has been solidly HIGH
           for the full debounce window, and only once refractory has
           elapsed. last_sensor is written in exactly this one place. */
        if (now >= refractory_until) {
            for (uint8_t s = 0; s < 2; s++) {
                if (s == last_sensor) continue;
                if (pin_high(s) && (now - rising_since[s]) >= DEBOUNCE_MS) {
                    last_sensor = s;
                    refractory_until = now + REFRACTORY_MS;
                    break;
                }
            }
        }

        set_leds(last_sensor);
    }
}