/*
 * Fan PWM proxy / remapper for ATtiny85 (Digispark)
 *
 *   Motherboard (PWM, pin 4) ---> PB2 / P2   (INT0, input PWM measurement)
 *   PB1 / P1 (OC1A, Timer1)  ---> fan (PWM, pin 4)   25 kHz
 *
 *   PB3/P3 and PB4/P4 are used by the Micronucleus USB bootloader - don't touch.
 *   Timer0 = free-running timer for measurement (Arduino millis()/delay() do NOT work!)
 *   Timer1 = hardware PWM output.
 */
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

// ============================ SETTINGS ============================

#define MEASURE_WINDOW_MS 100   // input PWM averaging window (multiple of 10)
#define OUT_PWM_HZ        25000UL
#define STARTUP_OUTPUT    100   // output % before the first measurement
#define DIRECTION_DEADBAND 2    // input must move back by >= N % to count as a change of direction

// ----------------------- MAPPING TABLE ------------------------------
// Fan behavior on GenMachine YI7000LP
// Temperature     Action
// 0-39            PWM 0%
// 40              no change (HOLD)
// 41-49           30%
// 50-55           HOLD
// 56-59           40%
// 60-65           HOLD
// 66-74           70%
// 75-80           HOLD
// 81 and above    100%

// Two separate tables: input PWM (%) from the motherboard -> output PWM (%) to the fan.
//   RISE - used while the input PWM is going UP
//   FALL - used while the input PWM is going DOWN
// Bounds are inclusive. Rules are checked top to bottom, first match wins.
// If no rule matches (a gap in the table), the output is HELD at its current value.
struct Rule { uint8_t in_from; uint8_t in_to; uint8_t out; };

static const Rule RISE[] PROGMEM = {
    //  from   to   output
    {   0,    39,     0 },
    {  40,    69,    30 },
    {  70,    80,    60 },
    {  81,   100,   100 },
};

static const Rule FALL[] PROGMEM = {
    //  from   to   output
    {   0,    29,     0 },
    {  30,    69,    30 },
    {  70,    74,    60 },
    {  75,   100,   100 },   // stays at 100% on the way down until input < 75
};

// ===================================================================

#define PIN_IN   PB2   // INT0
#define PIN_OUT  PB1   // OC1A

// Timer1: clocked from CPU via /4, TOP = OCR1C
//  16.5 MHz / 4 / 165 = 25.000 kHz (165 duty steps, ~0.6% each)
#define T1_DIV   4UL
#define PWM_TOP  ((F_CPU / T1_DIV / OUT_PWM_HZ) - 1)
static_assert(PWM_TOP >= 20 && PWM_TOP <= 255, "PWM_TOP out of range");

// If fewer edges than this are seen in a window, the signal is treated as static (0% or 100%)
#define MIN_EDGES 20

// ----------------------- input PWM measurement ----------------------
static volatile uint32_t g_high = 0;     // sum of "1" durations (Timer0 ticks)
static volatile uint32_t g_low  = 0;     // sum of "0" durations
static volatile uint16_t g_edges = 0;
static volatile uint8_t  g_last_t = 0;
static volatile uint8_t  g_have_last = 0;

// Timer0: /8 -> 0.485 us/tick, 8 bit -> up to ~124 us between edges
// (i.e. works for input PWM above ~8.5 kHz, the standard 25 kHz is fine)
ISR(INT0_vect)
{
    uint8_t now   = TCNT0;
    uint8_t level = PINB & _BV(PIN_IN);          // level AFTER the edge
    uint8_t dt    = (uint8_t)(now - g_last_t);
    g_last_t = now;

    if (!g_have_last) { g_have_last = 1; return; }

    if (level) g_low  += dt;   // rising edge: a "0" phase just ended
    else       g_high += dt;   // falling edge: a "1" phase just ended
    g_edges++;
}

// ----------------------- output PWM ---------------------------------
static void set_output(uint8_t pct)
{
    if (pct > 100) pct = 100;
    uint16_t c = ((uint16_t)pct * (PWM_TOP + 1) + 50) / 100;
    if (c > PWM_TOP) c = PWM_TOP;   // OCR1A == OCR1C -> constant "1" (100%)
    OCR1A = (uint8_t)c;             // 0 -> constant "0"
}

// Returns the output % for the first matching rule, or -1 if none matches (HOLD)
static int16_t lookup(const Rule *table, uint8_t n, uint8_t in)
{
    for (uint8_t i = 0; i < n; i++) {
        if (in >= pgm_read_byte(&table[i].in_from) &&
            in <= pgm_read_byte(&table[i].in_to))
            return pgm_read_byte(&table[i].out);
    }
    return -1;
}

// --------------------------------------------------------------------
void setup()
{
    cli();

    // pins
    DDRB  |=  _BV(PIN_OUT);
    PORTB &= ~_BV(PIN_OUT);
    DDRB  &= ~_BV(PIN_IN);
    PORTB |=  _BV(PIN_IN);                  // internal pull-up on the input

    // disable everything the Arduino core configured
    TIMSK = 0;
    PLLCSR = 0;                             // Timer1 runs from the CPU clock, not PLL
    GTCCR = 0;

    // Timer0: free-running counter, /8
    TCCR0A = 0;
    TCCR0B = _BV(CS01);
    TCNT0  = 0;

    // Timer1: PWM on OC1A (PB1), 25 kHz
    OCR1C  = (uint8_t)PWM_TOP;
    TCNT1  = 0;
    TCCR1  = _BV(PWM1A) | _BV(COM1A1) | _BV(CS11) | _BV(CS10);   // CK/4
    set_output(STARTUP_OUTPUT);

    // INT0: any logical change on PB2
    MCUCR = (MCUCR & ~(_BV(ISC01) | _BV(ISC00))) | _BV(ISC00);
    GIFR  = _BV(INTF0);
    GIMSK |= _BV(INT0);

    sei();
}

void loop()
{
    static bool    first  = true;
    static bool    rising = true;   // current direction of the input PWM
    static uint8_t ref    = 0;      // furthest input value reached in the current direction

    // wait for the measurement window (interrupts keep running, delay just stretches a bit)
    for (uint8_t i = 0; i < MEASURE_WINDOW_MS / 10; i++) _delay_ms(10);

    uint32_t h, l;
    uint16_t e;
    cli();
    h = g_high; l = g_low; e = g_edges;
    g_high = 0; g_low = 0; g_edges = 0;
    sei();

    uint32_t tot = h + l;
    uint8_t in_pct;
    if (e < MIN_EDGES || tot == 0)
        in_pct = (PINB & _BV(PIN_IN)) ? 100 : 0;      // no edges: constant level
    else
        in_pct = (uint8_t)((h * 100UL + tot / 2) / tot);

    // direction tracking: ref follows the input in the current direction;
    // the direction flips only after the input moves back by >= DIRECTION_DEADBAND
    if (first) { ref = in_pct; first = false; }
    if (rising) {
        if (in_pct > ref) ref = in_pct;
        else if (in_pct + DIRECTION_DEADBAND <= ref) { rising = false; ref = in_pct; }
    } else {
        if (in_pct < ref) ref = in_pct;
        else if (in_pct >= ref + DIRECTION_DEADBAND) { rising = true; ref = in_pct; }
    }

    int16_t out = rising
        ? lookup(RISE, sizeof(RISE) / sizeof(RISE[0]), in_pct)
        : lookup(FALL, sizeof(FALL) / sizeof(FALL[0]), in_pct);
    if (out >= 0) set_output((uint8_t)out);   // no match -> hold the current output
}