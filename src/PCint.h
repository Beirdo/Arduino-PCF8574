#ifndef PCINT_H__
#define PCINT_H__

#include <avr/pgmspace.h>
#include "pins_arduino.h"

static PROGMEM const uint8_t pcint_to_pin[] = {
    // PCMSK0 - Port B
    8, 9, 10, 11, 12, 13, 20, 21,
    // PCMSK1 - Port C
    23, 24, 25, 26, 27, 28, 22, 0xFF,
    // PCMSK2 - Port D
    0, 1, 2, 3, 4, 5, 6, 7,
#if 0
    // PCMSK3 - Port E
    23, 24, 25, 26
#endif
};

volatile uint8_t *port_to_pcmask[] = { &PCMSK0, &PCMSK1, &PCMSK2 };

typedef void (*voidFuncPtr)(void);

typedef struct {
    uint8_t intMode;
    volatile voidFuncPtr userFunc;
} PCint_t;

// contains index (1-based) into the PCintTable
uint8_t PCintIndex[NUM_DIGITAL_PINS] = {};

#define MAX_PCINT_COUNT 4
PCint_t PCintTable[MAX_PCINT_COUNT];  // limit pins supported for memory reasons
uint8_t PCintNext = 0;          // next (0-based) index to use

volatile static uint8_t PCintLast[3];

/*
 * attach an interrupt to a specific pin using pin change interrupts.
 */
void PCattachInterrupt(uint8_t pin, void (*userFunc)(void), int mode)
{
    uint8_t port = digitalPinToPort(pin);
    if (port == NOT_A_PORT) {
        return;
    }

    volatile uint8_t *pcicr = digitalPinToPCICR(pin);
    if (!pcicr) {
        return;
    }

    if (PCintNext >= MAX_PCINT_COUNT) {
        return;
    }

    PCint_t *interrupt = &PCintTable[PCintNext];
    interrupt->intMode = mode;
    interrupt->userFunc = userFunc;

    PCintIndex[pin] = ++PCintNext;
 
    // set the mask
    volatile uint8_t *pcmask = digitalPinToPCMSK(pin);
    *pcmask |= digitalPinToPCMSKbit(pin);

    // enable the interrupt
    *pcicr |= digitalPinToPCICRbit(pin);
    
    PCintLast[0] = *portInputRegister(2);   // PORTB
    PCintLast[1] = *portInputRegister(3);   // PORTC
    PCintLast[2] = *portInputRegister(4);   // PORTD
}

void PCdetachInterrupt(uint8_t pin)
{
    uint8_t port = digitalPinToPort(pin);
    if (port == NOT_A_PORT) {
        return;
    }

    volatile uint8_t *pcicr = digitalPinToPCICR(pin);
    if (!pcicr) {
        return;
    }

    // disable the mask.
    volatile uint8_t *pcmask = digitalPinToPCMSK(pin);
    *pcmask &= ~digitalPinToPCMSKbit(pin);

    // if that's the last one, disable the interrupt.
    if (*pcmask == 0) {
        *pcicr &= ~digitalPinToPCICRbit(pin);
    }
}

// common code for isr handler. "port" is the PCINT number.
static void PCint(uint8_t port)
{
    uint8_t bit;
    uint8_t curr;
    uint8_t mask;
    uint8_t pin;
    uint8_t pcint;

    // get the pin states for the indicated port.
    curr = *portInputRegister(port + 2);
    mask = curr ^ PCintLast[port];
    PCintLast[port] = curr;

    // mask is pins that have changed. screen out non pcint pins.
    if ((mask &= *port_to_pcmask[port]) == 0) {
        return;
    }

    // mask is pcint pins that have changed.
    for (uint8_t i = 0; i < 8; i++) {
        bit = 0x01 << i;
        if (bit & mask) {
            pcint = port * 8 + i;
            pin = pgm_read_byte(pcint_to_pin + pcint);
            if (pin == 0xFF) {
                continue;
            }

            // Trigger interrupt if mode is CHANGE, or if mode is RISING and
            // the bit is currently high, or if mode is FALLING and bit is low.
            if (PCintIndex[pin]) {
                PCint_t *interrupt = &PCintTable[PCintIndex[pin] - 1];

                if (interrupt->intMode == CHANGE ||
                    (interrupt->intMode == RISING && (curr & bit)) ||
                    (interrupt->intMode == FALLING && !(curr & bit))) {
                    if (interrupt->userFunc) {
                        interrupt->userFunc();
                    }
                }
            }
        }
    }
}

SIGNAL(PCINT0_vect)
{
    // Port B
    PCint(0);
}

SIGNAL(PCINT1_vect)
{
    // Port C
    PCint(1);
}

SIGNAL(PCINT2_vect)
{
    // Port D
    PCint(2);
}

#endif
 
// vim:ts=4:sw=4:ai:et:si:sts=4
