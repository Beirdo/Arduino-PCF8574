#ifndef PCINT_H__
#define PCINT_H__

#include "pins_arduino.h"

volatile uint8_t *port_to_pcmask[] = { &PCMSK0, &PCMSK1, &PCMSK2 };

typedef void (*voidFuncPtr)(void);

typedef struct {
    uint8_t intMode;
    volatile voidFuncPtr userFunc;
} PCint_t;

uint8_t PCintIndex[24] = {};    // contains index (1-based) into the PCintTable

#define MAX_PCINT_COUNT 4
PCint_t PCintTable[MAX_PCINT_COUNT];  // limit pins supported for memory reasons
uint8_t PCintNext = 0;          // next (0-based) index to use

volatile static uint8_t PCintLast[3];

/*
 * attach an interrupt to a specific pin using pin change interrupts.
 */
void PCattachInterrupt(uint8_t pin, void (*userFunc)(void), int mode)
{
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    uint8_t slot;
    volatile uint8_t *pcmask;

    // map pin to PCIR register
    if (port == NOT_A_PORT) {
        return;
    } else {
        port -= 2;
        pcmask = port_to_pcmask[port];
    }

    // deal with analog ports
    if (port == 1) {
        slot = port * 8 + (pin - 14);
    } else {
        slot = port * 8 + (pin % 8);
    }

    if (PCintNext >= MAX_PCINT_COUNT) {
        return;
    }

    PCint_t *interrupt = &PCintTable[PCintNext];
    interrupt->intMode = mode;
    interrupt->userFunc = userFunc;

    PCintIndex[slot] = ++PCintNext;
 
    // set the mask
    *pcmask |= bit;
    // enable the interrupt
    PCICR |= 0x01 << port;
    
    // Fix init by SkyWodd
    PCintLast[0] = *portInputRegister(2);
    PCintLast[1] = *portInputRegister(3);
    PCintLast[2] = *portInputRegister(4);
}

void PCdetachInterrupt(uint8_t pin)
{
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    volatile uint8_t *pcmask;

    // map pin to PCIR register
    if (port == NOT_A_PORT) {
        return;
    } else {
        port -= 2;
        pcmask = port_to_pcmask[port];
    }

    // disable the mask.
    *pcmask &= ~bit;
    // if that's the last one, disable the interrupt.
    if (*pcmask == 0) {
        PCICR &= ~(0x01 << port);
    }
}

// common code for isr handler. "port" is the PCINT number.
// there isn't really a good way to back-map ports and masks to pins.
static void PCint(uint8_t port)
{
    uint8_t bit;
    uint8_t curr;
    uint8_t mask;
    uint8_t pin;

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
            pin = port * 8 + i;

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
    PCint(0);
}

SIGNAL(PCINT1_vect)
{
    PCint(1);
}

SIGNAL(PCINT2_vect)
{
    PCint(2);
}

#endif
 
// vim:ts=4:sw=4:ai:et:si:sts=4
