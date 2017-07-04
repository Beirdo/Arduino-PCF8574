/*
 * See header file for details
 *
 *  This program is free software: you can redistribute it and/or modify\n
 *  it under the terms of the GNU General Public License as published by\n
 *  the Free Software Foundation, either version 3 of the License, or\n
 *  (at your option) any later version.\n
 * 
 *  This program is distributed in the hope that it will be useful,\n
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of\n
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n
 *  GNU General Public License for more details.\n
 * 
 *  You should have received a copy of the GNU General Public License\n
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.\n
 */

/* Dependencies */
#include <Wire.h>
#include "PCF8574.h"
#ifdef PCF8574_INTERRUPT_SUPPORT
#include "PCint.h"
#endif
#include <inttypes.h>

PCF8574::PCF8574() : _PORT(0), _PIN(0), _DDR(0), _address(0)
{
#ifdef PCF8574_INTERRUPT_SUPPORT
    _oldPIN = 0;
    _isrIgnore = 0;
    _pcintPin = 0;
#endif
}

void PCF8574::begin(uint8_t address)
{
    /* Store the I2C address and init the Wire library */
    _address = address;
    Wire.begin();
    readGPIO();
}

void PCF8574::pinMode(uint8_t pin, uint8_t mode, bool update)
{
    /* Switch according mode */
    switch (mode) {
        case INPUT:
            _DDR &= ~(1 << pin);
            _PORT &= ~(1 << pin);
            break;

        case INPUT_PULLUP:
            _DDR &= ~(1 << pin);
            _PORT |= (1 << pin);
            break;

        case OUTPUT:
            _DDR |= (1 << pin);
            _PORT &= ~(1 << pin);
            break;

        default:
            break;
    }

    if (update) {
        /* Update GPIO values */
        updateGPIO();
    }
}

uint8_t PCF8574::digitalWrite(uint8_t pin, uint8_t value)
{
    /* Set PORT bit value */
    if (value) {
        _PORT |= (1 << pin);
    } else {
        _PORT &= ~(1 << pin);
    }

    /* Update GPIO values */
    return updateGPIO();
}

uint8_t PCF8574::digitalRead(uint8_t pin)
{
    /* Read GPIO */
    readGPIO();

#ifdef PCF8574_INTERRUPT_SUPPORT
    /* Check for interrupt (manual detection) */
    // checkForInterrupt();
#endif

    /* Read and return the pin state */
    return (_PIN & (1 << pin)) ? HIGH : LOW;
}

uint8_t PCF8574::write(uint8_t value)
{
    /* Store pins values and apply */
    _PORT = value;

    /* Update GPIO values */
    return updateGPIO();
}

uint8_t PCF8574::read()
{
    /* Read GPIO */
    readGPIO();

#ifdef PCF8574_INTERRUPT_SUPPORT
    /* Check for interrupt (manual detection) */
    // checkForInterrupt();
#endif

    /* Return current pins values */
    return _PIN;
}

void PCF8574::pullUp(uint8_t pin)
{
    /* Same as pinMode(INPUT_PULLUP) */
    pinMode(pin, INPUT_PULLUP); // /!\ pinMode form THE LIBRARY
}

void PCF8574::pullDown(uint8_t pin)
{
    /* Same as pinMode(INPUT) */
    pinMode(pin, INPUT); // /!\ pinMode form THE LIBRARY
}

uint8_t PCF8574::clear()
{
    /* User friendly wrapper for write() */
    return write(0x00);
}

uint8_t PCF8574::set()
{
    /* User friendly wrapper for write() */
    return write(0xFF);
}

uint8_t PCF8574::toggle(uint8_t pin)
{
    /* Toggle pin state */
    _PORT ^= (1 << pin);

    /* Update GPIO values */
    return updateGPIO();
}

void PCF8574::blink(uint8_t pin, uint16_t count, uint32_t duration)
{
    /* Compute steps duration */
    duration /= count * 2;

    /* Loop n times */
    while (count--) {
        /* Toggle pin 2 times */
        toggle(pin);
        delay(duration);
        toggle(pin);
        delay(duration);
    }
}

#ifdef PCF8574_INTERRUPT_SUPPORT
void PCF8574::enableInterrupt(uint8_t pin, void (*selfCheckFunction)(void))
{
    /* Store interrupt pin number */
    _pcintPin = pin;

    /* Setup interrupt pin */
    ::pinMode(pin, INPUT_PULLUP); // /!\ pinMode form THE ARDUINO CORE

    /* Attach interrupt handler */
    PCattachInterrupt(pin, selfCheckFunction, FALLING);
}

void PCF8574::disableInterrupt()
{
    /* Detach interrupt handler */
    PCdetachInterrupt(_pcintPin);
}

void PCF8574::checkForInterrupt()
{
    /* Avoid nested interrupt triggered by I2C read/write */
    if(_isrIgnore) {
        return;
    }
    
    _isrIgnore = 1;
        
    /* Re-enable interrupts to allow Wire library to work */
    sei();

    /* Read current pins values */
    readGPIO();

    /* Check all pins */
    for (uint8_t i = 0; i < 8; ++i) {
        /* Check for interrupt handler */
        if (!_intCallback[i]) {
            continue;
        }

        uint8_t mask = (1 << i);
        bool pending = false;

        /* Check for interrupt event */
        switch (_intMode[i]) {
            case CHANGE:
                pending = !(!(mask & (_PIN ^ _oldPIN)));
                break;

            case LOW:
                pending = !(mask & _PIN);
                break;

            case FALLING:
                pending = (mask & _oldPIN) && !(mask & _PIN);
                break;

            case RISING:
                pending =  !(mask & _oldPIN) && (mask & _PIN);
                break;
        }

        if (pending) {
            _intCallback[i]();
        }
    }
    
    /* Turn off ISR ignore flag */
    _isrIgnore = 0;
}

void PCF8574::attachInterrupt(uint8_t pin, void (*userFunc)(void), uint8_t mode)
{
    /* Store interrupt mode and callback */
    _intMode[pin] = mode;
    _intCallback[pin] = userFunc;
}

void PCF8574::detachInterrupt(uint8_t pin)
{
    /* Void interrupt handler */
    _intCallback[pin] = 0;
}
#endif

void PCF8574::readGPIO()
{
#ifdef PCF8574_INTERRUPT_SUPPORT
    /* Store old _PIN value */
    _oldPIN = _PIN;
#endif

    /* Start request, wait for data and receive GPIO values as byte */
    Wire.setClock(100000);
    Wire.requestFrom(_address, (uint8_t) 0x01);
    while (Wire.available() < 1);
    _PIN = Wire.read();
}

uint8_t PCF8574::updateGPIO()
{
    uint8_t value = (_PIN & ~_DDR) | _PORT;

    /* Start communication and send GPIO values as byte */
    Wire.setClock(100000);
    Wire.beginTransmission(_address);
    Wire.write(value);
    return Wire.endTransmission();
}


// vim:ts=4:sw=4:ai:et:si:sts=4
