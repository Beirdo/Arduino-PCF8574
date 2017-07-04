/**
 * @brief PCF8574 arduino library
 * @author SkyWodd <skywodd@gmail.com>
 * @version 2.0
 * @link http://skyduino.wordpress.com/
 *
 * @section intro_sec Introduction
 * This class is designed to allow user to use PCF8574 gpio expander like standard arduino pins.\n
 * This class provides standard Arduino functions like pinMode, digitalWrite,
 * digitalRead, ...\n
 */
#ifndef PCF8574_H
#define PCF8574_H

#include <stdint.h>

/** Comment this define to disable interrupt support */
#define PCF8574_INTERRUPT_SUPPORT

#include <Arduino.h>
#include <inttypes.h>

/**
 * @brief PCF8574 Arduino class
 */
class PCF8574 {
public:
    /**
     * Create a new PCF8574 instance
     */
    PCF8574();

    /**
     * Start the I2C controller and store the PCF8574 chip address
     */
    void begin(uint8_t address = 0x21);

    /**
     * Set the direction of a pin (OUTPUT, INPUT or INPUT_PULLUP)
     * 
     * @param pin The pin to set
     * @param mode The new mode of the pin
     * @remarks INPUT_PULLUP does physicaly the same thing as INPUT (no software pull-up resistors available) but is REQUIRED if you use external pull-up resistor
     */
    void pinMode(uint8_t pin, uint8_t mode, bool update = true);

    /**
     * Set the state of a pin (HIGH or LOW)
     * 
     * @param pin The pin to set
     * @param value The new state of the pin
     * @remarks Software pull-up resistors are not available on the PCF8574
     *
     * Output   0 .. success
     *          1 .. length to long for buffer
     *          2 .. address send, NACK received
     *          3 .. data send, NACK received
     *          4 .. other twi error (lost bus arbitration, bus error, ..)
     */
    uint8_t digitalWrite(uint8_t pin, uint8_t value);

    /**
     * Read the state of a pin
     * 
     * @param pin The pin to read back
     * @return
     */
    uint8_t digitalRead(uint8_t pin);

    /**
     * Set the state of all pins in one go
     * 
     * @param value The new value of all pins (1 bit = 1 pin, '1' = HIGH, '0' = LOW)
     *
     * Output   0 .. success
     *          1 .. length to long for buffer
     *          2 .. address send, NACK received
     *          3 .. data send, NACK received
     *          4 .. other twi error (lost bus arbitration, bus error, ..)
     */
    uint8_t write(uint8_t value);

    /**
     * Read the state of all pins in one go
     * 
     * @return The current value of all pins (1 bit = 1 pin, '1' = HIGH, '0' = LOW)
     */
    uint8_t read();

    /**
     * Exactly like write(0x00), set all pins to LOW
     *
     * Output   0 .. success
     *          1 .. length to long for buffer
     *          2 .. address send, NACK received
     *          3 .. data send, NACK received
     *          4 .. other twi error (lost bus arbitration, bus error, ..)
     */
    uint8_t clear();

    /**
     * Exactly like write(0xFF), set all pins to HIGH
     *
     * Output   0 .. success
     *          1 .. length to long for buffer
     *          2 .. address send, NACK received
     *          3 .. data send, NACK received
     *          4 .. other twi error (lost bus arbitration, bus error, ..)
     */
    uint8_t set();

    /**
     * Toggle the state of a pin
     *
     * Output   0 .. success
     *          1 .. length to long for buffer
     *          2 .. address send, NACK received
     *          3 .. data send, NACK received
     *          4 .. other twi error (lost bus arbitration, bus error, ..)
     */
    uint8_t toggle(uint8_t pin);

    /**
     * Mark a pin as "pulled up"
     * 
     * @warning DO NOTHING - FOR RETRO COMPATIBILITY PURPOSE ONLY
     * @deprecated
     * @param pin Pin the mark as "pulled up"
     */
    void pullUp(uint8_t pin);

    /**
     * Mark a pin as "pulled down"
     * 
     * @warning DO NOTHING - FOR RETRO COMPATIBILITY PURPOSE ONLY
     * @deprecated
     * @param pin Pin the mark as "pulled down"
     */
    void pullDown(uint8_t pin);

    /**
     * Make a pin blink N times for T duration
     * 
     * @warning Blocking function, not recommended for new code
     * @deprecated
     * @param pin The pin to blink
     * @param count The number of ON/OFF couples to execute
     * @param duration The duration of the whole blink action in milliseconds
     */
    void blink(uint8_t pin, uint16_t count, uint32_t duration);

#ifdef PCF8574_INTERRUPT_SUPPORT
    /**
     * Enable interrupts support and setup interrupts handler
     * 
     * @remarks Any pin can be used as "INT" pin, internally the library use PCINT to work.
     * @warning The check wrapping routine must be provided by user and define in the global scope space.
     * @param pin The pin OF YOUR ARDUINO (not the PCF8574) to use as "INT" pin for interrupts detection
     * @param selfCheckFunction The wrapping routine used to process interrupts events.
     * @remarks For best performances you should avoid this "user friendly" fonction and use the standard attachInterrupt() fonction ;)
     * @remarks If multiple PCF8574 are wired on the same "INT" pin this function should be called only one time
     */
    void enableInterrupt(uint8_t pin, void (*selfCheckFunction)(void));

    /**
     * Disable interrupts support
     */
    void disableInterrupt();

    /**
     * Check for interrupt and process routine
     * 
     * @remarks Call this routine from your wrapping routine to detect and process interrupts (if any) of this PCF8574 instance.
     */
    void checkForInterrupt();

    /**
     * Attach a function to an interrupt event of a pin of the PCF8574
     * 
     * @param pin The pin to attach the interrupt event on
     * @param userFunc The callback function to call when the interrupt event is triggered
     * @param mode The interrupt mode to check for, only interrupts events coming from the specified pin and with the specified mode will call the callback function.
     * @remarks 1 PCF8574 pin = 1 interrupt, multiple interrupts on the same pin is not supported
     */
    void attachInterrupt(uint8_t pin, void (*userFunc)(void), uint8_t mode);

    /**
     * Detach any interrupt attached to the specified pin
     * 
     * @param pin The pin to detach any interrupt from
     */
    void detachInterrupt(uint8_t pin);
#endif

protected:
    /** Output pins values */
    volatile uint8_t _PORT;

    /** Current input pins values */
    volatile uint8_t _PIN;

    /** Pins modes values (OUTPUT or INPUT) */
    volatile uint8_t _DDR;

    /** PCF8574 I2C address */
    uint8_t _address;

#ifdef PCF8574_INTERRUPT_SUPPORT
    /** Old value of _PIN variable */
    volatile uint8_t _oldPIN;
    
    /** ISR ignore flag */
    volatile uint8_t _isrIgnore;

    /** PCINT pin used for "INT" pin handling */
    uint8_t _pcintPin;

    /** Interrupts modes of pins (LOW, CHANGE, FALLING, RISING)  */
    uint8_t _intMode[8];

    /** Interrupts callback functions */
    void (*_intCallback[8])(void);
#endif

    /** 
     * Read GPIO states and store them in _PIN variable
     *
     * @remarks Before reading current GPIO states, current _PIN variable value is moved to _oldPIN variable
     */
    void readGPIO();

    /** 
     * Write value of _PORT variable to the GPIO
     * 
     * @remarks Only pin marked as OUTPUT are set, for INPUT pins their value are unchanged
     * @warning To work properly (and avoid any states conflicts) readGPIO() MUST be called before call this function !
     *
     * Output   0 .. success
     *          1 .. length to long for buffer
     *          2 .. address send, NACK received
     *          3 .. data send, NACK received
     *          4 .. other twi error (lost bus arbitration, bus error, ..)
     */
    uint8_t updateGPIO();
};

#endif

// vim:ts=4:sw=4:ai:et:si:sts=4
