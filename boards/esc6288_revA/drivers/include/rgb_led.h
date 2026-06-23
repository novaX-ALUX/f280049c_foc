/*
 * rgb_led.h - WS2812-style RGB status LED for esc6288_revA.
 *
 * Drives the on-board XL-1010RGBC (RGB1) and the GH3 external LED connector through the
 * SN74LVC1T45 level shifter (U6) from GPIO12. Single data line, GRB order, MSB first.
 *
 * GPIO12 has NO SPI function on this device (it is EPWM7A / CANB_TX / GPIO), so this is a
 * bit-banged driver. The bit timing is approximate and MUST be verified/tuned with a scope
 * on the bench (see rgb_led.c). Sufficient for a status indicator on the first LED.
 */
#ifndef ESC6288_RGB_LED_H
#define ESC6288_RGB_LED_H

#include <stdint.h>

//! \brief Re-mux GPIO12 to a GPIO output and clear the LED.
void RGB_init(void);

//! \brief Set the first LED (RGB1) to (r,g,b) and latch. Masks interrupts during the frame.
void RGB_setColor(uint8_t r, uint8_t g, uint8_t b);

#endif /* ESC6288_RGB_LED_H */
