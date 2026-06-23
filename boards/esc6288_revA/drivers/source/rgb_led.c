#include "device.h"
#include "driverlib.h"
#include "board.h"
#include "rgb_led.h"
#include <stdbool.h>

/*
 * WS2812(B) bit-bang on GPIO12 (-> SN74LVC1T45 -> RGB1 + GH3). One data line, GRB order,
 * MSB first. A '1' is a long high pulse, a '0' a short one; a >50 us low latches the frame.
 *
 * BENCH TODO: these are LOOP counts, not cycles -- each ws_delay() iteration is several
 * SYSCLK cycles (loop overhead). Scope the GPIO12 line and tune WS_*_LOOPS so that, at the
 * 100 MHz SYSCLK, T1H ~= 0.7 us, T0H ~= 0.35 us, and the bit period ~= 1.25 us. WS2812B
 * tolerances are wide (~+/-150 ns), so exact values are not critical once in range.
 */
#define WS_T1H_LOOPS   18U   /* '1' high  (tune to ~0.70 us) */
#define WS_T1L_LOOPS   14U   /* '1' low   (tune to ~0.55 us) */
#define WS_T0H_LOOPS   7U    /* '0' high  (tune to ~0.35 us) */
#define WS_T0L_LOOPS   20U   /* '0' low   (tune to ~0.80 us) */
#define WS_RESET_US    60U   /* >50 us low latch */

static inline void ws_delay(uint16_t loops)
{
    volatile uint16_t i;
    for(i = 0U; i < loops; i++)
    {
        __asm(" NOP");
    }
}

static void ws_send_byte(uint16_t b)
{
    uint16_t bit;

    for(bit = 0U; bit < 8U; bit++)
    {
        if((b & 0x80U) != 0U)
        {
            GPIO_writePin(BOARD_RGB_GPIO, 1U);
            ws_delay(WS_T1H_LOOPS);
            GPIO_writePin(BOARD_RGB_GPIO, 0U);
            ws_delay(WS_T1L_LOOPS);
        }
        else
        {
            GPIO_writePin(BOARD_RGB_GPIO, 1U);
            ws_delay(WS_T0H_LOOPS);
            GPIO_writePin(BOARD_RGB_GPIO, 0U);
            ws_delay(WS_T0L_LOOPS);
        }
        b = (uint16_t)(b << 1);
    }
}

void RGB_init(void)
{
    /* GPIO12 is muxed to EPWM7A by HAL_setupGPIOs; re-mux to plain GPIO output for the
     * bit-bang (the template PWM-DAC that used EPWM7 is disabled on this board). */
    GPIO_setMasterCore(BOARD_RGB_GPIO, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_12_GPIO12);
    GPIO_writePin(BOARD_RGB_GPIO, 0U);
    GPIO_setDirectionMode(BOARD_RGB_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(BOARD_RGB_GPIO, GPIO_PIN_TYPE_STD);

    RGB_setColor(0U, 0U, 0U);
}

void RGB_setColor(uint8_t r, uint8_t g, uint8_t b)
{
    bool was_disabled;

    /* Mask interrupts so an ISR can't stretch a bit and corrupt the frame. */
    was_disabled = Interrupt_disableGlobal();
    ws_send_byte(g);
    ws_send_byte(r);
    ws_send_byte(b);
    if(!was_disabled)
    {
        Interrupt_enableGlobal();
    }

    /* >50 us low latches the data into the LED(s). */
    SysCtl_delay((uint32_t)(WS_RESET_US * (DEVICE_SYSCLK_FREQ / 1000000UL) / 5UL));
}
