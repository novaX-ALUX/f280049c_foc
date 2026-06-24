#include "gate_driver.h"
#include "board.h"

// BOOSTXL-3PhGaNInv gate "enable" is the SN74AVC8T245 buffer output-enable (nEn_uC), which is
// ACTIVE-LOW: drive the pin LOW to pass PWM to the LMG5200s; HIGH (or released, held by the
// on-board pull-up R30) puts the buffer in Hi-Z and the on-board pull-downs hold both FETs off.
// Hence enable() writes 0 and disable() writes 1 -- inverted vs an active-high EN_GATE.
// The enable GPIO defaults to BOARD_GPIO_NONE so writes are hard-skipped until HAL_setupGate()
// binds the pin, and the safe-off state (pin HIGH) is established by HAL_setupGPIOs().

GATE_DRIVER_Handle GATE_DRIVER_init(void *pMemory, const size_t numBytes)
{
    GATE_DRIVER_Handle handle;
    GATE_DRIVER_Obj *obj;

    if(numBytes < sizeof(GATE_DRIVER_Obj))
    {
        return((GATE_DRIVER_Handle)NULL);
    }

    handle = (GATE_DRIVER_Handle)pMemory;
    obj = (GATE_DRIVER_Obj *)handle;

    obj->enableGpio = BOARD_GPIO_NONE;   // safe default until HAL_setupGate() binds it
    obj->enabled = false;

    return(handle);
}

void GATE_DRIVER_setEnableGPIO(GATE_DRIVER_Handle handle,
                               const uint32_t gpioNumber)
{
    GATE_DRIVER_Obj *obj = (GATE_DRIVER_Obj *)handle;

    obj->enableGpio = gpioNumber;

    return;
}

void GATE_DRIVER_enable(GATE_DRIVER_Handle handle)
{
    GATE_DRIVER_Obj *obj = (GATE_DRIVER_Obj *)handle;

    if(obj->enableGpio != BOARD_GPIO_NONE)
    {
        GPIO_writePin(obj->enableGpio, 0U);   // active-low: 0 = buffer enabled (PWM passes)
    }
    obj->enabled = true;

    return;
}

void GATE_DRIVER_disable(GATE_DRIVER_Handle handle)
{
    GATE_DRIVER_Obj *obj = (GATE_DRIVER_Obj *)handle;

    if(obj->enableGpio != BOARD_GPIO_NONE)
    {
        GPIO_writePin(obj->enableGpio, 1U);   // active-low: 1 = buffer Hi-Z (safe-off)
    }
    obj->enabled = false;

    return;
}

bool GATE_DRIVER_isEnabled(GATE_DRIVER_Handle handle)
{
    GATE_DRIVER_Obj *obj = (GATE_DRIVER_Obj *)handle;

    return(obj->enabled);
}
