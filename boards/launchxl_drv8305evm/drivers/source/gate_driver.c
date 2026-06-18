#include "gate_driver.h"

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

    obj->enableGpio = 0U;
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

    GPIO_writePin(obj->enableGpio, 1U);
    obj->enabled = true;

    return;
}

void GATE_DRIVER_disable(GATE_DRIVER_Handle handle)
{
    GATE_DRIVER_Obj *obj = (GATE_DRIVER_Obj *)handle;

    GPIO_writePin(obj->enableGpio, 0U);
    obj->enabled = false;

    return;
}

bool GATE_DRIVER_isEnabled(GATE_DRIVER_Handle handle)
{
    GATE_DRIVER_Obj *obj = (GATE_DRIVER_Obj *)handle;

    return(obj->enabled);
}
