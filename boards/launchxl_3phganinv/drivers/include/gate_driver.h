#ifndef GATE_DRIVER_H
#define GATE_DRIVER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driverlib.h"

typedef struct _GATE_DRIVER_Obj_
{
    uint32_t enableGpio;
    bool enabled;
} GATE_DRIVER_Obj;

typedef struct _GATE_DRIVER_Obj_ *GATE_DRIVER_Handle;

extern GATE_DRIVER_Handle GATE_DRIVER_init(void *pMemory, const size_t numBytes);
extern void GATE_DRIVER_setEnableGPIO(GATE_DRIVER_Handle handle,
                                      const uint32_t gpioNumber);
extern void GATE_DRIVER_enable(GATE_DRIVER_Handle handle);
extern void GATE_DRIVER_disable(GATE_DRIVER_Handle handle);
extern bool GATE_DRIVER_isEnabled(GATE_DRIVER_Handle handle);

#endif // GATE_DRIVER_H
