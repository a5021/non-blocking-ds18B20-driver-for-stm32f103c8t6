/**
 * @file ow_hal.h
 * @brief Hardware Abstraction Layer for the non-blocking 1-Wire engine
 *
 * The 1-Wire layer (onewire.c) and the DS18B20 driver (ds18b20.c) are written
 * against this thin, target-agnostic interface. Each target provides the
 * ow_hal_* implementation as a header of static inline functions, so the
 * abstraction compiles away to exactly the same register writes as a direct
 * bare-metal implementation: zero call overhead in any build mode, with or
 * without LTO, and no function pointers or runtime dispatch.
 *
 * Every ow_hal_* call schedules exactly one hardware-timed operation on the
 * shared timer/DMA engine and returns immediately; the caller advances by
 * polling ow_hal_bus_done(). The CPU is never in the timing-critical path.
 */

#ifndef OW_HAL_H
#define OW_HAL_H

#include <stdint.h>

/* --- Compiler helpers the driver expects (portable stand-ins; real CMSIS
 *     headers define them too, so the #ifndef guards keep both paths
 *     identical) --- */
#ifndef __STATIC_FORCEINLINE
#define __STATIC_FORCEINLINE static __attribute__((always_inline)) inline
#endif
#ifndef __WEAK
#define __WEAK __attribute__((weak))
#endif

/* --- 1-Wire reset timeslot geometry (microseconds), shared by all backends.
 *     The '1'/'0' bit-slot durations live in onewire.h (ONEWIRE_ONE_PULSE,
 *     ONEWIRE_ZERO_PULSE, ONEWIRE_GUARD_BAND). --- */
#define OW_HAL_RESET_PULSE_DURATION 480u
#define OW_HAL_RESET_TIMEOUT 960u
#define OW_HAL_CAPTURE_BUF_SIZE 2u

/* --- Backend selection --- */
#if defined(OW_HAL_TARGET_F1)
#include "ow_hal_f1.h"
#elif defined(OW_HAL_TARGET_F0)
#include "ow_hal_f0.h"
#else
#error "ow_hal: no backend selected (define OW_HAL_TARGET_F1, OW_HAL_TARGET_F0, ...)"
#endif

#endif /* OW_HAL_H */