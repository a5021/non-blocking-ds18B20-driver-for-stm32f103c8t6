#ifndef MOCK_TARGET_H
#define MOCK_TARGET_H
/* Target dispatcher for the host-test mocks: pulls in the device stand-in
 * matching the backend under test (OW_HAL_TARGET_F0 / OW_HAL_TARGET_F1). */
#if defined(OW_HAL_TARGET_F0)
#include "stm32f0xx.h"
#else
#include "stm32f1xx.h"
#endif
#endif /* MOCK_TARGET_H */
