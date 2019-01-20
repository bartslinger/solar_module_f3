#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

uint32_t get_PV_mV(uint16_t adc);

#ifdef __cplusplus
}
#endif

#endif // MEASUREMENT_H
