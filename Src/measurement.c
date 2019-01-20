#include "measurement.h"

uint32_t get_PV_mV(uint16_t adc)
{
  float PV_V = ((float)(adc)) * 0.003591103465567194f;
  return (uint32_t)(PV_V * 1000.0f);
}
