#include "measurement.h"

uint32_t get_PV_mV(uint16_t adc)
{
  const float R1 = 220.0f;            // [kOhm]
  const float R2 = 68.0f;             // [kOhm]
  const float Vref = 3.3f;            // [V]
  const float ratio = 1000.0f * Vref * (R1 + R2) / (4096.0f * R2);

  return (uint32_t)( (float)adc * ratio );
}

uint32_t get_BATT_mV(uint16_t adc)
{
  return 0;
}

int32_t get_PV_mA(uint16_t adc)
{
  // ACS711 15.5A version
  const float Vref = 3.3f;            // [V]
  const float sensitivity = 90.0f;    // [mV/A]
  const float ratio = 1000.0f * Vref / (4096.0f * sensitivity);

  return (int32_t)( (float)(adc - 2048) * ratio );
}

int32_t get_BATT_mA(uint16_t adc)
{
  // ACS711 31A version
  const float Vref = 3.3f;            // [V]
  const float sensitivity = 45.0f;    // [mV/A]
  const float ratio = 1000.0f * Vref / (4096.0f * sensitivity);

  return (int32_t)( (float)(adc - 2048) * ratio );
}

int32_t get_OUT_mA(uint16_t adc)
{
  return 0;
}
