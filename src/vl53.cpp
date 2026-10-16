#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "vl53.h"

bool vl53_begin(VL53L0X &sensor) {
  sensor.setTimeout(500);

  if (!sensor.init()) {
    Serial.println("VL53L0X no encontrado.");
    return false;
  }

  sensor.startContinuous();
  Serial.println("VL53L0X listo.");
  return true;
}

bool vl53_leer(VL53L0X &sensor, float &distanciaMm) {
  uint16_t milimetros = sensor.readRangeContinuousMillimeters();

  if (sensor.timeoutOccurred() || milimetros >= 8190) {
    return false;
  }

  // Ajuste experimental: valida este valor con tu montaje físico.
  distanciaMm = (milimetros * 0.87f) - 3.0f;

  if (distanciaMm < 0.0f) {
    distanciaMm = 0.0f;
  }

  return true;
}