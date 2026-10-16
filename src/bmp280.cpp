#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "bmp280.h"

namespace {
  constexpr uint8_t BMP280_ADDR = 0x77;

  bool inicializado = false;

  uint16_t dig_T1, dig_P1;
  int16_t dig_T2, dig_T3;
  int16_t dig_P2, dig_P3, dig_P4, dig_P5;
  int16_t dig_P6, dig_P7, dig_P8, dig_P9;

  bool escribir(uint8_t registro, uint8_t valor) {
    Wire.beginTransmission(BMP280_ADDR);
    Wire.write(registro);
    Wire.write(valor);
    return Wire.endTransmission() == 0;
  }

  bool leer8(uint8_t registro, uint8_t &valor) {
    Wire.beginTransmission(BMP280_ADDR);
    Wire.write(registro);

    if (Wire.endTransmission(false) != 0) {
      return false;
    }

    if (Wire.requestFrom(BMP280_ADDR, 1) != 1 || Wire.available() < 1) {
      return false;
    }

    valor = Wire.read();
    return true;
  }

  bool leer16(uint8_t registro, uint16_t &valor) {
    Wire.beginTransmission(BMP280_ADDR);
    Wire.write(registro);

    if (Wire.endTransmission(false) != 0) {
      return false;
    }

    if (Wire.requestFrom(BMP280_ADDR, 2) != 2 || Wire.available() < 2) {
      return false;
    }

    uint8_t bajo = Wire.read();
    uint8_t alto = Wire.read();

    valor = static_cast<uint16_t>(bajo) |
            (static_cast<uint16_t>(alto) << 8);

    return true;
  }

  bool leer16ConSigno(uint8_t registro, int16_t &valor) {
    uint16_t dato;
    if (!leer16(registro, dato)) {
      return false;
    }

    valor = static_cast<int16_t>(dato);
    return true;
  }
}

bool bmp280_begin() {
  uint8_t id;

  if (!leer8(0xD0, id)) {
    Serial.println("BMP280 no responde.");
    return false;
  }

  if (id != 0x58 && id != 0x60) {
    Serial.println("BMP280/BME280 no encontrado.");
    return false;
  }

  if (!leer16(0x88, dig_T1) ||
      !leer16ConSigno(0x8A, dig_T2) ||
      !leer16ConSigno(0x8C, dig_T3) ||
      !leer16(0x8E, dig_P1) ||
      !leer16ConSigno(0x90, dig_P2) ||
      !leer16ConSigno(0x92, dig_P3) ||
      !leer16ConSigno(0x94, dig_P4) ||
      !leer16ConSigno(0x96, dig_P5) ||
      !leer16ConSigno(0x98, dig_P6) ||
      !leer16ConSigno(0x9A, dig_P7) ||
      !leer16ConSigno(0x9C, dig_P8) ||
      !leer16ConSigno(0x9E, dig_P9)) {
    Serial.println("Error leyendo calibracion BMP280.");
    return false;
  }

  // Primero sleep para poder configurar el sensor.
  if (!escribir(0xF4, 0x00)) {
    return false;
  }

  // Standby 1000 ms, filtro apagado.
  if (!escribir(0xF5, 0xA0)) {
    return false;
  }

  // Temperatura x1, presión x1, modo normal.
  if (!escribir(0xF4, 0x27)) {
    return false;
  }

  inicializado = true;
  Serial.println(id == 0x60 ? "BME280 detectado." : "BMP280 listo.");
  return true;
}

bool bmp280_leer(DatosBMP &datos) {
  if (!inicializado) {
    return false;
  }

  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(0xF7);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(BMP280_ADDR, 6) != 6 || Wire.available() < 6) {
    return false;
  }

  int32_t rawPres = (static_cast<int32_t>(Wire.read()) << 12) |
                    (static_cast<int32_t>(Wire.read()) << 4) |
                    (Wire.read() >> 4);

  int32_t rawTemp = (static_cast<int32_t>(Wire.read()) << 12) |
                    (static_cast<int32_t>(Wire.read()) << 4) |
                    (Wire.read() >> 4);

  int32_t var1 = ((((rawTemp >> 3) - (static_cast<int32_t>(dig_T1) << 1))) *
                  static_cast<int32_t>(dig_T2)) >> 11;

  int32_t var2 = (((((rawTemp >> 4) - static_cast<int32_t>(dig_T1)) *
                    ((rawTemp >> 4) - static_cast<int32_t>(dig_T1))) >> 12) *
                  static_cast<int32_t>(dig_T3)) >> 14;

  int32_t tFine = var1 + var2;

  datos.temperatura = ((tFine * 5 + 128) >> 8) / 100.0f;

  int64_t pVar1 = static_cast<int64_t>(tFine) - 128000;
  int64_t pVar2 = pVar1 * pVar1 * static_cast<int64_t>(dig_P6);
  pVar2 += (pVar1 * static_cast<int64_t>(dig_P5)) << 17;
  pVar2 += static_cast<int64_t>(dig_P4) << 35;

  pVar1 = ((pVar1 * pVar1 * static_cast<int64_t>(dig_P3)) >> 8) +
          ((pVar1 * static_cast<int64_t>(dig_P2)) << 12);

  pVar1 = (((static_cast<int64_t>(1) << 47) + pVar1) *
           static_cast<int64_t>(dig_P1)) >> 33;

  if (pVar1 == 0) {
    return false;
  }

  int64_t presionRaw = 1048576 - rawPres;
  presionRaw = (((presionRaw << 31) - pVar2) * 3125) / pVar1;

  pVar1 = (static_cast<int64_t>(dig_P9) *
           (presionRaw >> 13) *
           (presionRaw >> 13)) >> 25;

  pVar2 = (static_cast<int64_t>(dig_P8) * presionRaw) >> 19;

  presionRaw = ((presionRaw + pVar1 + pVar2) >> 8) +
               (static_cast<int64_t>(dig_P7) << 4);

  datos.presion = static_cast<float>(presionRaw) / 25600.0f;

  if (datos.presion <= 0.0f) {
    return false;
  }

  // Cambia 1013.25 por la presión local al nivel del mar si buscas altitud real.
  datos.altitud = 44330.0f *
                  (1.0f - powf(datos.presion / 1013.25f, 0.1903f));

  return true;
}