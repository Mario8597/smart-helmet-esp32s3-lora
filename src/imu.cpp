#include <Arduino.h>
#include <Wire.h>
#include "imu.h"

namespace {
  constexpr uint8_t ICM_ADDR = 0x68;

  constexpr float ACCEL_ESCALA = 16384.0f; // ±2 g
  constexpr float GYRO_ESCALA  = 131.0f;   // ±250 °/s

  float off_ax = 0.0f;
  float off_ay = 0.0f;
  float off_gx = 0.0f;
  float off_gy = 0.0f;
  float off_gz = 0.0f;

  bool leerRaw(int16_t &ax, int16_t &ay, int16_t &az,
               int16_t &gx, int16_t &gy, int16_t &gz,
               int16_t &temp) {
    Wire.beginTransmission(ICM_ADDR);
    Wire.write(0x3B);

    if (Wire.endTransmission(false) != 0) {
      return false;
    }

    if (Wire.requestFrom(ICM_ADDR, 14, true) != 14 || Wire.available() < 14) {
      return false;
    }

    ax   = (Wire.read() << 8) | Wire.read();
    ay   = (Wire.read() << 8) | Wire.read();
    az   = (Wire.read() << 8) | Wire.read();
    temp = (Wire.read() << 8) | Wire.read();
    gx   = (Wire.read() << 8) | Wire.read();
    gy   = (Wire.read() << 8) | Wire.read();
    gz   = (Wire.read() << 8) | Wire.read();

    return true;
  }

  bool escribirRegistro(uint8_t registro, uint8_t valor) {
    Wire.beginTransmission(ICM_ADDR);
    Wire.write(registro);
    Wire.write(valor);
    return Wire.endTransmission() == 0;
  }
}

bool imu_begin() {
  // Despertar la IMU.
  if (!escribirRegistro(0x6B, 0x00)) {
    Serial.println("IMU no encontrada.");
    return false;
  }

  // Acelerómetro: ±2 g.
  if (!escribirRegistro(0x1C, 0x00)) {
    return false;
  }

  // Giroscopio: ±250 °/s.
  if (!escribirRegistro(0x1B, 0x00)) {
    return false;
  }

  delay(200);
  Serial.println("IMU lista.");
  return true;
}

bool imu_calibrar(int muestras) {
  if (muestras <= 0) {
    return false;
  }

  Serial.println("Calibrando IMU... No muevas el sensor.");

  int64_t sumaAx = 0;
  int64_t sumaAy = 0;
  int64_t sumaGx = 0;
  int64_t sumaGy = 0;
  int64_t sumaGz = 0;

  int16_t ax, ay, az, gx, gy, gz, temp;

  for (int i = 0; i < muestras; i++) {
    if (!leerRaw(ax, ay, az, gx, gy, gz, temp)) {
      Serial.println("Error al leer IMU durante calibracion.");
      return false;
    }

    sumaAx += ax;
    sumaAy += ay;
    sumaGx += gx;
    sumaGy += gy;
    sumaGz += gz;

    delay(2);
  }

  // No se corrige AZ: en reposo debe conservar aproximadamente ±1 g.
  // No compensar acelerómetro aquí.
  // La gravedad puede estar en X, Y o Z según la orientación física del MPU6050.
  off_ax = 0.0f;
  off_ay = 0.0f;
  off_gx = (static_cast<float>(sumaGx) / muestras) / GYRO_ESCALA;
  off_gy = (static_cast<float>(sumaGy) / muestras) / GYRO_ESCALA;
  off_gz = (static_cast<float>(sumaGz) / muestras) / GYRO_ESCALA;

  Serial.println("Calibracion IMU lista.");
  return true;
}

bool imu_leer(DatosIMU &datos) {
  int16_t ax, ay, az, gx, gy, gz, temp;

  if (!leerRaw(ax, ay, az, gx, gy, gz, temp)) {
    return false;
  }

  datos.ax = ax / ACCEL_ESCALA;
  datos.ay = ay / ACCEL_ESCALA;
  datos.az = az / ACCEL_ESCALA;

  datos.gx = gx / GYRO_ESCALA - off_gx;
  datos.gy = gy / GYRO_ESCALA - off_gy;
  datos.gz = gz / GYRO_ESCALA - off_gz;

  datos.temp = temp / 340.0f + 36.53f;

  return true;
}