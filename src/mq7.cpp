#include <Arduino.h>
#include "mq7.h"

namespace {
  constexpr uint8_t MQ7_PIN = 4;

  constexpr float RL_KOHM = 10.0f;
  constexpr float RO_CALIBRADA_KOHM = 105.32f;

  constexpr float VOLTAJE_ADC = 3.3f;
  constexpr int RESOLUCION_ADC = 4095;

  constexpr unsigned long TIEMPO_CALENTAMIENTO_MS = 60000;

  float roKohm = RO_CALIBRADA_KOHM;
  float ultimoRatio = -1.0f;

  bool iniciado = false;
  unsigned long inicioCalentamiento = 0;
}

bool mq7_begin() {
  pinMode(MQ7_PIN, INPUT);

#if defined(ARDUINO_ARCH_ESP32)
  analogReadResolution(12);
  analogSetPinAttenuation(MQ7_PIN, ADC_11db);
#endif

  inicioCalentamiento = millis();
  iniciado = true;

  Serial.println("MQ7 iniciado. Calentando 60 segundos...");
  return true;
}

bool mq7_estaListo() {
  return iniciado &&
         (millis() - inicioCalentamiento >= TIEMPO_CALENTAMIENTO_MS);
}

unsigned long mq7_tiempoCalentamientoRestanteMs() {
  if (!iniciado || mq7_estaListo()) {
    return 0;
  }

  unsigned long transcurrido = millis() - inicioCalentamiento;
  return TIEMPO_CALENTAMIENTO_MS - transcurrido;
}

bool mq7_leerVoltaje(float &voltaje) {
  long suma = 0;
  constexpr int MUESTRAS = 30;

  for (int i = 0; i < MUESTRAS; i++) {
    suma += analogRead(MQ7_PIN);
  }

  float promedio = static_cast<float>(suma) / MUESTRAS;
  voltaje = promedio * (VOLTAJE_ADC / RESOLUCION_ADC);

  return voltaje >= 0.0f && voltaje <= VOLTAJE_ADC;
}

bool mq7_leerRs(float &resistencia) {
  float voltaje;

  if (!mq7_leerVoltaje(voltaje) || voltaje <= 0.01f) {
    return false;
  }

  resistencia = RL_KOHM * (VOLTAJE_ADC - voltaje) / voltaje;
  return resistencia > 0.0f;
}

bool mq7_calibrar(int muestras) {
  if (!mq7_estaListo() || muestras <= 0) {
    return false;
  }

  Serial.println("Calibrando MQ7 en aire limpio...");

  float suma = 0.0f;
  int lecturasValidas = 0;

  for (int i = 0; i < muestras; i++) {
    float rs;

    if (mq7_leerRs(rs)) {
      suma += rs;
      lecturasValidas++;
    }

    delay(100);
  }

  if (lecturasValidas == 0) {
    Serial.println("No se pudo calibrar el MQ7.");
    return false;
  }

  roKohm = suma / lecturasValidas;

  Serial.print("Ro calibrado: ");
  Serial.print(roKohm, 3);
  Serial.println(" kOhm");

  return true;
}

bool mq7_ratio(float &ratio) {
  if (!mq7_estaListo()) {
    ultimoRatio = -1.0f;
    return false;
  }

  float rs;

  if (!mq7_leerRs(rs) || roKohm <= 0.0f) {
    ultimoRatio = -1.0f;
    return false;
  }

  ratio = rs / roKohm;
  ultimoRatio = ratio;

  return true;
}

const char* mq7_nivel() {
  if (!mq7_estaListo() || ultimoRatio < 0.0f) {
    return "ERROR";
  }

  if (ultimoRatio > 0.7f) {
    return "SEGURO";
  }

  if (ultimoRatio > 0.4f) {
    return "PRECAUCION";
  }

  return "PELIGRO";
}