#include <Arduino.h>
#include <Wire.h>
#include "aht20.h"

namespace {
  constexpr uint8_t AHT20_ADDR = 0x38;

  bool midiendo = false;
  unsigned long tiempoInicio = 0;
  DatosAHT ultimosDatos = {0.0f, 0.0f};
  bool hayDatosValidos = false;

  bool escribirComando(uint8_t comando, uint8_t dato1, uint8_t dato2) {
    Wire.beginTransmission(AHT20_ADDR);
    Wire.write(comando);
    Wire.write(dato1);
    Wire.write(dato2);

    return Wire.endTransmission() == 0;
  }

  bool iniciarMedicion() {
    if (escribirComando(0xAC, 0x33, 0x00)) {
      midiendo = true;
      tiempoInicio = millis();
      return true;
    }
    return false;
  }
}

bool aht20_begin() {
  delay(100);

  if (!escribirComando(0xBE, 0x08, 0x00)) {
    Serial.println("AHT20 no encontrado.");
    return false;
  }

  delay(20);

  if (Wire.requestFrom(AHT20_ADDR, 1) != 1 || Wire.available() < 1) {
    Serial.println("AHT20 no responde.");
    return false;
  }

  uint8_t estado = Wire.read();

  // Bit 3: sensor calibrado/inicializado.
  if ((estado & 0x08) == 0) {
    Serial.println("AHT20 no se pudo inicializar.");
    return false;
  }

  // Iniciar la primera medición
  iniciarMedicion();

  Serial.println("AHT20 listo.");
  return true;
}

bool aht20_leer(DatosAHT &datos) {
  unsigned long ahora = millis();

  // Si no se había iniciado medición, iniciarla
  if (!midiendo) {
    iniciarMedicion();
    if (hayDatosValidos) {
      datos = ultimosDatos;
      return true;
    }
    return false;
  }

  // Si aún no transcurrieron los 80 ms necesarios de conversión
  if (ahora - tiempoInicio < 80) {
    if (hayDatosValidos) {
      datos = ultimosDatos;
      return true;
    }
    return false;
  }

  if (Wire.requestFrom(AHT20_ADDR, 6) != 6 || Wire.available() < 6) {
    midiendo = false;
    return hayDatosValidos;
  }

  uint8_t buffer[6];
  for (uint8_t i = 0; i < 6; i++) {
    buffer[i] = Wire.read();
  }

  // Bit 7 indica si el sensor sigue ocupado
  if (buffer[0] & 0x80) {
    return hayDatosValidos;
  }

  uint32_t rawHum = (static_cast<uint32_t>(buffer[1]) << 12) |
                    (static_cast<uint32_t>(buffer[2]) << 4) |
                    (buffer[3] >> 4);

  uint32_t rawTemp = (static_cast<uint32_t>(buffer[3] & 0x0F) << 16) |
                     (static_cast<uint32_t>(buffer[4]) << 8) |
                     buffer[5];

  ultimosDatos.humedad = (rawHum / 1048576.0f) * 100.0f;
  ultimosDatos.temperatura = (rawTemp / 1048576.0f) * 200.0f - 50.0f;
  hayDatosValidos = true;
  midiendo = false;

  datos = ultimosDatos;

  // Iniciar de una vez la siguiente medición para el próximo ciclo
  iniciarMedicion();

  return true;
}