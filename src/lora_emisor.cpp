#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

#include "lora_emisor.h"

namespace {
  constexpr uint8_t kLoraSck  = 12;
  constexpr uint8_t kLoraMiso = 13;
  constexpr uint8_t kLoraMosi = 11;
  constexpr uint8_t kLoraSs   = 10;
  constexpr uint8_t kLoraRst  = 14;
  constexpr uint8_t kLoraDio0 = 2;

  constexpr long kLoraFrecuencia = 433E6;

  bool inicializado = false;
}

bool lora_begin() {
  SPI.begin(kLoraSck, kLoraMiso, kLoraMosi, kLoraSs);
  LoRa.setPins(kLoraSs, kLoraRst, kLoraDio0);

  if (!LoRa.begin(kLoraFrecuencia)) {
    Serial.println("Error al iniciar LoRa.");
    return false;
  }

  LoRa.setTxPower(14);
  LoRa.setSpreadingFactor(7);        
  LoRa.setSignalBandwidth(125E3);    
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  LoRa.receive();

  inicializado = true;

  Serial.println("LoRa listo y escuchando.");
  return true;
}

bool lora_enviarPaquete(const char *paquete) {
  if (!inicializado || paquete == nullptr) {
    return false;
  }

  LoRa.idle();
  LoRa.beginPacket();
  LoRa.print(paquete);

  // Envío síncrono (~150 ms): garantiza que el paquete salió antes de volver a escuchar.
  // El modo asíncrono (endPacket(true)) causaba cambios espontáneos RUN→PRUEBAS
  // porque la interrupción DIO0 es compartida entre TX-done y RX-packet.
  int resultado = LoRa.endPacket();

  // Volver a modo escucha inmediatamente después de transmitir
  LoRa.receive();

  if (resultado != 1) {
    Serial.println("Error enviando paquete LoRa.");
    return false;
  }

  Serial.print("LoRa enviado: ");
  Serial.println(paquete);

  return true;
}

bool lora_enviarEstado(const char *mensaje) {
  if (mensaje == nullptr) {
    return false;
  }

  char paquete[140];

  int longitud = snprintf(
    paquete,
    sizeof(paquete),
    "TIPO:ESTADO,MSG:%s",
    mensaje
  );

  if (longitud < 0 || longitud >= static_cast<int>(sizeof(paquete))) {
    return false;
  }

  return lora_enviarPaquete(paquete);
}

bool lora_leerPaquete(char *buffer, size_t tamanoBuffer) {
  if (!inicializado || buffer == nullptr || tamanoBuffer < 2) {
    return false;
  }

  int tamanoPaquete = LoRa.parsePacket();

  if (tamanoPaquete == 0) {
    return false;
  }

  size_t posicion = 0;

  while (LoRa.available()) {
    char caracter = static_cast<char>(LoRa.read());

    if (posicion < tamanoBuffer - 1) {
      buffer[posicion++] = caracter;
    }
  }

  buffer[posicion] = '\0';
  return true;
}