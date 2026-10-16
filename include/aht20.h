// aht20.h
#pragma once

struct DatosAHT{
  float temperatura;
  float humedad;
};

bool aht20_begin();
bool aht20_leer(DatosAHT &datos);