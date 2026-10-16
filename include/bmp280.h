// bmp280.h
#pragma once

struct DatosBMP{
  /* data */
  float temperatura;
  float presion;
  float altitud;
};

bool bmp280_begin();
bool bmp280_leer(DatosBMP &datos);