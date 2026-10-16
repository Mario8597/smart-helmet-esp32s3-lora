#pragma once

bool mq7_begin();

bool mq7_estaListo();

unsigned long mq7_tiempoCalentamientoRestanteMs();

bool mq7_calibrar(int muestras = 50);

bool mq7_leerVoltaje(float &voltaje);
bool mq7_leerRs(float &resistencia);

bool mq7_ratio(float &ratio);

// Devuelve: "ERROR", "SEGURO", "PRECAUCION" o "PELIGRO".
const char* mq7_nivel();

/*
#define RO_AIRE   93.1342f
#define MQ7_PIN   4
#define RL        10.0f
#define RO_AIRE   105.32f */