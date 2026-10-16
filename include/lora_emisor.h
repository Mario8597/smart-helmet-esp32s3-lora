#pragma once

#include <stddef.h>

bool lora_begin();
bool lora_enviarEstado(const char *mensaje);
bool lora_enviarPaquete(const char *paquete);
bool lora_leerPaquete(char *buffer, size_t tamanoBuffer);