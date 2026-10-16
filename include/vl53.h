// vl53l0x.h
#pragma once

class VL53L0X;

bool vl53_begin(VL53L0X &sensor);
bool vl53_leer(VL53L0X &sensor, float &distanciaMm);