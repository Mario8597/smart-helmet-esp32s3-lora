#pragma once

struct DatosIMU {
    /* data */
    float ax, ay, az;
    float gx, gy, gz;
    float temp;
};

bool imu_begin();
bool imu_calibrar(int muestras = 500);
bool imu_leer(DatosIMU &datos);