#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <string.h>

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#if defined(ARDUINO_ARCH_ESP32)
  #include <esp_system.h>
#endif

#include "imu.h"
#include "aht20.h"
#include "bmp280.h"
#include "vl53.h"
#include "mq7.h"
#include "lora_emisor.h"

// ---- Pines I2C ----
constexpr uint8_t SDA_PIN = 8;
constexpr uint8_t SCL_PIN = 9;

// ---- Buzzer activo ----
constexpr uint8_t BUZZER_PIN = 5;
// Si el transistor activa el buzzer cuando GPIO5 está en HIGH:
constexpr bool BUZZER_ACTIVO_ALTO = true;
// Déjalo en false hasta corregir y calibrar el MQ-7.
constexpr bool ALERTA_MQ7_HABILITADA = false;
// El ratio menor a 0.4 corresponde a PELIGRO en mq7_nivel().
constexpr float UMBRAL_MQ7_PELIGRO = 0.40f;

VL53L0X vl53;

DatosIMU datosImu{};
DatosAHT datosAht{};
DatosBMP datosBmp{};

float distanciaMm = -1.0f;
float ratioCO = -1.0f;

bool imuDisponible = false;
bool ahtDisponible = false;
bool bmpDisponible = false;
bool vl53Disponible = false;
bool mq7Disponible = false;

bool imuLecturaValida = false;
bool ahtLecturaValida = false;
bool bmpLecturaValida = false;
bool vl53LecturaValida = false;
bool mq7LecturaValida = false;

bool mq7ListoInformado = false;

// ---------- Detector de caída ----------

enum class EstadoCaida {
  NORMAL,
  POSIBLE_CAIDA_LIBRE,
  POST_IMPACTO
};

EstadoCaida estadoCaida = EstadoCaida::NORMAL;

enum class ModoSistema {
  PRUEBAS,
  RUN
};

ModoSistema modoActual = ModoSistema::PRUEBAS;

unsigned long inicioPosibleCaida = 0;
unsigned long inicioPostImpacto  = 0;
unsigned long inicioInmovilidad  = 0;
unsigned long tiempoConfirmacion = 0;  // Momento en que se confirmó la caída

bool caidaConfirmada = false;

// Umbral de caída libre: aceleración total < 0.70g (más fácil de alcanzar en caídas simuladas)
constexpr float UMBRAL_CAIDA_LIBRE_G = 0.70f;
constexpr float UMBRAL_IMPACTO_G     = 2.50f;
// Tolerancia de giro para considerar al operario inmóvil post-impacto
constexpr float UMBRAL_GIRO_INMOVIL  = 30.0f;

constexpr unsigned long VENTANA_IMPACTO_MS          = 2000;   // Tiempo máximo entre caída libre e impacto
constexpr unsigned long INMOVILIDAD_CONFIRMACION_MS = 2000;   // Inmovilidad continua para confirmar caída
constexpr unsigned long LIMITE_POST_IMPACTO_MS      = 12000;  // Timeout del estado POST_IMPACTO
constexpr unsigned long AUTO_RESET_CAIDA_MS         = 30000;  // Reset automático de alerta tras 30 s

float magnitudAceleracion(const DatosIMU &imu) {
  return sqrtf(
    imu.ax * imu.ax +
    imu.ay * imu.ay +
    imu.az * imu.az
  );
}

float magnitudGiro(const DatosIMU &imu) {
  return sqrtf(
    imu.gx * imu.gx +
    imu.gy * imu.gy +
    imu.gz * imu.gz
  );
}

void actualizarDetectorCaida(const DatosIMU &imu, unsigned long ahora) {
  float aceleracion = magnitudAceleracion(imu);
  float giro = magnitudGiro(imu);

  // Auto-reset: si la caída ya fue confirmada y pasaron 30 s, limpiar la alerta
  if (caidaConfirmada && (ahora - tiempoConfirmacion >= AUTO_RESET_CAIDA_MS)) {
    caidaConfirmada    = false;
    tiempoConfirmacion = 0;
    estadoCaida        = EstadoCaida::NORMAL;
    inicioInmovilidad  = 0;
    return;  // Salir para no procesar más estados este ciclo
  }

  // Si la caída ya está confirmada, no buscar nuevas caídas hasta que el auto-reset limpie el flag
  if (caidaConfirmada) {
    return;
  }

  // Inmóvil: aceleración en reposo (0.60g a 1.40g) y rotación baja (< 30°/s)
  bool estaInmovil =
    aceleracion > 0.60f &&
    aceleracion < 1.40f &&
    giro < UMBRAL_GIRO_INMOVIL;

  switch (estadoCaida) {
    case EstadoCaida::NORMAL:
      // NO limpiar caidaConfirmada aquí: solo el auto-reset (30 s) o PRUEBAS la limpian
      if (aceleracion < UMBRAL_CAIDA_LIBRE_G) {
        estadoCaida        = EstadoCaida::POSIBLE_CAIDA_LIBRE;
        inicioPosibleCaida = ahora;
      }
      break;

    case EstadoCaida::POSIBLE_CAIDA_LIBRE:
      if (aceleracion > UMBRAL_IMPACTO_G) {
        estadoCaida       = EstadoCaida::POST_IMPACTO;
        inicioPostImpacto = ahora;
        inicioInmovilidad = 0;
      } else if (ahora - inicioPosibleCaida > VENTANA_IMPACTO_MS) {
        // No hubo impacto en el tiempo esperado → falsa alarma
        estadoCaida = EstadoCaida::NORMAL;
      }
      break;

    case EstadoCaida::POST_IMPACTO:
      if (estaInmovil) {
        if (inicioInmovilidad == 0) {
          inicioInmovilidad = ahora;
        }

        if (ahora - inicioInmovilidad >= INMOVILIDAD_CONFIRMACION_MS) {
          caidaConfirmada    = true;
          tiempoConfirmacion = ahora;
          // El auto-reset se encargará de volver a NORMAL tras 30 s
        }
      } else {
        inicioInmovilidad = 0;
      }

      // Timeout del estado POST_IMPACTO (no hubo inmovilidad suficiente → falsa alarma)
      if (ahora - inicioPostImpacto > LIMITE_POST_IMPACTO_MS) {
        estadoCaida       = EstadoCaida::NORMAL;
        inicioInmovilidad = 0;
      }
      break;
  }
}

void buzzerEscribir(bool encendido) {
  if (BUZZER_ACTIVO_ALTO) {
    digitalWrite(BUZZER_PIN, encendido ? HIGH : LOW);
  } else {
    digitalWrite(BUZZER_PIN, encendido ? LOW : HIGH);
  }
}

void actualizarBuzzer(unsigned long ahora) {
  bool alertaMq7 =
    ALERTA_MQ7_HABILITADA &&
    mq7LecturaValida &&
    ratioCO >= 0.0f &&
    ratioCO <= UMBRAL_MQ7_PELIGRO;

  if (caidaConfirmada) {
    // Dos pitidos repetidos cada segundo.
    unsigned long fase = ahora % 1000;

    bool encender =
      (fase < 180) ||
      (fase >= 320 && fase < 650);

    buzzerEscribir(encender);
    return;
  }

  if (alertaMq7) {
    // Un pitido corto cada dos segundos.
    unsigned long fase = ahora % 2000;

    buzzerEscribir(fase < 250);
    return;
  }

  buzzerEscribir(false);
}

void enviarRespuesta(const char *contenido) {
  char paquete[220];

  snprintf(
    paquete,
    sizeof(paquete),
    "TIPO:RSP,%s",
    contenido
  );

  lora_enviarPaquete(paquete);
}

void procesarComandoLoRa(const char *comando) {
  if (strstr(comando, "TIPO:CMD") == nullptr) {
    return;
  }

  if (strstr(comando, "ACCION:PING") != nullptr) {
    enviarRespuesta("ACCION:PONG,OK:1");
    return;
  }

  if (strstr(comando, "ACCION:STATUS") != nullptr) {
    char respuesta[180];

    snprintf(
      respuesta,
      sizeof(respuesta),
      "ACCION:STATUS,MODO:%s,IMU:%d,AHT:%d,BMP:%d,VL53:%d,MQ7:%d,MQREADY:%d",
      modoActual == ModoSistema::PRUEBAS ? "PRUEBAS" : "RUN",
      imuDisponible ? 1 : 0,
      ahtDisponible ? 1 : 0,
      bmpDisponible ? 1 : 0,
      vl53Disponible ? 1 : 0,
      mq7Disponible ? 1 : 0,
      mq7_estaListo() ? 1 : 0
    );

    enviarRespuesta(respuesta);
    return;
  }

  if (strstr(comando, "ACCION:TEST_IMU") != nullptr) {
    DatosIMU imu;

    if (!imuDisponible || !imu_leer(imu)) {
      enviarRespuesta("ACCION:TEST_IMU,OK:0");
      return;
    }

    float magnitud = magnitudAceleracion(imu);

    char respuesta[180];
    snprintf(
      respuesta,
      sizeof(respuesta),
      "ACCION:TEST_IMU,OK:1,AX:%.2f,AY:%.2f,AZ:%.2f,GX:%.1f,GY:%.1f,GZ:%.1f,AM:%.2f",
      imu.ax, imu.ay, imu.az,
      imu.gx, imu.gy, imu.gz,
      magnitud
    );

    enviarRespuesta(respuesta);
    return;
  }

  if (strstr(comando, "ACCION:TEST_AHT") != nullptr) {
    DatosAHT aht;

    if (!ahtDisponible || !aht20_leer(aht)) {
      enviarRespuesta("ACCION:TEST_AHT,OK:0");
      return;
    }

    char respuesta[100];
    snprintf(
      respuesta,
      sizeof(respuesta),
      "ACCION:TEST_AHT,OK:1,T:%.1f,H:%.1f",
      aht.temperatura,
      aht.humedad
    );

    enviarRespuesta(respuesta);
    return;
  }

  if (strstr(comando, "ACCION:TEST_BMP") != nullptr) {
    DatosBMP bmp;

    if (!bmpDisponible || !bmp280_leer(bmp)) {
      enviarRespuesta("ACCION:TEST_BMP,OK:0");
      return;
    }

    char respuesta[120];
    snprintf(
      respuesta,
      sizeof(respuesta),
      "ACCION:TEST_BMP,OK:1,T:%.1f,P:%.1f,ALT:%.1f",
      bmp.temperatura,
      bmp.presion,
      bmp.altitud
    );

    enviarRespuesta(respuesta);
    return;
  }

  if (strstr(comando, "ACCION:TEST_VL53") != nullptr) {
    float distancia;

    if (!vl53Disponible || !vl53_leer(vl53, distancia)) {
      enviarRespuesta("ACCION:TEST_VL53,OK:0,D:-1");
      return;
    }

    char respuesta[100];
    snprintf(
      respuesta,
      sizeof(respuesta),
      "ACCION:TEST_VL53,OK:1,D:%.0f,CASCO:%d",
      distancia,
      (distancia > 0.0f && distancia < 150.0f) ? 1 : 0
    );

    enviarRespuesta(respuesta);
    return;
  }

  if (strstr(comando, "ACCION:TEST_MQ7") != nullptr) {
    if (!mq7Disponible) {
      enviarRespuesta("ACCION:TEST_MQ7,OK:0,ERROR:NO_DISPONIBLE");
      return;
    }

    if (!mq7_estaListo()) {
      char respuesta[100];

      snprintf(
        respuesta,
        sizeof(respuesta),
        "ACCION:TEST_MQ7,OK:0,ESTADO:CALENTANDO,REST:%lu",
        mq7_tiempoCalentamientoRestanteMs() / 1000UL
      );

      enviarRespuesta(respuesta);
      return;
    }

    float ratio;

    if (!mq7_ratio(ratio)) {
      enviarRespuesta("ACCION:TEST_MQ7,OK:0,ERROR:LECTURA");
      return;
    }

    char respuesta[100];
    snprintf(
      respuesta,
      sizeof(respuesta),
      "ACCION:TEST_MQ7,OK:1,CO:%.2f,NIVEL:%s",
      ratio,
      mq7_nivel()
    );

    enviarRespuesta(respuesta);
    return;
  }

  if (strstr(comando, "ACCION:RUN") != nullptr) {
  // No permitir operación normal sin IMU:
  // la detección de caídas no funcionaría.
  if (!imuDisponible) {
    enviarRespuesta("ACCION:RUN,OK:0,ERROR:IMU_NO_DISPONIBLE");
    return;
  }

  modoActual = ModoSistema::RUN;
  enviarRespuesta("ACCION:RUN,OK:1,MODO:RUN");
  return;
}

  if (strstr(comando, "ACCION:PRUEBAS") != nullptr) {
    modoActual = ModoSistema::PRUEBAS;
    // Limpiar estado del detector para que el próximo RUN empiece sin falsos positivos
    caidaConfirmada    = false;
    tiempoConfirmacion = 0;
    estadoCaida        = EstadoCaida::NORMAL;
    inicioInmovilidad  = 0;
    buzzerEscribir(false);
    enviarRespuesta("ACCION:PRUEBAS,OK:1,MODO:PRUEBAS");
    return;
  }

  enviarRespuesta("ACCION:ERROR,MSG:COMANDO_DESCONOCIDO");
}

void revisarComandosLoRa() {
  char comando[220];

  if (lora_leerPaquete(comando, sizeof(comando))) {
    Serial.print("Comando LoRa: ");
    Serial.println(comando);

    procesarComandoLoRa(comando);
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Desactivar detector de brownout
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=== Inicio del emisor ===");
  
  //=====================
  
  pinMode(BUZZER_PIN, OUTPUT);
  buzzerEscribir(false);

  /*#if defined(ARDUINO_ARCH_ESP32)
    Serial.print("Razon de reset: ");
    Serial.println(esp_reset_reason());
  #endif*/

  // ---- LoRa ----
  if (!lora_begin()){
    Serial.println("ERROR: LoRa no pudo iniciar.");
    buzzerEscribir(true);

    while (true) {
      delay(1000);
    }
  }

  if (!lora_enviarEstado("LoRa listo")) {
  Serial.println("ADVERTENCIA: no se pudo enviar el estado inicial LoRa.");
  }

  // ---- I2C ----
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  // ---- IMU ----
  imuDisponible = imu_begin();

  if (imuDisponible) {
    lora_enviarEstado("IMU lista");
    lora_enviarEstado("Calibrando IMU - no mover sensor");

    imuDisponible = imu_calibrar();

    if (imuDisponible) {
      lora_enviarEstado("Calibracion IMU lista");
    } else {
      lora_enviarEstado("ERROR calibracion IMU");
    }
  } else {
    lora_enviarEstado("ERROR IMU no encontrada");
  }

  // ---- AHT20 ----
  ahtDisponible = aht20_begin();

  if (ahtDisponible) {
    lora_enviarEstado("AHT20 listo");
  } else {
    lora_enviarEstado("ERROR AHT20 no encontrado");
  }

  // ---- BMP280 ----
  bmpDisponible = bmp280_begin();

  if (bmpDisponible) {
    lora_enviarEstado("BMP280 listo");
  } else {
    lora_enviarEstado("ERROR BMP280 no encontrado");
  }

  // ---- VL53L0X ----
  vl53Disponible = vl53_begin(vl53);

  if (vl53Disponible) {
    lora_enviarEstado("VL53L0X listo");
  } else {
    lora_enviarEstado("ERROR VL53L0X no encontrado");
  }

  // ---- MQ-7 ----
  // Este mensaje llegará antes de que mq7_begin() bloquee 60 segundos.
  mq7Disponible = mq7_begin();

  if (mq7Disponible) {
    lora_enviarEstado("MQ7 iniciado - calentando 60 segundos");
  } else {
    lora_enviarEstado("ERROR MQ7");
  }

  Serial.println("=== Sistemas inicializados ===");
  lora_enviarEstado("Sistemas inicializados - MODO PRUEBAS");
}

void loop() {
  static unsigned long ultimaLecturaImu = 0;
  static unsigned long ultimaLecturaAht = 0;
  static unsigned long ultimaLecturaBmp = 0;
  static unsigned long ultimaLecturaVl53 = 0;
  static unsigned long ultimaLecturaMq7 = 0;
  static unsigned long ultimoEnvioLoRa = 0;

  unsigned long ahora = millis();
  
  revisarComandosLoRa();

if (mq7Disponible && mq7_estaListo() && !mq7ListoInformado) {
  mq7ListoInformado = true;

  Serial.println("MQ7 listo.");
  lora_enviarEstado("MQ7 listo");
}

if (modoActual == ModoSistema::PRUEBAS) {
  buzzerEscribir(false);
  return;
}

  if (imuDisponible && ahora - ultimaLecturaImu >= 10) {
    ultimaLecturaImu = ahora;
    imuLecturaValida = imu_leer(datosImu);

    if (!imuLecturaValida) {
      Serial.println("Error de lectura IMU.");
    } else {
      actualizarDetectorCaida(datosImu, ahora);
    }
  }

  if (ahtDisponible && ahora - ultimaLecturaAht >= 2000) {
    ultimaLecturaAht = ahora;
    ahtLecturaValida = aht20_leer(datosAht);

    if (!ahtLecturaValida) {
      Serial.println("Error de lectura AHT20.");
    }
  }

  if (bmpDisponible && ahora - ultimaLecturaBmp >= 2000) {
    ultimaLecturaBmp = ahora;
    bmpLecturaValida = bmp280_leer(datosBmp);

    if (!bmpLecturaValida) {
      Serial.println("Error de lectura BMP280.");
    }
  }

  if (vl53Disponible && ahora - ultimaLecturaVl53 >= 100) {
    ultimaLecturaVl53 = ahora;
    vl53LecturaValida = vl53_leer(vl53, distanciaMm);

    if (!vl53LecturaValida) {
      distanciaMm = -1.0f;
      Serial.println("Error de lectura VL53L0X.");
    }
  }

    if (mq7Disponible && ahora - ultimaLecturaMq7 >= 2000) {
  ultimaLecturaMq7 = ahora;

  if (!mq7_estaListo()) {
    // No es un error: el sensor todavía está calentando.
    ratioCO = -1.0f;
    mq7LecturaValida = false;
  } else {
    mq7LecturaValida = mq7_ratio(ratioCO);

    if (!mq7LecturaValida) {
      ratioCO = -1.0f;
      Serial.println("Error de lectura MQ7.");
    }
  }
}

  actualizarBuzzer(ahora);

  if (ahora - ultimoEnvioLoRa >= 500) {
  ultimoEnvioLoRa = ahora;

  // -1 = lectura VL53 inválida.
  //  0 = casco no detectado.
  //  1 = casco detectado.
  int estadoCasco = -1;

  if (vl53LecturaValida) {
    estadoCasco = (distanciaMm > 0.0f && distanciaMm < 150.0f) ? 1 : 0;
  }

  char paquete[240];

  int longitud = snprintf(
    paquete,
    sizeof(paquete),
    "AX:%.2f,AY:%.2f,AZ:%.2f,T:%.1f,H:%.1f,P:%.1f,D:%.0f,CASCO:%d,CO:%.2f,MQREADY:%d,NIVEL:%s,FALL:%d,ERR:%d%d%d%d%d",
    datosImu.ax,
    datosImu.ay,
    datosImu.az,
    datosAht.temperatura,
    datosAht.humedad,
    datosBmp.presion,
    distanciaMm,
    estadoCasco,
    ratioCO,
    mq7_estaListo() ? 1 : 0,
    mq7_nivel(),
    caidaConfirmada ? 1 : 0,
    imuLecturaValida ? 0 : 1,
    ahtLecturaValida ? 0 : 1,
    bmpLecturaValida ? 0 : 1,
    vl53LecturaValida ? 0 : 1,
    mq7LecturaValida ? 0 : 1
  );

  if (longitud >= 0 && longitud < static_cast<int>(sizeof(paquete))) {
    lora_enviarPaquete(paquete);
  } else {
    Serial.println("ERROR: paquete LoRa demasiado largo.");
  }
}
}

