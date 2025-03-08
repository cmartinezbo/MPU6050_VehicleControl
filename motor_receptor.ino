#include <WiFi.h>
#include <esp_now.h>
#include "Arduino.h"

// --------------------------------------------------------------------
// Configuración de control de motores (L298N)
// --------------------------------------------------------------------
// Motor A: MOTOR DERECHO
const int ENA = 13;  // Pin PWM Motor A
const int IN1 = 12;  // Dirección Motor A
const int IN2 = 14;

// Motor B: MOTOR IZQUIERDO
const int ENB = 25;  // Pin PWM Motor B
const int IN3 = 27;  // Dirección Motor B
const int IN4 = 26;

// Parámetros de PWM (nueva API LEDC)
const int pwmFreq = 2000;    // Frecuencia de PWM: 2 kHz
const int pwmResolution = 8; // Resolución: 8 bits (0-255)

// --------------------------------------------------------------------
// Configuración de recepción ESP-NOW
// --------------------------------------------------------------------
typedef struct {
  float yaw;
  float pitch;
  float roll;
} OrientationData;

OrientationData orientation = {0, 0, 0};

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  if (len == sizeof(OrientationData)) {
    OrientationData received;
    memcpy(&received, data, sizeof(received));
    orientation = received;
    
    Serial.print("Datos recibidos -> Yaw: ");
    Serial.print(received.yaw);
    Serial.print(" | Pitch: ");
    Serial.print(received.pitch);
    Serial.print(" | Roll: ");
    Serial.println(received.roll);
  } else {
    Serial.println("Tamaño de datos recibido inesperado");
  }
}

// --------------------------------------------------------------------
// Parámetros de control basados en orientación
// --------------------------------------------------------------------
const float pitchThreshold = 5.0;   // Umbral en grados para activar movimiento con pitch
const float yawThreshold   = 5.0;   // Umbral en grados para activar giro
const int maxMappedAngle   = 90;    // Ángulo máximo para mapear (en grados)
const float maxYawAngle    = 90.0;  // Ángulo máximo para mapear yaw (en grados)
const int minPWM           = 100;   // Valor mínimo de PWM para arrancar los motores

// --------------------------------------------------------------------
// Funciones auxiliares para el mapeo
// --------------------------------------------------------------------
// Mapea el pitch a velocidad PWM de minPWM a 255.
// Se mapea desde pitchThreshold hasta maxMappedAngle.
int mapAngleToSpeed(float angle) {
  int speed = map((int)abs(angle), (int)pitchThreshold, maxMappedAngle, minPWM, 255);
  if (speed > 255) speed = 255;
  return speed;
}

// Mapea el yaw a velocidad PWM de minPWM a 255.
// Se mapea desde yawThreshold hasta maxYawAngle.
int mapYawToSpeed(float yawVal) {
  float absYaw = abs(yawVal);
  if (absYaw < yawThreshold) return 0;
  if (absYaw > maxYawAngle) absYaw = maxYawAngle;
  int turnSpeed = map((int)absYaw, (int)yawThreshold, (int)maxYawAngle, minPWM, 255);
  if (turnSpeed > 255) turnSpeed = 255;
  return turnSpeed;
}

// --------------------------------------------------------------------
// Funciones de control de motores
// --------------------------------------------------------------------
// Movimiento hacia adelante: (pitch negativo => avance)
void setMotorsForward(int speedRight, int speedLeft) {
  // Avanzar:
  // Motor A (derecho): IN1 HIGH, IN2 LOW.
  // Motor B (izquierdo): IN3 HIGH, IN4 LOW.
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(ENA, speedRight);
  ledcWrite(ENB, speedLeft);
}

// Movimiento hacia atrás: (pitch positivo => retroceso)
void setMotorsBackward(int speedRight, int speedLeft) {
  // Retroceder:
  // Motor A (derecho): IN1 LOW, IN2 HIGH.
  // Motor B (izquierdo): IN3 LOW, IN4 HIGH.
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(ENA, speedRight);
  ledcWrite(ENB, speedLeft);
}

// Frenado activo
void brakeMotors() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
}

// Giro en sitio (turn in place):
// Se ignoran los datos de pitch y se realiza un giro sobre el propio eje.
// Según lo requerido:
// - Yaw positivo: giro a la derecha (Motor A retrocede, Motor B avanza).
// - Yaw negativo: giro a la izquierda (Motor A avanza, Motor B retrocede).
void turnInPlace(float yawVal, int turnSpeed) {
  if (yawVal > 0) { // Giro a la derecha
    digitalWrite(IN1, HIGH);   // Motor A (derecho) retrocede
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);     // Motor B (izquierdo) avanza
    digitalWrite(IN4, LOW);
  } else if (yawVal < 0) { // Giro a la izquierda
    digitalWrite(IN1, LOW);    // Motor A (derecho) avanza
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);     // Motor B (izquierdo) retrocede
    digitalWrite(IN4, HIGH);
  }
  ledcWrite(ENA, turnSpeed);
  ledcWrite(ENB, turnSpeed);
}

// --------------------------------------------------------------------
// Setup
// --------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Receptor + Control de Motores - Iniciando...");

  // Configurar WiFi en modo STA
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error al inicializar ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("Esperando datos de orientación...");

  // Configurar pines de dirección de motores
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Configurar PWM usando la nueva API LEDC
  ledcAttach(ENA, pwmFreq, pwmResolution);
  ledcAttach(ENB, pwmFreq, pwmResolution);
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
}

// --------------------------------------------------------------------
// Loop
// --------------------------------------------------------------------
void loop() {
  // Obtener los valores actuales de orientación
  float pitch = orientation.pitch; // Pitch negativo: dedos hacia abajo => avance
  float yaw   = orientation.yaw;   // Yaw positivo: giro a la derecha; yaw negativo: giro a la izquierda

  // Si ni pitch ni yaw superan sus umbrales, frenar el carro
  if (abs(pitch) < pitchThreshold && abs(yaw) < yawThreshold) {
    brakeMotors();
  }
  // Si ambos están activos, se toma el ángulo de mayor magnitud
  else {
    if (abs(yaw) >= abs(pitch)) {
      int turnSpeed = mapYawToSpeed(yaw);
      turnInPlace(yaw, turnSpeed);
    } else {
      int baseSpeed = mapAngleToSpeed(pitch);
      // Según el signo del pitch: si negativo, avanzar; si positivo, retroceder.
      if (pitch < 0) {
        setMotorsForward(baseSpeed, baseSpeed);
      } else {
        setMotorsBackward(baseSpeed, baseSpeed);
      }
    }
  }
  
  delay(100);
}
