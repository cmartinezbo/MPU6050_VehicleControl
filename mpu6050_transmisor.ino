#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <WiFi.h>
#include <esp_now.h>

// Selecciona la salida que deseas ver en el Serial Monitor
#define OUTPUT_READABLE_YAWPITCHROLL

// Instancia del MPU6050
MPU6050 mpu;

// Variables de control y estado del DMP
bool dmpReady = false;     // Se pone en true cuando el DMP se inicializa correctamente
uint8_t mpuIntStatus;      // Estado de la interrupción (no se usa en este ejemplo)
uint8_t devStatus;         // Estado de retorno de las operaciones (0 = éxito)
uint16_t packetSize;       // Tamaño esperado del paquete DMP (usualmente 42 bytes)
uint16_t fifoCount;        // Número de bytes actualmente en el FIFO
uint8_t fifoBuffer[64];    // Buffer para almacenar datos del FIFO

// Variables para la orientación
Quaternion q;            // Contenedor para el cuaternión [w, x, y, z]
VectorInt16 aa;          // (no usado en este ejemplo)
VectorInt16 aaReal;      // (no usado en este ejemplo)
VectorInt16 aaWorld;     // (no usado en este ejemplo)
VectorFloat gravity;     // Vector de gravedad
float euler[3];          // (no usado en este ejemplo)
float ypr[3];            // Contenedor para yaw, pitch y roll (en radianes)

// Estructura para transmitir los datos de orientación vía ESPNOW
typedef struct {
  float yaw;
  float pitch;
  float roll;
} OrientationData;
OrientationData orientationData;

// Dirección MAC del ESP32 RECEPTOR: A0:B7:65:2B:F6:98
uint8_t peerAddress[] = {0xA0, 0xB7, 0x65, 0x2B, 0xF6, 0x98};

// Callback opcional para confirmar el envío de datos
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Estado de envío: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Éxito" : "Fallo");
}

void setup() {
    // Inicializar el bus I2C
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400 kHz
    #endif

    Serial.begin(115200);
    while (!Serial); // Esperar a la conexión serial

    // Inicializar el MPU6050
    Serial.println("Inicializando MPU6050...");
    mpu.initialize();

    // Inicializar el DMP
    devStatus = mpu.dmpInitialize();

    // Configurar offsets (ajusta estos valores según tu hardware)
    mpu.setXGyroOffset(-150);
    mpu.setYGyroOffset(-30);
    mpu.setZGyroOffset(4);
    mpu.setZAccelOffset(1788); // Valor de fábrica para tu sensor

    if (devStatus == 0) {
        // Calibrar acelerómetro y giroscopio (opcional)
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();

        // Habilitar el DMP
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;

        // Obtener el tamaño del paquete esperado
        packetSize = mpu.dmpGetFIFOPacketSize();
        Serial.println("DMP listo!");
    } else {
        Serial.print("Fallo la inicialización del DMP (código ");
        Serial.print(devStatus);
        Serial.println(")");
    }

    // ------------------- Configuración de ESPNOW -------------------
    // Configurar el ESP32 en modo estación
    WiFi.mode(WIFI_STA);
    // Inicializar ESPNOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error al inicializar ESP-NOW");
        return;
    }
    // Registrar callback para envío (opcional)
    esp_now_register_send_cb(OnDataSent);

    // Agregar el peer con la dirección MAC del ESP32 receptor
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, peerAddress, 6);
    peerInfo.channel = 0;   // Canal por defecto
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Error al agregar el peer");
        return;
    }
    // --------------------------------------------------------------
}

void loop() {
    if (!dmpReady) return;
    
    // Leer un paquete del FIFO (si está disponible)
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // Obtener los datos del DMP
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            // Convertir de radianes a grados
            float yaw   = ypr[0] * 180.0 / M_PI;
            float pitch = ypr[1] * 180.0 / M_PI;
            float roll  = ypr[2] * 180.0 / M_PI;

            
            // Mostrar los valores por el monitor serial
            Serial.print("ypr\t");
            Serial.print(yaw);
            Serial.print("\t");
            Serial.print(pitch);
            Serial.print("\t");
            Serial.println(roll);
            
            
            // Preparar la estructura para transmitir vía ESPNOW
            orientationData.yaw = yaw;
            orientationData.pitch = pitch;
            orientationData.roll = roll;

            // Enviar los datos al ESP32 receptor
            esp_now_send(peerAddress, (uint8_t *) &orientationData, sizeof(orientationData));
        #endif
    }
}
