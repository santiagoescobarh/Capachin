#include <Wire.h>
#include <MPU6050.h>

class MPU6050Handler {
public:
  // Inicializa el sensor MPU6050
  bool begin() {
    Wire.begin();
    mpu.initialize();
    if (!mpu.testConnection()) {
      Serial.println("Error: No se pudo conectar con el MPU6050.");
      return false;
    }
    Serial.println("MPU6050 conectado correctamente.");
    return true;
  }
  
  // Procesa un comando recibido (por ejemplo, "girar")
  void processCommand(String command) {
    if (command == "girar") {
      executeTurn();
    } else {
      Serial.println("Comando no reconocido.");
    }
  }
  
  // Método público para obtener los datos del sensor
  void getSensorData(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
    readSensorData(ax, ay, az, gx, gy, gz);
  }
  
private:
  MPU6050 mpu;  // Objeto MPU6050, encapsulado en la sección privada

  // Función privada que lee los datos de aceleración y giroscopio
  void readSensorData(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  }
  
  // Función privada que simula la acción de giro
  void executeTurn() {
    Serial.println("Ejecutando comando de giro...");
    int16_t ax, ay, az, gx, gy, gz;
    readSensorData(ax, ay, az, gx, gy, gz);
    Serial.println("Datos del sensor durante el giro:");
    Serial.print("Aceleración -> X: "); Serial.print(ax);
    Serial.print(" | Y: "); Serial.print(ay);
    Serial.print(" | Z: "); Serial.println(az);
    Serial.print("Giroscopio -> X: "); Serial.print(gx);
    Serial.print(" | Y: "); Serial.print(gy);
    Serial.print(" | Z: "); Serial.println(gz);
    // Aquí podrías integrar el control de un servo o motor para realizar el giro físico.
  }
};

MPU6050Handler mpuHandler;

void setup() {
  Serial.begin(115200);
  
  // Inicializamos el MPU6050
  if (!mpuHandler.begin()) {
    Serial.println("Fallo al inicializar el MPU6050.");
  }
}

void loop() {
  // Verificamos si se ha recibido algún comando vía Serial
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Eliminamos espacios y saltos de línea
    mpuHandler.processCommand(command);
  }
  
  // Puedes agregar aquí lógica adicional, como lecturas periódicas del sensor
  delay(100);
}
