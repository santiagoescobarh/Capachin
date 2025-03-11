#include <Wire.h>
#include <VL53L0X.h>

// Clase para manejar un sensor VL53L0X
class VL53L0X_SensorHandler {
public:
  VL53L0X sensor;
  int shutdownPin;
  uint8_t newAddress;
  
  // Constructor: se le indica el pin de apagado y la nueva dirección I2C
  VL53L0X_SensorHandler(int shutdownPin, uint8_t newAddress)
    : shutdownPin(shutdownPin), newAddress(newAddress) {}

  // Inicialización del sensor: enciende el sensor, lo inicializa y cambia su dirección
  bool begin() {
    // Configuramos el pin de apagado
    pinMode(shutdownPin, OUTPUT);
    // Apagamos el sensor para evitar conflictos en el bus I2C
    digitalWrite(shutdownPin, LOW);
    delay(10);
    // Encendemos el sensor
    digitalWrite(shutdownPin, HIGH);
    delay(10);
    
    // Inicializamos el sensor
    if (!sensor.init()) {
      Serial.print("Error al inicializar sensor en pin ");
      Serial.println(shutdownPin);
      return false;
    }
    // Cambiamos la dirección I2C (la dirección por defecto es 0x29)
    sensor.setAddress(newAddress);
    // Iniciamos la medición en modo continuo (opcional)
    sensor.startContinuous();
    
    return true;
  }

  // Función para obtener la distancia medida en milímetros
  uint16_t getDistance() {
    return sensor.readRangeContinuousMillimeters();
  }
};

// Instanciamos tres sensores: se asignan pines de XSHUT y nuevas direcciones I2C diferentes.
VL53L0X_SensorHandler sensor1(15, 0x30); // Sensor 1: pin 15, dirección 0x30
VL53L0X_SensorHandler sensor2(2, 0x31);  // Sensor 2: pin 2, dirección 0x31
VL53L0X_SensorHandler sensor3(4, 0x32);  // Sensor 3: pin 4, dirección 0x32

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Apagamos inicialmente todos los sensores para evitar que se enciendan al mismo tiempo
  pinMode(15, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  digitalWrite(15, LOW);
  digitalWrite(2, LOW);
  digitalWrite(4, LOW);
  delay(10);
  
  // Inicializamos cada sensor secuencialmente
  if (!sensor1.begin()) {
    Serial.println("Error en sensor1");
  }
  if (!sensor2.begin()) {
    Serial.println("Error en sensor2");
  }
  if (!sensor3.begin()) {
    Serial.println("Error en sensor3");
  }
}

void loop() {
  // Leemos las distancias de cada sensor
  uint16_t dist1 = sensor1.getDistance();
  uint16_t dist2 = sensor2.getDistance();
  uint16_t dist3 = sensor3.getDistance();
  
  // Imprimimos los valores en el monitor serial
  Serial.print("Sensor1: ");
  Serial.print(dist1);
  Serial.print(" mm, Sensor2: ");
  Serial.print(dist2);
  Serial.print(" mm, Sensor3: ");
  Serial.print(dist3);
  Serial.println(" mm");
  
  delay(100);
}
