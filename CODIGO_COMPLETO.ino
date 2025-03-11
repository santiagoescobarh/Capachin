#include <Wire.h>
#include <VL53L0X.h>

// -------------------------------------------------
// Clase para manejar un sensor VL53L0X individual
// -------------------------------------------------
class VL53L0X_SensorHandler {
  public:
    // Constructor: recibe el pin de XSHUT y la nueva dirección I2C
    VL53L0X_SensorHandler(int shutdownPin, uint8_t newAddress)
      : shutdownPin(shutdownPin), newAddress(newAddress) {}

    // Inicializa el sensor: configura el pin, lo enciende, lo inicializa, cambia su dirección y arranca en modo continuo
    bool begin() {
      pinMode(shutdownPin, OUTPUT);
      digitalWrite(shutdownPin, LOW);  // Apaga el sensor para evitar conflictos en el bus I2C
      delay(10);
      digitalWrite(shutdownPin, HIGH); // Enciende el sensor
      delay(10);
      
      if (!sensor.init()) {
        Serial.print("Error al inicializar sensor en pin ");
        Serial.println(shutdownPin);
        return false;
      }
      sensor.setAddress(newAddress); // Cambia la dirección (por defecto es 0x29)
      sensor.startContinuous();        // Inicia la medición continua
      return true;
    }
    
    // Devuelve la distancia medida en milímetros
    uint16_t getDistance() {
      return sensor.readRangeContinuousMillimeters();
    }
    
  private:
    int shutdownPin;
    uint8_t newAddress;
    VL53L0X sensor;
};

// -------------------------------------------------
// Clase para manejar los tres sensores de distancia
// -------------------------------------------------
class DistanceSensors {
  public:
    // Constructor: asigna pines y direcciones para cada sensor  
    // Izquierdo: pin 15, dirección 0x30  
    // Central:   pin 2,  dirección 0x31  
    // Derecho:   pin 4,  dirección 0x32
    DistanceSensors() 
      : sensorLeft(15, 0x30), sensorCenter(2, 0x31), sensorRight(4, 0x32) {}

    // Inicializa los tres sensores
    bool begin() {
      pinMode(15, OUTPUT);
      pinMode(2, OUTPUT);
      pinMode(4, OUTPUT);
      digitalWrite(15, LOW);
      digitalWrite(2, LOW);
      digitalWrite(4, LOW);
      delay(10);
      
      bool ok = true;
      if (!sensorLeft.begin()) {
        Serial.println("Error al inicializar sensor izquierdo");
        ok = false;
      }
      if (!sensorCenter.begin()) {
        Serial.println("Error al inicializar sensor central");
        ok = false;
      }
      if (!sensorRight.begin()) {
        Serial.println("Error al inicializar sensor derecho");
        ok = false;
      }
      return ok;
    }
    
    // Lee las distancias de los tres sensores y las retorna por referencia
    void readSensors(uint16_t &left, uint16_t &center, uint16_t &right) {
      left = sensorLeft.getDistance();
      center = sensorCenter.getDistance();
      right = sensorRight.getDistance();
    }
    
  private:
    VL53L0X_SensorHandler sensorLeft;
    VL53L0X_SensorHandler sensorCenter;
    VL53L0X_SensorHandler sensorRight;
};

// -------------------------------------------------
// Clase para controlar los motores mediante un puente H
// -------------------------------------------------
class MotorDriver {
  public:
    // Inicializa los pines de los motores
    bool begin() {
      // Pines de ejemplo, ajusta según tu conexión:
      // Motor izquierdo: forward (16) y backward (17)
      // Motor derecho: forward (18) y backward (19)
      pinMode(leftMotorForwardPin, OUTPUT);
      pinMode(leftMotorBackwardPin, OUTPUT);
      pinMode(rightMotorForwardPin, OUTPUT);
      pinMode(rightMotorBackwardPin, OUTPUT);
      stop();
      return true;
    }
    
    // Avanza ambos motores
    void forward() {
      digitalWrite(leftMotorForwardPin, HIGH);
      digitalWrite(leftMotorBackwardPin, LOW);
      digitalWrite(rightMotorForwardPin, HIGH);
      digitalWrite(rightMotorBackwardPin, LOW);
    }
    
    // Retrocede ambos motores
    void backward() {
      digitalWrite(leftMotorForwardPin, LOW);
      digitalWrite(leftMotorBackwardPin, HIGH);
      digitalWrite(rightMotorForwardPin, LOW);
      digitalWrite(rightMotorBackwardPin, HIGH);
    }
    
    // Gira a la izquierda: motor izquierdo retrocede, motor derecho avanza
    void turnLeft() {
      digitalWrite(leftMotorForwardPin, LOW);
      digitalWrite(leftMotorBackwardPin, HIGH);
      digitalWrite(rightMotorForwardPin, HIGH);
      digitalWrite(rightMotorBackwardPin, LOW);
    }
    
    // Gira a la derecha: motor izquierdo avanza, motor derecho retrocede
    void turnRight() {
      digitalWrite(leftMotorForwardPin, HIGH);
      digitalWrite(leftMotorBackwardPin, LOW);
      digitalWrite(rightMotorForwardPin, LOW);
      digitalWrite(rightMotorBackwardPin, HIGH);
    }
    
    // Detiene ambos motores
    void stop() {
      digitalWrite(leftMotorForwardPin, LOW);
      digitalWrite(leftMotorBackwardPin, LOW);
      digitalWrite(rightMotorForwardPin, LOW);
      digitalWrite(rightMotorBackwardPin, LOW);
    }
    
  private:
    // Pines de control para los motores (ajusta según tu hardware)
    const int leftMotorForwardPin  = 16;
    const int leftMotorBackwardPin = 17;
    const int rightMotorForwardPin = 18;
    const int rightMotorBackwardPin = 19;
};

// -------------------------------------------------
// Clase Robot que integra sensores y motores
// -------------------------------------------------
class Robot {
  public:
    // Inicializa Serial, I2C, motores y sensores
    bool begin() {
      Serial.begin(115200);
      Wire.begin();
      bool motorOk = motorDriver.begin();
      bool sensorsOk = sensors.begin();
      return motorOk && sensorsOk;
    }
    
    // Función principal de actualización que se llama en cada ciclo (loop)
    void update() {
      // 1. Leer las distancias de los sensores
      uint16_t leftDist, centerDist, rightDist;
      sensors.readSensors(leftDist, centerDist, rightDist);
      
      Serial.print("Izq: ");
      Serial.print(leftDist);
      Serial.print(" mm | Ctr: ");
      Serial.print(centerDist);
      Serial.print(" mm | Der: ");
      Serial.print(rightDist);
      Serial.println(" mm");
      
      // 2. Si se detecta un obstáculo (distancia < 50 mm), se ejecuta la maniobra de evasión
      if (leftDist < obstacleThreshold || centerDist < obstacleThreshold || rightDist < obstacleThreshold) {
        avoidObstacle(leftDist, centerDist, rightDist);
      }
      // 3. Si no hay obstáculos, se procesa el comando recibido vía Serial (si lo hubiera)
      else {
        if (Serial.available() > 0) {
          String command = Serial.readStringUntil('\n');
          command.trim();
          processCommand(command);
        } else {
          // Si no se recibe comando, se puede definir una acción por defecto (por ejemplo, avanzar)
          motorDriver.forward();
        }
      }
      
      delay(100);
    }
    
  private:
    MotorDriver motorDriver;
    DistanceSensors sensors;
    const uint16_t obstacleThreshold = 50; // Umbral en milímetros
    
    // Procesa el comando recibido (por ejemplo, "adelante", "atras", "giro derecha", "giro izquierda", "stop")
    void processCommand(String command) {
      Serial.print("Comando recibido: ");
      Serial.println(command);
      if (command.equalsIgnoreCase("adelante")) {
        motorDriver.forward();
      }
      else if (command.equalsIgnoreCase("atras")) {
        motorDriver.backward();
      }
      else if (command.equalsIgnoreCase("giro derecha") || command.equalsIgnoreCase("derecha")) {
        motorDriver.turnRight();
      }
      else if (command.equalsIgnoreCase("giro izquierda") || command.equalsIgnoreCase("izquierda")) {
        motorDriver.turnLeft();
      }
      else if (command.equalsIgnoreCase("stop") || command.equalsIgnoreCase("parar")) {
        motorDriver.stop();
      }
      else {
        Serial.println("Comando no reconocido.");
        motorDriver.stop();
      }
    }
    
    // Ejecuta la maniobra de evasión según la lectura de los sensores
    void avoidObstacle(uint16_t leftDist, uint16_t centerDist, uint16_t rightDist) {
      Serial.println("Obstáculo detectado. Ejecutando maniobra de evasión.");
      // Si el sensor izquierdo detecta el obstáculo, se gira a la derecha
      if (leftDist < obstacleThreshold && leftDist <= centerDist && leftDist <= rightDist) {
        Serial.println("Evasión: Giro a la DERECHA.");
        motorDriver.turnRight();
      }
      // Si el sensor derecho detecta el obstáculo, se gira a la izquierda
      else if (rightDist < obstacleThreshold && rightDist <= centerDist && rightDist <= leftDist) {
        Serial.println("Evasión: Giro a la IZQUIERDA.");
        motorDriver.turnLeft();
      }
      // Si el sensor central detecta el obstáculo, se decide según los laterales
      else if (centerDist < obstacleThreshold) {
        if (leftDist > rightDist) {
          Serial.println("Evasión: Centro detectado, giro a la IZQUIERDA.");
          motorDriver.turnLeft();
        }
        else {
          Serial.println("Evasión: Centro detectado, giro a la DERECHA.");
          motorDriver.turnRight();
        }
      }
      delay(500);  // Tiempo para completar la maniobra
      motorDriver.stop();
    }
};

Robot robot;

void setup() {
  if (!robot.begin()) {
    Serial.println("Error al iniciar el robot.");
    while (1); // Detiene el programa si falla la inicialización
  }
  Serial.println("Robot iniciado correctamente.");
}

void loop() {
  robot.update();
}
