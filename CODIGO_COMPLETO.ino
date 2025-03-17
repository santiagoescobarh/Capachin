#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// Instancias de los sensores
Adafruit_VL53L0X sensor_frontal = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor_lateral = Adafruit_VL53L0X();

// Pines del puente H L298N
#define ENA 5   // PWM Motor A
#define IN1 18   // Dirección Motor A
#define IN2 19   // Dirección Motor A
#define ENB 25  // PWM Motor B
#define IN3 4  // Dirección Motor B
#define IN4 23  // Dirección Motor B

// Pines de control de encendido de los sensores
#define XSHUT_FRONTAL 32
#define XSHUT_LATERAL 33

// Potencia del motor (0-255)
int potencia = 150; // Aumentado para notar más el giro

void setup() {
    Serial.begin(9600);
    Serial.println("Iniciando sensores VL53L0X...");

    // Configurar pines del motor
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    
    // Configurar pines XSHUT de los sensores
    pinMode(XSHUT_FRONTAL, OUTPUT);
    pinMode(XSHUT_LATERAL, OUTPUT);
    
    Wire.begin();
    
    // Apagar ambos sensores antes de inicializar
    digitalWrite(XSHUT_FRONTAL, LOW);
    digitalWrite(XSHUT_LATERAL, LOW);
    delay(10);
    
    // Encender e inicializar sensor frontal
    digitalWrite(XSHUT_FRONTAL, HIGH);
    delay(10);
    if (!sensor_frontal.begin(0x30)) {
        Serial.println("Error al iniciar el sensor frontal");
    }
    
    // Encender e inicializar sensor lateral con dirección diferente
    digitalWrite(XSHUT_LATERAL, HIGH);
    delay(10);
    if (!sensor_lateral.begin(0x31)) {
        Serial.println("Error al iniciar el sensor lateral");
    }
}

void loop() {
    int distancia_frontal = Medir_VL53L0X(sensor_frontal);
    int distancia_lateral = Medir_VL53L0X(sensor_lateral);
    
    Serial.print("Distancia Frontal: "); Serial.println(distancia_frontal);
    Serial.print("Distancia Lateral: "); Serial.println(distancia_lateral);
    
    if (distancia_frontal > 100) {
        adelante();
        Serial.println("Avanzando");
    } else {
        detener();
        delay(5);
        
        if (distancia_frontal <= 100 && distancia_lateral <= 100) {
            izquierda();
            Serial.println("Girando a la izquierda");
        } else if (distancia_lateral > 100) {
            derecha();
            Serial.println("Girando a la derecha");
        } else {
            izquierda();
            Serial.println("Girando a la izquierda");
        }
    }
    delay(5);
}

float Medir_VL53L0X(Adafruit_VL53L0X &sensor) {
    VL53L0X_RangingMeasurementData_t medida;
    sensor.rangingTest(&medida, false);
    return (medida.RangeStatus != 4) ? medida.RangeMilliMeter : 9999; // 9999 indica fuera de rango
}

void adelante() {
    analogWrite(ENA, potencia);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENB, potencia);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void atras() {
    analogWrite(ENA, potencia);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENB, potencia);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void izquierda() {
    analogWrite(ENA, potencia);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENB, potencia);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void derecha() {
    
    analogWrite(ENA, potencia);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENB, potencia);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void detener() {
    analogWrite(ENA, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENB, 0);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

