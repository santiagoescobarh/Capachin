// Definición de pines para Motor A
#define IN1 15    // Dirección Motor A
#define IN2 2     // Dirección Motor A
#define ENA 4     // PWM Motor A (PWM control)

// Definición de pines para Motor B
#define IN3 16    // Dirección Motor B
#define IN4 17    // Dirección Motor B
#define ENB 18    // PWM Motor B (PWM control)

// Configuración para LEDC (PWM en ESP32)
const int freq = 5000;         // Frecuencia de PWM en Hz
const int resolution = 8;      // Resolución en bits (0-255)
const int channelA = 0;        // Canal LEDC para Motor A
const int channelB = 1;        // Canal LEDC para Motor B

// Clase Motor: encapsula la configuración y control de un motor
class Motor {
  private:
    int _in1;
    int _in2;
    int _en;
    int _channel;
    int _freq;
    int _resolution;
    
  public:
    // Constructor
    Motor(int in1, int in2, int en, int channel, int freq, int resolution) {
      _in1 = in1;
      _in2 = in2;
      _en = en;
      _channel = channel;
      _freq = freq;
      _resolution = resolution;
    }
    
    // Inicializa el pin de control y configura el canal PWM
    void init() {
      pinMode(_in1, OUTPUT);
      pinMode(_in2, OUTPUT);
      ledcSetup(_channel, _freq, _resolution);
      ledcAttachPin(_en, _channel);
    }
    
    // Mueve el motor hacia adelante a la velocidad especificada (valor por defecto: 255)
    void forward(uint8_t speed = 255) {
      digitalWrite(_in1, HIGH);
      digitalWrite(_in2, LOW);
      ledcWrite(_channel, speed);
    }
    
    // Mueve el motor hacia atrás a la velocidad especificada
    void backward(uint8_t speed = 255) {
      digitalWrite(_in1, LOW);
      digitalWrite(_in2, HIGH);
      ledcWrite(_channel, speed);
    }
    
    // Detiene el motor
    void stop() {
      digitalWrite(_in1, LOW);
      digitalWrite(_in2, LOW);
      ledcWrite(_channel, 0);
    }
};

// Clase Robot: gestiona dos motores para controlar la dirección general
class Robot {
  private:
    Motor motorA;
    Motor motorB;
    
  public:
    // Constructor que recibe dos objetos Motor
    Robot(Motor mA, Motor mB) : motorA(mA), motorB(mB) {}
    
    // Inicializa ambos motores
    void init() {
      motorA.init();
      motorB.init();
    }
    
    // Avanza ambos motores
    void forward(uint8_t speed = 255) {
      motorA.forward(speed);
      motorB.forward(speed);
      Serial.println("Ambos motores avanzando");
    }
    
    // Retrocede ambos motores
    void backward(uint8_t speed = 255) {
      motorA.backward(speed);
      motorB.backward(speed);
      Serial.println("Ambos motores en reversa");
    }
    
    // Giro a la derecha: motor A hacia adelante y motor B en reversa
    void turnRight(uint8_t speed = 255) {
      motorA.forward(speed);
      motorB.backward(speed);
      Serial.println("Giro a la derecha");
    }
    
    // Giro a la izquierda: motor A en reversa y motor B hacia adelante
    void turnLeft(uint8_t speed = 255) {
      motorA.backward(speed);
      motorB.forward(speed);
      Serial.println("Giro a la izquierda");
    }
    
    // Detiene ambos motores
    void stop() {
      motorA.stop();
      motorB.stop();
      Serial.println("Motores detenidos");
    }
};

// Instanciación global del Robot con dos motores
Robot robot(
  Motor(IN1, IN2, ENA, channelA, freq, resolution),
  Motor(IN3, IN4, ENB, channelB, freq, resolution)
);

void setup() {
  Serial.begin(115200);
  robot.init();
  Serial.println("Comandos: a=Adelante, b=Atrás, c=Derecha, d=Izquierda");
}

void loop() {
  if (Serial.available() > 0) {
    char comando = Serial.read();
    
    // Ignorar saltos de línea y retorno de carro
    if (comando == '\n' || comando == '\r') {
      return;
    }
    
    switch(comando) {
      case 'a':  // Avanzar
        robot.forward();
        break;
        
      case 'b':  // Retroceder
        robot.backward();
        break;
        
      case 'c':  // Giro a la derecha
        robot.turnRight();
        break;
        
      case 'd':  // Giro a la izquierda
        robot.turnLeft();
        break;
        
      default:  // Comando no reconocido: detener motores
        robot.stop();
        Serial.println("Comando no reconocido, motores detenidos");
        break;
    }
  }
}
