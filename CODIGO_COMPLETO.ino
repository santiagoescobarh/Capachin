#include <Arduino.h>
#include <Sensores.h>
#include <Motores.h>

class RobotController {
  public:
    // Constructor: inicializa el estado y el temporizador
    RobotController() : _estadoActual(Avanzar), _estadoTimer(0) {}

    // Método de inicialización: configura sensores (y otros si es necesario)
    void begin() {
      _sensores.begin();
      _estadoTimer = millis();
    }

    // Método que se debe llamar en loop() para actualizar el estado del robot
    void update() {
      int lecturaIzq    = _sensores.detectarSensor1();
      int lecturaCentro = _sensores.detectarSensor2();
      int lecturaDer    = _sensores.detectarSensor3();

      unsigned long tiempoActual = millis();

      // Si los tres sensores detectan objeto, se cambia a giro de 180°
      if (lecturaIzq == 1 && lecturaCentro == 1 && lecturaDer == 1) {
        _estadoActual = Girar_180;
        _estadoTimer = tiempoActual;
      }

      switch (_estadoActual) {
        case Avanzar:
          if (lecturaCentro == 1) {
            _motores.Alto();
            if (lecturaIzq == 1) {
              _estadoActual = Girar_derecha;
            } else if (lecturaDer == 1) {
              _estadoActual = Girar_izquierda;
            } else {
              _estadoActual = Girar_derecha;  // Por defecto, gira a la derecha
            }
            _estadoTimer = tiempoActual;
          } else {
            _motores.Adelante();
          }
          break;

        case Girar_izquierda:
          _motores.GirarIzquierda(_duracionGiro);
          _estadoActual = Avanzar;
          _estadoTimer = tiempoActual;
          break;

        case Girar_derecha:
          _motores.GirarDerecha(_duracionGiro);
          _estadoActual = Avanzar;
          _estadoTimer = tiempoActual;
          break;

        case Girar_180:
          _motores.Girar180(2 * _duracionGiro);
          _estadoActual = Avanzar;
          _estadoTimer = tiempoActual;
          break;

        case Parar:
          _motores.Alto();
          if (tiempoActual - _estadoTimer >= 500) {
            _estadoActual = Avanzar;
            _estadoTimer = tiempoActual;
          }
          break;
      }
    }

  private:
    // Instancias de sensores y motores (atributos privados)
    Sensores _sensores;
    Motores _motores;

    // Enumeración de estados para el comportamiento del robot
    enum Estados {
      Avanzar,
      Girar_izquierda,
      Girar_derecha,
      Girar_180,
      Parar
    };

    Estados _estadoActual;
    unsigned long _estadoTimer;
    const unsigned long _duracionGiro = 400;  // Duración del giro en milisegundos
};

// Instancia global del controlador del robot
RobotController robot;

void setup() {
  Serial.begin(115200);
  robot.begin();
}

void loop() {
  robot.update();
}

