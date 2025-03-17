from machine import Pin, PWM, I2C
import time

# Configuración de pines del puente H L298N
ENA = PWM(Pin(5))   # PWM Motor A
IN1 = Pin(18, Pin.OUT)  # Dirección Motor A
IN2 = Pin(19, Pin.OUT)  # Dirección Motor A
ENB = PWM(Pin(25))  # PWM Motor B
IN3 = Pin(4, Pin.OUT)  # Dirección Motor B
IN4 = Pin(23, Pin.OUT)  # Dirección Motor B

# Configuración de pines XSHUT de los sensores
XSHUT_FRONTAL = Pin(32, Pin.OUT)
XSHUT_LATERAL = Pin(33, Pin.OUT)

# Potencia del motor (0-1023 en MicroPython)
potencia = 512  # Ajustado para MicroPython (0-1023)

# Inicialización de I2C para los sensores
i2c = I2C(0, scl=Pin(22), sda=Pin(21))

# Direcciones I2C de los sensores
FRONTAL_ADDR = 0x30
LATERAL_ADDR = 0x31

# Configuración inicial de los sensores
XSHUT_FRONTAL.value(0)
XSHUT_LATERAL.value(0)
time.sleep_ms(10)

# Encender e inicializar sensor frontal
XSHUT_FRONTAL.value(1)
time.sleep_ms(10)
# Aquí debes inicializar el sensor frontal con la dirección FRONTAL_ADDR

# Encender e inicializar sensor lateral
XSHUT_LATERAL.value(1)
time.sleep_ms(10)
# Aquí debes inicializar el sensor lateral con la dirección LATERAL_ADDR

def medir_distancia(sensor_addr):
    # Aquí debes implementar la lectura del sensor VL53L0X
    # Esto depende de la biblioteca que uses para el sensor en MicroPython
    # Por ahora, devolvemos un valor fijo para pruebas
    return 200  # Valor de ejemplo

def adelante():
    ENA.duty(potencia)
    IN1.value(1)
    IN2.value(0)
    ENB.duty(potencia)
    IN3.value(1)
    IN4.value(0)

def atras():
    ENA.duty(potencia)
    IN1.value(0)
    IN2.value(1)
    ENB.duty(potencia)
    IN3.value(0)
    IN4.value(1)

def izquierda():
    ENA.duty(potencia)
    IN1.value(1)
    IN2.value(0)
    ENB.duty(potencia)
    IN3.value(0)
    IN4.value(1)

def derecha():
    ENA.duty(potencia)
    IN1.value(0)
    IN2.value(1)
    ENB.duty(potencia)
    IN3.value(1)
    IN4.value(0)

def detener():
    ENA.duty(0)
    IN1.value(0)
    IN2.value(0)
    ENB.duty(0)
    IN3.value(0)
    IN4.value(0)

def loop():
    while True:
        distancia_frontal = medir_distancia(FRONTAL_ADDR)
        distancia_lateral = medir_distancia(LATERAL_ADDR)
        
        print("Distancia Frontal:", distancia_frontal)
        print("Distancia Lateral:", distancia_lateral)
        
        if distancia_frontal > 100:
            adelante()
            print("Avanzando")
        else:
            detener()
            time.sleep_ms(5)
            
            if distancia_frontal <= 100 and distancia_lateral <= 100:
                izquierda()
                print("Girando a la izquierda")
            elif distancia_lateral > 100:
                derecha()
                print("Girando a la derecha")
            else:
                izquierda()
                print("Girando a la izquierda")
        
        time.sleep_ms(5)

# Ejecutar el bucle principal
loop()
