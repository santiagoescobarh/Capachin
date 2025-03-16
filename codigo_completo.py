from machine import Pin, PWM, I2C
import time, urandom

class VL53L0X:
    def __init__(self, i2c, address=0x29):
        self.i2c = i2c
        self.address = address
    def read_mm(self):
        return 100  # Reemplaza por lectura real

class MPU6050:
    def __init__(self, i2c, address=0x68):
        self.i2c = i2c
        self.address = address
        self.i2c.writeto_mem(self.address, 0x6B, b'\x00')
    def read_gyro_z(self):
        data = self.i2c.readfrom_mem(self.address, 0x47, 2)
        raw = data[0] | (data[1] << 8)
        if raw & 0x8000:
            raw -= 65536
        return raw / 131

class Motor:
    def __init__(self, in1_pin, in2_pin, en_pin, freq=1000):
        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)
        self.pwm = PWM(Pin(en_pin), freq=freq)
        self.pwm.duty(0)
    def forward(self, duty=512):
        self.in1.value(1)
        self.in2.value(0)
        self.pwm.duty(duty)
    def backward(self, duty=512):
        self.in1.value(0)
        self.in2.value(1)
        self.pwm.duty(duty)
    def stop(self):
        self.in1.value(0)
        self.in2.value(0)
        self.pwm.duty(0)

class Motors:
    def __init__(self, in1A, in2A, enA, in1B, in2B, enB, freq=1000):
        self.motorA = Motor(in1A, in2A, enA, freq)
        self.motorB = Motor(in1B, in2B, enB, freq)
    def adelante(self, duty=512):
        self.motorA.forward(duty)
        self.motorB.forward(duty)
    def atras(self, duty=512):
        self.motorA.backward(duty)
        self.motorB.backward(duty)
    def alto(self):
        self.motorA.stop()
        self.motorB.stop()
    def girar_izquierda(self, duty=512):
        self.motorA.backward(duty)
        self.motorB.forward(duty)
    def girar_derecha(self, duty=512):
        self.motorA.forward(duty)
        self.motorB.backward(duty)

class Robot:
    AVANZAR = 0
    GIRAR_IZQ = 1
    GIRAR_DER = 2
    GIRAR_180 = 3

    def __init__(self):
        self.i2c = I2C(1, scl=Pin(22), sda=Pin(21))
        self.sensor_left   = VL53L0X(self.i2c, 0x30)
        self.sensor_center = VL53L0X(self.i2c, 0x29)
        self.sensor_right  = VL53L0X(self.i2c, 0x31)
        self.gyro = MPU6050(self.i2c, 0x68)
        self.motors = Motors(5, 18, 19, 16, 17, 4, 1000)
        self.estado = Robot.AVANZAR
        self.t_inicio = time.ticks_ms()
        self.last_turn = None
        self.DIST_MIN = 70    # 7 cm en mm
        self.DUTY_BASE = 500
        self.TIEMPO_GIRO = 500
        self.TIEMPO_180 = 1000

    def cambiar_estado(self, nuevo_estado):
        self.estado = nuevo_estado
        self.t_inicio = time.ticks_ms()

    def actualizar(self):
        t_actual = time.ticks_ms()
        delta_t = time.ticks_diff(t_actual, self.t_inicio)
        dist_left   = self.sensor_left.read_mm()
        dist_center = self.sensor_center.read_mm()
        dist_right  = self.sensor_right.read_mm()
        gyro_z = self.gyro.read_gyro_z()

        if self.estado == Robot.AVANZAR:
            if (dist_center < self.DIST_MIN or dist_left < self.DIST_MIN or dist_right < self.DIST_MIN):
                self.motors.alto()
                time.sleep_ms(100)
                free_left   = dist_left   >= self.DIST_MIN
                free_center = dist_center >= self.DIST_MIN
                free_right  = dist_right  >= self.DIST_MIN
                if not free_center:
                    if free_left and free_right:
                        if self.last_turn == 'left':
                            self.cambiar_estado(Robot.GIRAR_DER)
                            self.last_turn = 'right'
                        else:
                            self.cambiar_estado(Robot.GIRAR_IZQ)
                            self.last_turn = 'left'
                    elif free_left:
                        self.cambiar_estado(Robot.GIRAR_IZQ)
                        self.last_turn = 'left'
                    elif free_right:
                        self.cambiar_estado(Robot.GIRAR_DER)
                        self.last_turn = 'right'
                    else:
                        self.cambiar_estado(Robot.GIRAR_180)
                else:
                    if not free_left:
                        self.cambiar_estado(Robot.GIRAR_DER)
                        self.last_turn = 'right'
                    elif not free_right:
                        self.cambiar_estado(Robot.GIRAR_IZQ)
                        self.last_turn = 'left'
                    else:
                        self.cambiar_estado(Robot.AVANZAR)
                return
            else:
                self.motors.adelante(self.DUTY_BASE)
        elif self.estado == Robot.GIRAR_IZQ:
            if delta_t < self.TIEMPO_GIRO:
                self.motors.girar_izquierda(self.DUTY_BASE)
            else:
                self.cambiar_estado(Robot.AVANZAR)
        elif self.estado == Robot.GIRAR_DER:
            if delta_t < self.TIEMPO_GIRO:
                self.motors.girar_derecha(self.DUTY_BASE)
            else:
                self.cambiar_estado(Robot.AVANZAR)
        elif self.estado == Robot.GIRAR_180:
            if delta_t < self.TIEMPO_180:
                self.motors.girar_derecha(self.DUTY_BASE)
            else:
                self.cambiar_estado(Robot.AVANZAR)

def main():
    robot = Robot()
    while True:
        robot.actualizar()
        time.sleep_ms(50)

if __name__ == "__main__":
    main()
