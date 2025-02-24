from machine import Pin, PWM
from utime import sleep
class Motor:
 def __init__(self, pin1, pin2):
    #self.m1Dir = Pin(7 , Pin.OUT) # set motor direction
    self.m1Dir = Pin(pin1 , Pin.OUT) # set motor direction
    #self.pwm1 = PWM(Pin(6)) # set speed
    self.pwm1 = PWM(Pin(pin2)) # set speed
    self.pwm1.freq(1000) # set max frequency
    self.pwm1.duty_u16(0) # set duty cycle
    
 def off(self):
    self.pwm1.duty_u16(0)

 def Forward(self, speed):
    self.m1Dir.value(0) # forward = 0 reverse = 1 motor 1
    #self.pwm1.duty_u16(int(65535*100/100)) # speed range 0-100 motor 1
    self.pwm1.duty_u16(int(65535*speed/100)) # speed range 0-100 motor 1

 def Reverse(self, speed):
    self.m1Dir.value(1)
    #self.pwm1.duty_u16(int(65535*30/100))
    self.pwm1.duty_u16(int(65535*speed/100))

class TrackSensor(self, pin):
    def __init__(self, pin):
        self.pin = Pin(pin, Pin.IN, Pin.PULL_DOWN)
        self.value = self.pin.value()

motor1 = Motor(7,6)
motor2 = Motor(4,5)
sensor1 = TrackSensor(15)#right
sensor2 = TrackSensor(14)#left
speed = 30

def forward(motor1, motor2, speed):
    motor1.Forward(speed)
    motor2.Forward(speed)

def reverse(motor1,motor2, speed):
    motor1.Reverse(speed)
    motor2.Reverse(speed)

def rightTurn(motor1,motor2):
    motor1.Forward(30)
    motor2.Reverse(30)

def leftTurn(motor1,motor2):
    motor1.Reverse(30)
    motor2.Forward(30)

def stop(motor1, motor2):
    motor1.off()
    motor2.off()

while True:
    right_val = sensor1.value()
    left_val = sensor2.value()
    #right_val=right_ir.value() #Getting right IR value(0 or 1)
    #left_val=left_ir.value() #Getting left IR value(0 or 1)
    # Controlling robot direction based on IR value
    if right_val==0 and left_val==0:
        forward()
    elif right_val==1 and left_val==0:
        rightTurn()
    elif right_val==0 and left_val==1:
        leftTurn()
    else:
        stop()