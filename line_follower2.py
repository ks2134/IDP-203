from machine import Pin, PWM
from time import sleep

LED_pin = Pin(11 , Pin.OUT)
LED_pin.value(1)

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
    self.m1Dir.value(1) # forward = 1 reverse = 0 motor 1
    #self.pwm1.duty_u16(int(65535*100/100)) # speed range 0-100 motor 1
    self.pwm1.duty_u16(int(65535*speed/100)) # speed range 0-100 motor 1

 def Reverse(self, speed):
    self.m1Dir.value(0)
    #self.pwm1.duty_u16(int(65535*30/100))
    self.pwm1.duty_u16(int(65535*speed/100))

#line is 2.5cm wide
class TrackSensor:
    def __init__(self, pin):
        self.sensor_pin = Pin(pin, Pin.IN, Pin.PULL_DOWN)
    
    def reading(self):
        return self.sensor_pin.value()
    

def forward(speed):
    motor1.Forward(speed)
    motor2.Forward(speed)

def reverse(speed):
    motor1.Reverse(speed)
    motor2.Reverse(speed)

def leftTurn(f, speed):
    motor1.Forward(speed)
    motor2.Forward(f*speed)

def rightTurn(f, speed):
    motor1.Forward(f*speed)
    motor2.Forward(speed)

def leftPivot(f,speed):
    motor1.Forward(speed)
    motor2.Reverse(f*speed)

def rightPivot(f, speed):
    motor1.Reverse(f*speed)
    motor2.Forward(speed)

def stop():
    motor1.off()
    motor2.off()
    
motor1 = Motor(7,6)
motor2 = Motor(4,5)
sensor_left = TrackSensor(15)
sensor_right = TrackSensor(14)
sensor_Tleft = TrackSensor(13)
sensor_Tright = TrackSensor(12)
#minimum speed for motors is 12
speed1 = 100
speed2 = 100
f1 = 0.7 #good for small corrections when straight lining
f2 = 0.5

def go_forward():
    right_val = sensor_right.reading()
    left_val = sensor_left.reading()
    Tright_val = sensor_Tright.reading()
    Tleft_val = sensor_Tleft.reading()

    if right_val==0 and left_val==0:
        forward(speed1)
    elif right_val==1 and left_val==0:
        rightTurn(f1,speed1)
    elif right_val==0 and left_val==1:
        leftTurn(f1,speed1) 
    else:
        forward(speed1)

def turn_right():
    while (left_val == 0):
        left_val = sensor_left.reading()
        rightPivot(f2,speed2)
    while (Tleft == 1):
        rightPivot(f2,speed2)

def turn_left():
    while (right_val == 0):
        right_val = sensor_right.reading()
        leftPivot(f2,speed2)
    while (Tright == 1):
        leftPivot(f2,speed2)

def detect_node():
    Tright_val = sensor_Tright.reading()
    Tleft_val = sensor_Tleft.reading()
    if (Tright_val == 1) or (Tleft_val == 1):
        return True
    else:
        return False

my_functions = {"L":turn_left, "R":turn_right, "0":go_forward}
test_route = ["0","L","0","R","R","L","0","L","L", "L","L","0","R","0","R","0","R","0"]
cur = -1

while cur < 18:
    if detect_node == False:
        go_forward()
    else:
        cur += 1
        my_functions[test_route[cur]]()

stop()
