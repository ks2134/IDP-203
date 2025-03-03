from machine import Pin, PWM
from time import sleep

LED_pin = Pin(11 , Pin.OUT)
LED_pin.value(1)

F1_ORIGINAL = 0.7

f1 = F1_ORIGINAL #good for small corrections when straight lining
f2 = 0.5

line_correction = 0.2

previous_state = [0, 0]     #Store value of previous action to keep robot centred [right_val, left_val]
state_counter = 0       #Store how many times the same previous action has been called, for PI control.
state_counter_trip = 500

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
    global state_counter
    global line_correction
    global f1
    global F1_ORIGINAL

    if previous_state != [0, 0]:
        state_counter = 0
        f1 = F1_ORIGINAL
        
        motor1.Forward(speed)
        motor2.Forward(speed)
    
    else:
        motor1.Forward(speed)
        motor2.Forward(speed)

def reverse(speed):
    motor1.Reverse(speed)
    motor2.Reverse(speed)

def leftTurn(f, speed):
    global state_counter
    global line_correction
    global f1
    global F1_ORIGINAL
    global state_counter_trip

    if previous_state != [0, 1]:
        state_counter = 0
        f1 = F1_ORIGINAL
    
    elif previous_state == [0, 1]:
        state_counter += 1
        
        motor1.Forward(speed)
        motor2.Forward(f*speed)

        if (state_counter >= state_counter_trip) and (f1 > 0.0):
            f1 -= line_correction
            state_counter = 0

def rightTurn(f, speed):
    global state_counter
    global line_correction
    global f1
    global F1_ORIGINAL
    global state_counter_trip

    if previous_state != [1, 0]:
        state_counter = 0
        f1 = F1_ORIGINAL
    
    elif previous_state == [1, 0]:
        state_counter += 1

        motor1.Forward(f*speed)
        motor2.Forward(speed)

        if (state_counter >= state_counter_trip) and (f1 > 0.0):
            f1 -= line_correction
            state_counter = 0

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
speed1 = 90
speed2 = 100


def go_forward():
    right_val = sensor_right.reading()
    left_val = sensor_left.reading()

    global previous_state

    if right_val==0 and left_val==0:
        forward(speed1)
        previous_state = [0, 0]

    elif right_val==1 and left_val==0:
        rightTurn(f1,speed1)
        previous_state = [1, 0]

    elif right_val==0 and left_val==1:
        leftTurn(f1,speed1)
        previous_state = [0, 1]

    else:
        previous_state = [1, 1]
        forward(speed1)

def turn_Cright(): #corner
    left_val = sensor_left.reading()
    while (left_val ==0):
        left_val = sensor_left.reading()
        rightPivot(f2,speed2)

def turn_right():
    Tleft_val = sensor_Tleft.reading()
    left_val = sensor_left.reading()
    while (Tleft_val == 0):
        Tleft_val = sensor_Tleft.reading()
        rightPivot(f2, speed2)
    while (Tleft_val == 1):
        Tleft_val = sensor_Tleft.reading()
        rightPivot(f2, speed2)
    while (left_val == 0):
        left_val = sensor_left.reading()
        rightPivot(f2, speed2)

def turn_Tright():
    left_val = sensor_left.reading()
    while (left_val ==1):
        left_val = sensor_left.reading()
        rightPivot(f2,speed2)
    while (left_val ==0):
        left_val = sensor_left.reading()
        rightPivot(f2,speed2)
    

def turn_Cleft(): #corner
    right_val = sensor_right.reading()
    while (right_val == 0):
        right_val = sensor_right.reading()
        leftPivot(f2,speed2)

def turn_left(): 
    Tright_val = sensor_Tright.reading()
    right_val = sensor_right.reading()
    while (Tright_val == 0):
        Tright_val = sensor_Tright.reading()
        leftPivot(f2, speed2)
    while (Tright_val == 1):
        Tright_val = sensor_Tright.reading()
        leftPivot(f2, speed2)
    while (right_val == 0):
        right_val = sensor_right.reading()
        leftPivot(f2, speed2)

def turn_Tleft(): 
    right_val = sensor_right.reading()
    while (right_val ==1):
        right_val = sensor_right.reading()
        leftPivot(f2,speed2)
    while (right_val ==0):
        right_val = sensor_right.reading()
        leftPivot(f2,speed2)

def ignore():
    Tleft_val = sensor_Tleft.reading()
    Tright_val = sensor_Tright.reading()
    while ((Tleft_val == 1) or (Tright_val == 1)):
        Tleft_val = sensor_Tleft.reading()
        Tright_val = sensor_Tright.reading()
        go_forward()

def detect_node(next_node):
    # if next_node == "L":
    #     Tleft_val = sensor_Tleft.reading()
    #     return (Tleft_val == 1)
    # elif next_node == "R":
    #     Tright_val = sensor_Tright.reading()
    #     return (Tright_val == 1)
    # else:
    Tleft_val = sensor_Tleft.reading()
    Tright_val = sensor_Tright.reading()
    
    return ((Tright_val == 1) or (Tleft_val == 1))

my_functions = {"L":turn_left, "R":turn_right, "S":ignore, "CR":turn_Cright, "CL":turn_Cleft,"TR":turn_Tright,"TL":turn_Tleft}
test_route = [ "CL", "S", "L", "S", "TL", "S", "TL"]
#S for straight, CR CL for corners, TL TR for head on t, L and R for side ts
node_route = [11, 10, 7, 8, 9, 13, 12]
#indices represent node index
node_route_type = ["T", "T", "T", "T", "T", "T", "C", "T", "T", "T", "T", "C", "T", "T", "T", "T", "P", "P", "P", "P"] #T for t junction, c for corner, p for plus
cur = -1

while cur < (len(node_route) -1):
    next_node = test_route[cur + 1]
    node = detect_node(next_node)
    if node == False:
        go_forward()
    elif node == True:
        print(cur)
        my_functions[next_node]()
        cur += 1

stop()



