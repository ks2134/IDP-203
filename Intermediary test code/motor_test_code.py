# Pin 0  PWM Motor 1               Ground---[]              []-------------------------------
# Pin 1  DIR Motor 1               Ground---[]              []------                     ----3V3
#                                  Ground---[]              []------                     ----Ground
# Pin 2  PWM Motor 2               Ground---[]              []-------------------------------
# Pin 3  DIR Motor 2               Ground---[]              []-------------------------------
# Pin 4  PWM Motor 3 or Servo 1    Ground---[]              []-------------------------------
# Pin 5  DIR Motor 3               Ground---[]              []-------------------------------
#                                  Ground...[]              []-------------------------------
# Pin 6  PWM Motor 4 or Servo 2    Ground---[]              []-------------------------------
# Pin 7  DIR Motor 4               Ground---[]              []-------------------------------
# Pin 8  GPIO...............................[]              []-------------------------------
# Pin 9  GPIO...............................[]              []-------------------------------
#                                  Ground                   []-------------------------------Ground
# Pin 10 GPIO...............................[]              []-------------------------------
# Pin 11 GPIO...............................[]              []-------------------------------Pin 20 GPIO
# Pin 12 GPIO...............................[]              []-------------------------------Pin 19 GPIO
# Pin 13 GPIO............... ..       ......[]              []-------------------------------Pin 18 GPIO
# Ground....................................[]              []-------------------------------Ground
# Pin 14 GPIO...............................[]              []-------------------------------Pin 17 GPIO
# Pin 15 GPIO...............................[]              []-------------------------------Pin 16 GPIO
from machine import Pin, PWM
from time import sleep

#MOTOR 1 SETUP
motor1 = Pin(4,Pin.OUT)
motor1_pwm = PWM(Pin(5))
motor1_pwm.freq(1000)


#MOTOR 2 SETUP
motor2 = Pin(7,Pin.OUT)
motor2_pwm = PWM(Pin(6))
motor2_pwm.freq(1000)


duty_cycle = (100)

def motor_cw(duty, motor, motor_pwm):
    motor.value(1)
    duty_16 = int((duty*65536)/100)
    motor_pwm.duty_u16(duty_16)
    
def motor_ccw(duty, motor, motor_pwm):
    motor.value(0)
    duty_16 = int((duty*65536)/100)
    motor_pwm.duty_u16(duty_16)
    
def stop_motor(motor, motor_pwm):
    motor.value(0)
    motor_pwm.duty_u16(0)
    

while True:
    motor_cw(duty_cycle, motor1, motor1_pwm)
    motor_cw(duty_cycle, motor2, motor2_pwm)
    sleep(2)
    stop_motor(motor1, motor1_pwm)
    stop_motor(motor2, motor2_pwm)
    sleep(2)
    motor_ccw(duty_cycle, motor1, motor1_pwm)
    motor_ccw(duty_cycle, motor2, motor2_pwm)
    sleep(2)
    stop_motor(motor1, motor1_pwm)
    stop_motor(motor2, motor2_pwm)
    sleep(2)
    