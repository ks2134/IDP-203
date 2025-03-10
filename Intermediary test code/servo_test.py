from machine import Pin, PWM 
from time import sleep 
# Set up PWM Pin for servo control 
servo_pin = machine.Pin(15) 
servo = PWM(servo_pin) 
# Set Duty Cycle for Different Angles 
max_duty = 7864 
min_duty = 1802
start_duty = 1500 #approx 0 degrees
end_duty = 2000 # approx 10 degrees

half_duty = int(max_duty/2) 
#Set PWM frequency 
frequency = 50 
servo.freq(frequency) 

#Servo at 0 degrees 
servo.duty_u16(start_duty) 
sleep(2) 
#Servo at 90 degrees 
servo.duty_u16(end_duty) 
sleep(2)  



