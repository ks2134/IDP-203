from machine import Pin, PWM, I2C
from time import sleep, time
from colour_sensor import TCS34725
#from distance_sensor import VL53L0X


#Driving motor on robot. Inputs: pin1 = motor direction, pin2 = pwm pin
class Motor:
   """
   Description
   -----------
   Class representing one of the driving motors on the robot and sets up control for the two directions the robot can travel in 
   (Forward and Reverse).
   
   Attributes
   ----------
   dir_pin : int
      Direction control pin for the motor (see pinout diagram for Pi HAT for the correct pin for each motor)
   pwm_pin : int
      PWM pin for speed control for the motor (again see pinout diagram for the correct pin values)
   
   Methods
   -------
   off()
      Turn motor off
   Forward(speed: int)
      Drive motor in the forward direction at specified speed (0 to 100)
   Reverse(speed: int)
      Drive motor in reverse direction at specified speed (0 to 100)   
   """

   def __init__(self, dir_pin: int, pwm_pin: int):
      #self.m1Dir = Pin(7 , Pin.OUT) # set motor direction
      self.m1Dir = Pin(dir_pin , Pin.OUT) # set motor direction
      #self.pwm1 = PWM(Pin(6)) # set speed
      self.pwm1 = PWM(Pin(pwm_pin)) # set speed
      self.pwm1.freq(1000) # set max frequency
      self.pwm1.duty_u16(0) # set duty cycle
      
   def off(self):
      """Turn off the motor (sets duty cycle to 0)"""
      self.pwm1.duty_u16(0)

   def Forward(self, speed: int):
      """Drives motor in forward direction at specified speed (0 to 100)"""
      self.m1Dir.value(1) # forward = 1 reverse = 0 motor 1
      #self.pwm1.duty_u16(int(65535*100/100)) # speed range 0-100 motor 1
      self.pwm1.duty_u16(int(65535*speed/100)) # speed range 0-100 motor 1

   def Reverse(self, speed: int):
      """Drives motor in reverse direction at specified speed (0 to 100)"""
      self.m1Dir.value(0)
      #self.pwm1.duty_u16(int(65535*30/100))
      self.pwm1.duty_u16(int(65535*speed/100))


class TrackSensor:
   """
   Description
   -----------
   Class representing a line sensor on the robot, which is used for general line following, or for detecting and implimenting turns.

   Attributes
   ----------
   pin: int
      GPIO Pin the data output of the sensor is connected to.

   Methods
   -------
   reading() -> int
      Returns the sensor reading at that instant (1 - white, 0 - black)
   """
   def __init__(self, pin: int):
      self.sensor_pin = Pin(pin, Pin.IN, Pin.PULL_DOWN)

   def reading(self) -> int:
      """Return current value of sensor: 1 - white, 0 - black"""
      return self.sensor_pin.value()
   
   
class Vehicle:
   """
   Description
   -----------
   Class used to set up a Vehicle (whole robot).

   Attributes
   ----------
   left_motor_dir: int
      Direction GPIO pin number for left motor
   left_motor_pwm: int
      PWM GPIO pin number for left motor
   right_motor_dir: int
      Direction GPIO pin number for right motor
   right_motor_pwm: int
      PWM GPIO pin number for right motor
   
   left_track: int
      GPIO Pin for inner left line sensor
   right_track: int
      GPIO Pin for inner right line sensor
   Tleft_track: int
      GPIO Pin for outer left line sensor
   Tright_track: int
      GPIO Pin for outer right line sensor

   led_pin: int
      GPIO Pin to output to flashing LED subsystem. Set high or low to turn flashing on/off respectively.
   speed: int
      Speed value for robot (range 0-100). Class accounts for balancing of motors.
   i2c0_sda: int
      SDA pin used for I2C bus 0 (this is used for colour sensor)
   i2c0_scl: int
      SCL pin used for I2C bus 0 (this is used for colour sensor)
   i2c0_bus_no: int
      I2C bus being used for colour sensor (set as 0 in this case)

   max_servo_pos: int
      Servo position value for upper setting - used when carrying box.
   min_servo_pos: int
      Servo position value for lower setting - used when picking/dropping off box.
   servo_pin: int
      Data PWM pin for the servo.
   
   button_pin: int
      GPIO pin for push button input.
   
   Methods
   -------

   """
   def __init__(self, left_motor_dir:int, left_motor_pwm:int, right_motor_dir:int, right_motor_pwm:int,
                 left_track:int, right_track:int, Tleft_track:int, Tright_track:int, led_pin:int, speed:int, 
                 i2c0_sda:int, i2c0_scl:int, i2c0_bus_no:int, max_servo_pos:int, min_servo_pos:int, servo_pin:int, 
                 button_pin:int):
      
      #Setting up driving motors
      self.left_motor = Motor(left_motor_dir, left_motor_pwm)
      self.right_motor = Motor(right_motor_dir, right_motor_pwm)

      #Setting up line sensors
      self.sensor_left = TrackSensor(left_track)
      self.sensor_right = TrackSensor(right_track)
      self.sensor_Tleft = TrackSensor(Tleft_track)
      self.sensor_Tright = TrackSensor(Tright_track)

      #Setting up colour sensor:
      self.i2c0_bus = I2C(i2c0_bus_no, sda=i2c0_sda, scl=i2c0_scl)
      self.colour_sensor = TCS34725(self.i2c0_bus)

      #Setting up miscellaneous peripheries
      self.led = Pin(led_pin, Pin.OUT)
      self.button = Pin(button_pin, Pin.IN, Pin.PULL_DOWN)

      #Set slightly different motor speeds, in order to balance the two motors
      self.left_motor_speed = speed
      self.right_motor_speed = 0.97 * speed

      #Initialise servo
      self.max_servo_pos = max_servo_pos
      self.min_servo_pos = min_servo_pos
      self.servo = PWM(Pin(servo_pin))
      self.servo.freq(50)                 #PWM Frequency set at 50Hz

      #Setting up distance sensor
      #self.i2c1_bus = I2C(i2c1_bus_no, sda=i2c1_sda, scl=i2c1_scl)
      #self.distance_sensor = VL53L0X(self.i2c1_bus)
      # self.distance_sensor.set_measurement_timing_budget(40000)
      # self.distance_sensor.set_Vcsel_pulse_period(self.distance_sensor.vcsel_period_type[0], 8)
      
   #Move both motors in forward direction.
   def forward(self, getting_box=1, previous_state: list = [0, 0], state_counter=0):
      
      if previous_state != [0, 0]:
         state_counter = 0
         
         self.left_motor.Forward(self.left_motor_speed * getting_box)
         self.right_motor.Forward(self.right_motor_speed * getting_box)
      
      return state_counter
      
   #Move both motors in reverse direction.
   def reverse(self):
      self.left_motor.Reverse(self.left_motor_speed)
      self.right_motor.Reverse(self.right_motor_speed)

   #Slight adjustment left for line following.
   def leftTurn(self, previous_state:list, state_counter:int, state_counter_trip:int, line_correction:int,
                 F1_ORIGINAL:float, current_f:float, getting_box:float):

      if previous_state != [0, 1]:
         state_counter = 0
         current_f = F1_ORIGINAL
         
      elif previous_state == [0, 1]:
         state_counter += 1
            
         self.right_motor.Forward((self.right_motor_speed) * getting_box)
         self.left_motor.Forward(current_f * (self.left_motor_speed) * getting_box)

         if (state_counter >= state_counter_trip) and (current_f > 0.0):
            current_f -= line_correction
            state_counter = 0

      return state_counter, current_f

   #Slight adjustment right for line following.
   def rightTurn(self, previous_state:list, state_counter:int, state_counter_trip:int, line_correction:int,
                  F1_ORIGINAL:float, current_f:float, getting_box:float):

      if previous_state != [1, 0]:
         state_counter = 0
         current_f = F1_ORIGINAL
      
      elif previous_state == [1, 0]:
         state_counter += 1

         self.right_motor.Forward(current_f * (self.right_motor_speed) * getting_box)
         self.left_motor.Forward((self.left_motor_speed) * getting_box)

         if (state_counter >= state_counter_trip) and (current_f > 0.0):
            current_f -= line_correction
            state_counter = 0

      return state_counter, current_f

   #Major turn to the left.
   def leftPivot(self, f:float, previous_state:list):

      self.right_motor.Forward(self.right_motor_speed)
      self.left_motor.Reverse(f * self.left_motor_speed)

      previous_state = [1, 1]
      return previous_state

   #Major turn to the right.
   def rightPivot(self, f:float, previous_state:list):

      self.right_motor.Reverse(f * self.right_motor_speed)
      self.left_motor.Forward(self.left_motor_speed)

      previous_state = [1, 1]
      return previous_state

   #Bring robot to complete halt.
   def stop(self):
      self.right_motor.off()
      self.left_motor.off()
               
   
   #Normal forward driving algorithm with line following
   def go_forward(self, previous_state:list, F1_ORIGINAL:float, current_f:float, state_counter:int, line_correction:int,
                   state_counter_trip:int, getting_box:float):
      
      right_val = self.sensor_right.reading()
      left_val = self.sensor_left.reading()

      if right_val==0 and left_val==0:
         state_counter = self.forward(getting_box, previous_state, state_counter)
         previous_state = [0, 0]

      elif right_val==1 and left_val==0:
         state_counter, current_f = self.rightTurn(previous_state, state_counter, state_counter_trip, line_correction, F1_ORIGINAL, current_f, getting_box)

         previous_state = [1, 0]

      elif right_val==0 and left_val==1:
         state_counter, current_f = self.leftTurn(previous_state, state_counter, state_counter_trip, line_correction, F1_ORIGINAL, current_f, getting_box)
         previous_state = [0, 1]

      else:
         previous_state = [1, 1]
         state_counter = self.forward(getting_box, previous_state, state_counter)

      return previous_state, state_counter, current_f
   
   #Turning right on a corner (CR)
   def turn_Cright(self, f:float, previous_state:list):

      sleep(0.1)
      left_val = self.sensor_left.reading()
      while (left_val == 1):
         left_val = self.sensor_left.reading()
         previous_state = self.rightPivot(f, previous_state)

      while (left_val == 0):
         left_val = self.sensor_left.reading()
         previous_state = self.rightPivot(f, previous_state)

      return previous_state

   #Turning left on a corner (CL)
   def turn_Cleft(self, f:float, previous_state:list): #corner

      sleep(0.1)
      right_val = self.sensor_right.reading()
      while (right_val == 1):
         right_val = self.sensor_right.reading()
         previous_state = self.leftPivot(f, previous_state)

      while (right_val == 0):
         right_val = self.sensor_right.reading()
         previous_state = self.leftPivot(f, previous_state)

      return previous_state
         
   #Turning right on a 'sideways T' (R)
   def turn_right(self, f:float, previous_state:list):

      Tleft_val = self.sensor_Tleft.reading()
      left_val = self.sensor_left.reading()

      while (Tleft_val == 0):
         Tleft_val = self.sensor_Tleft.reading()
         previous_state = self.rightPivot(f, previous_state)

      while (Tleft_val == 1):
         Tleft_val = self.sensor_Tleft.reading()
         previous_state = self.rightPivot(f, previous_state)

      while (left_val == 1):
         left_val = self.sensor_left.reading()
         previous_state = self.rightPivot(f, previous_state)

      while (left_val == 0):
         left_val = self.sensor_left.reading()
         previous_state = self.rightPivot(f, previous_state)

      return previous_state

   #Turning left on a 'sideways T' (L)
   def turn_left(self, f:float, previous_state:list):

      Tright_val = self.sensor_Tright.reading()
      right_val = self.sensor_right.reading()
      
      while (Tright_val == 0):
         Tright_val = self.sensor_Tright.reading()
         previous_state = self.leftPivot(f, previous_state)

      while (Tright_val == 1):
         Tright_val = self.sensor_Tright.reading()
         previous_state = self.leftPivot(f, previous_state)

      while (right_val == 1):
         right_val = self.sensor_right.reading()
         previous_state = self.leftPivot(f, previous_state)

      while (right_val == 0):
         right_val = self.sensor_right.reading()
         previous_state = self.leftPivot(f, previous_state)

      return previous_state

   #Turning right on 'head-on T' (TR)
   def turn_Tright(self, f:float, previous_state:list):
      left_val = self.sensor_left.reading()
      
      while (left_val == 1):
         left_val = self.sensor_left.reading()
         previous_state = self.rightPivot(f, previous_state)

      while (left_val == 0):
         left_val = self.sensor_left.reading()
         previous_state = self.rightPivot(f, previous_state)

      return previous_state

   #Turning left on 'head-on T' (TL)
   def turn_Tleft(self, f:float, previous_state:list): 
      right_val = self.sensor_right.reading()
      
      while (right_val == 1):
         right_val = self.sensor_right.reading()
         previous_state = self.leftPivot(f, previous_state)

      while (right_val == 0):
         right_val = self.sensor_right.reading()
         previous_state = self.leftPivot(f, previous_state)

      return previous_state


   #Continuing straight and ignoring any alternate paths (S)
   def ignore(self, previous_state:list, F1_ORIGINAL:float, current_f:float, state_counter:int, line_correction:int,
              state_counter_trip:int, getting_box:float):
      
      Tleft_val = self.sensor_Tleft.reading()
      Tright_val = self.sensor_Tright.reading()

      while ((Tleft_val == 1) or (Tright_val == 1)):
         Tleft_val = self.sensor_Tleft.reading()
         Tright_val = self.sensor_Tright.reading()

         self.go_forward(previous_state, F1_ORIGINAL, current_f, state_counter, line_correction, state_counter_trip, getting_box)

   #Go back and drive off facing left after picking up box
   def reverse_left(self, f:float, previous_state:list):
      right_val = self.sensor_right.reading()

      while (right_val == 0):
         right_val = self.sensor_right.reading()
         previous_state = self.leftPivot(f, previous_state)

      while (right_val == 1):
         right_val = self.sensor_right.reading()
         previous_state = self.leftPivot(f, previous_state)

      while (right_val == 0):
         right_val = self.sensor_right.reading()
         previous_state = self.leftPivot(f, previous_state)

      return previous_state

   #Go back and drive off facing right after picking up box
   def reverse_right(self, f:float, previous_state:list):
      left_val = self.sensor_left.reading()

      while (left_val == 0):
         left_val = self.sensor_left.reading()
         previous_state = self.rightPivot(f, previous_state)

      while (left_val == 1):
         left_val = self.sensor_left.reading()
         previous_state = self.rightPivot(f, previous_state)

      while (left_val == 0):
         left_val = self.sensor_left.reading()
         previous_state = self.rightPivot(f, previous_state)

      return previous_state

   #After dropping off the box, perform 180 degree turn to the left.
   def spin_left(self, f:float, previous_state:list):
      state_counter = self.forward()
      sleep(0.5)
      self.servo.duty_u16(self.min_servo_pos) #drops box

      self.reverse()
      sleep(0.5)

      right_val = self.sensor_right.reading()

      while (right_val == 1):
         right_val = self.sensor_right.reading()
         previous_state = self.leftPivot(f, previous_state)

      while (right_val == 0):
         right_val = self.sensor_right.reading()
         previous_state = self.leftPivot(f, previous_state)

      return previous_state

   #After dropping off the box, perform 180 degree turn to the right.
   def spin_right(self, f:float, previous_state:list):
      state_counter = self.forward()
      sleep(0.5)
      self.servo.duty_u16(self.min_servo_pos) #drops box

      self.reverse()
      sleep(0.5)

      left_val = self.sensor_left.reading()

      while (left_val == 1):
         left_val = self.sensor_left.reading()
         previous_state = self.rightPivot(f, previous_state)

      while (left_val == 0):
         left_val = self.sensor_left.reading()
         previous_state = self.rightPivot(f, previous_state)

      return previous_state

   #Turning in right to pick up box.
   def box_right(self, f:float, previous_state:list):

      self.forward(previous_state)
      sleep(0.32)

      previous_state = self.rightPivot(f, previous_state)
      sleep(0.75)

      return previous_state

   #Turning in left to pick up box.
   def box_left(self, f:float, previous_state:list):    
      
      self.forward(previous_state)
      sleep(0.28)

      previous_state = self.leftPivot(f, previous_state)
      sleep(0.7)

      return previous_state


   #Pick up box without using distance sensor.
   def get_box(self) -> int:
      #print("get box func")
      self.servo.duty_u16(self.max_servo_pos) #picks up box
      self.stop()
      sleep(0.5)

      RGB_inc = self.get_colour()

      Tleft_val = self.sensor_Tleft.reading()
      Tright_val = self.sensor_Tright.reading()
      
      while ((Tleft_val == 1) or (Tright_val == 1)):
         #print("go back")
         Tleft_val = self.sensor_Tleft.reading()
         Tright_val = self.sensor_Tright.reading()
         self.reverse()

      return RGB_inc
   
   #Take 5 colour sensor readings, average them, and decide which colour box it.
   def get_colour(self) -> int:
      #[red, green, blue]
      colour_values = [[], [], []]

      for i in range(5):
         colour_sensor_reading = self.colour_sensor.read('rgb')
         red, green, blue = colour_sensor_reading[0], colour_sensor_reading[1], colour_sensor_reading[2]
         colour_values[0].append(red)
         colour_values[1].append(green)
         colour_values[2].append(blue)

      avg_colour_values = []
      for i in range(len(colour_values)):
         avg_colour_values.append(int(sum(colour_values[i]) / len(colour_values)))

      maximum_colour_reading = max(avg_colour_values)
      avg_red = avg_colour_values[0]
      avg_green = avg_colour_values[1]
      avg_blue = avg_colour_values[2]

      #colour_sensor_reading format: (r, g, b, c)
      
      #Checking for green
      if ((avg_green == avg_blue) and (avg_red != maximum_colour_reading)):
         RGB_inc = 2

      #Checking for red
      if maximum_colour_reading == avg_red:
         RGB_inc = 1

      #Checking for blue
      elif maximum_colour_reading == avg_blue:
         RGB_inc = 2

      #Checking for yellow
      elif maximum_colour_reading == avg_green:
         RGB_inc = 1

      return RGB_inc


      # if ((colour_sensor_reading[1] == colour_sensor_reading[2]) or ((colour[1] - 1) == colour[2]) or ((colour[1] + 1) == colour[2])): #green
         #    RGB_inc = 2
         # elif ((colour[2] >= colour[1]) and (colour[2] >= colour[0])): #blue
         #    RGB_inc = 2
         # elif ((colour[1] >= colour[0]) and (colour[1] >= colour[2])): #yellow
         #    RGB_inc = 1
         # elif ((colour[0] >= colour[1]) and (colour[0] >= colour[2])): #red
         #    RGB_inc = 1
         # #RGB_inc = random.randint(1,2)
         # return RGB_inc

      #Starting position manoeuvring
   

   #Detect and move outside the starting zone, onto track.
   def start(self, previous_state:list, F1_ORIGINAL:float, current_f:float, state_counter:int, line_correction:int,
              state_counter_trip:int, getting_box:float):
      
      Tleft_val = self.sensor_Tleft.reading()
      Tright_val = self.sensor_Tright.reading()

      while ((Tleft_val == 0) or (Tright_val == 0)):
         Tleft_val = self.sensor_Tleft.reading()
         Tright_val = self.sensor_Tright.reading()
         
         previous_state, state_counter, current_f = self.go_forward(previous_state, F1_ORIGINAL, current_f, state_counter, line_correction, state_counter_trip, getting_box)

      while ((Tleft_val == 1) or (Tright_val == 1)):
         Tleft_val = self.sensor_Tleft.reading()
         Tright_val = self.sensor_Tright.reading()
         
         previous_state, state_counter, current_f = self.go_forward(previous_state, F1_ORIGINAL, current_f, state_counter, line_correction, state_counter_trip, getting_box)

      return previous_state, state_counter, current_f


      #End up back in the box.
   
   #Come off the track and stop in starting zone.
   def finish(self, previous_state:list, F1_ORIGINAL:float, current_f:float, state_counter:int, line_correction:int,
               state_counter_trip:int, getting_box:float):
      
      Tleft_val = self.sensor_Tleft.reading()
      Tright_val = self.sensor_Tright.reading()

      while ((Tleft_val == 1) or (Tright_val == 1)):
         Tleft_val = self.sensor_Tleft.reading()
         Tright_val = self.sensor_Tright.reading()
         
         previous_state, state_counter, current_f = self.go_forward(previous_state, F1_ORIGINAL, current_f, state_counter, line_correction, state_counter_trip, getting_box)

      dummy = self.forward()
      sleep(0.7)

      return previous_state, state_counter, current_f


   #Identify whether a node on the track has been passed.
   def detect_node(self):
      Tleft_val = self.sensor_Tleft.reading()
      Tright_val = self.sensor_Tright.reading()
      
      return ((Tright_val == 1) or (Tleft_val == 1))
