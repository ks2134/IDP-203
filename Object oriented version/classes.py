from machine import Pin, PWM, I2C
from time import sleep, sleep_ms
from colour_sensor import TCS34725
from distance_sensor import VL53L0X

#Driving motor on robot. Inputs: pin1 = motor direction, pin2 = pwm pin
class Motor:
   def __init__(self, dir_pin, pwm_pin):
      #self.m1Dir = Pin(7 , Pin.OUT) # set motor direction
      self.m1Dir = Pin(dir_pin , Pin.OUT) # set motor direction
      #self.pwm1 = PWM(Pin(6)) # set speed
      self.pwm1 = PWM(Pin(pwm_pin)) # set speed
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


class TrackSensor:
   def __init__(self, pin):
      self.sensor_pin = Pin(pin, Pin.IN, Pin.PULL_DOWN)

   def reading(self):
      return self.sensor_pin.value()
   
   
class Vehicle:
   def __init__(self, left_motor_dir, left_motor_pwm, right_motor_dir, right_motor_pwm,
                 left_track, right_track, Tleft_track, Tright_track, led_pin, speed, 
                 i2c0_sda, i2c0_scl, i2c0_bus_no, max_servo_pos, min_servo_pos, servo_pin,
                 i2c1_sda, i2c1_scl, i2c1_bus_no):
      
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

      #Set slightly different motor speeds, in order to balance the two motors
      self.left_motor_speed = speed
      self.right_motor_speed = 0.97 * speed

      #Initialise servo
      self.max_servo_pos = max_servo_pos
      self.min_servo_pos = min_servo_pos
      self.servo = PWM(Pin(servo_pin))
      self.servo.freq(50)        #PWM Frequency set at 50Hz

      #Setting up distance sensor
      self.i2c1_bus = I2C(i2c1_bus_no, sda=i2c1_sda, scl=i2c1_scl)
      self.distance_sensor = VL53L0X(self.i2c1_bus)
      self.distance_sensor.set_measurement_timing_budget(40000)
      self.distance_sensor.set_Vcsel_pulse_period(self.distance_sensor.vcsel_period_type[0], 8)

      

   def forward(self, previous_state=[0, 0], state_counter=0):
      
      if previous_state != [0, 0]:
         state_counter = 0
         
      self.left_motor.Forward(self.left_motor_speed)
      self.right_motor.Forward(self.right_motor_speed)
      
      
      return state_counter
      

   def reverse(self):
      self.left_motor.Reverse(self.left_motor_speed)
      self.right_motor.Reverse(self.right_motor_speed)


   def leftTurn(self, previous_state, state_counter, state_counter_trip, line_correction, F1_ORIGINAL, current_f):

      if previous_state != [0, 1]:
         state_counter = 0
         current_f = F1_ORIGINAL
         
      elif previous_state == [0, 1]:
         state_counter += 1
            
         self.right_motor.Forward(self.right_motor_speed)
         self.left_motor.Forward(current_f * self.left_motor_speed)

         if (state_counter >= state_counter_trip) and (current_f > 0.0):
            current_f -= line_correction
            state_counter = 0

      return state_counter, current_f


   def rightTurn(self, previous_state, state_counter, state_counter_trip, line_correction, F1_ORIGINAL, current_f):

      if previous_state != [1, 0]:
         state_counter = 0
         current_f = F1_ORIGINAL
      
      elif previous_state == [1, 0]:
         state_counter += 1

         self.right_motor.Forward(current_f * self.right_motor_speed)
         self.left_motor.Forward(self.left_motor_speed)

         if (state_counter >= state_counter_trip) and (current_f > 0.0):
            current_f -= line_correction
            state_counter = 0

      return state_counter, current_f


   def leftPivot(self, f):
      self.right_motor.Forward(self.right_motor_speed)
      self.left_motor.Reverse(f * self.left_motor_speed)


   def rightPivot(self, f):
      self.right_motor.Reverse(f * self.right_motor_speed)
      self.left_motor.Forward(self.left_motor_speed)


   def stop(self):
      self.right_motor.off()
      self.left_motor.off()
               
   
   #Normal forward driving on line following
   def go_forward(self, previous_state, F1_ORIGINAL, current_f, state_counter, line_correction, state_counter_trip):
      right_val = self.sensor_right.reading()
      left_val = self.sensor_left.reading()

      if right_val==0 and left_val==0:
         state_counter = self.forward(previous_state, state_counter)
         previous_state = [0, 0]

      elif right_val==1 and left_val==0:
         state_counter, current_f = self.rightTurn(previous_state, state_counter, state_counter_trip, line_correction, F1_ORIGINAL, current_f)

         previous_state = [1, 0]

      elif right_val==0 and left_val==1:
         state_counter, current_f = self.leftTurn(previous_state, state_counter, state_counter_trip, line_correction, F1_ORIGINAL, current_f)
         previous_state = [0, 1]

      else:
         previous_state = [1, 1]
         state_counter = self.forward(previous_state, state_counter)

      return previous_state, state_counter, current_f

   
   #Turning right on a corner (CR)
   def turn_Cright(self, f):

      left_val = self.sensor_left.reading()
      while (left_val == 1):
         left_val = self.sensor_left.reading()
         self.rightPivot(f)

      while (left_val == 0):
         left_val = self.sensor_left.reading()
         self.rightPivot(f)

   #Turning left on a corner (CL)
   def turn_Cleft(self, f): #corner

      right_val = self.sensor_right.reading()
      while (right_val == 1):
         right_val = self.sensor_right.reading()
         self.leftPivot(f)

      while (right_val == 0):
         right_val = self.sensor_right.reading()
         self.leftPivot(f)


         
   #Turning right on a 'sideways T' (R)
   def turn_right(self, f):

      Tleft_val = self.sensor_Tleft.reading()
      left_val = self.sensor_left.reading()

      while (Tleft_val == 0):
         Tleft_val = self.sensor_Tleft.reading()
         self.rightPivot(f)

      while (Tleft_val == 1):
         Tleft_val = self.sensor_Tleft.reading()
         self.rightPivot(f)

      while (left_val == 1):
         left_val = self.sensor_left.reading()
         self.rightPivot(f)

      while (left_val == 0):
         left_val = self.sensor_left.reading()
         self.rightPivot(f)

   #Turning left on a 'sideways T' (L)
   def turn_left(self, f):

      Tright_val = self.sensor_Tright.reading()
      right_val = self.sensor_right.reading()
      
      while (Tright_val == 0):
         Tright_val = self.sensor_Tright.reading()
         self.leftPivot(f)

      while (Tright_val == 1):
         Tright_val = self.sensor_Tright.reading()
         self.leftPivot(f)

      while (right_val == 1):
         right_val = self.sensor_right.reading()
         self.leftPivot(f)

      while (right_val == 0):
         right_val = self.sensor_right.reading()
         self.leftPivot(f)

   #Turning right on 'head-on T' (TR)
   def turn_Tright(self, f):
      left_val = self.sensor_left.reading()
      
      while (left_val == 1):
         left_val = self.sensor_left.reading()
         self.rightPivot(f)

      while (left_val == 0):
         left_val = self.sensor_left.reading()
         self.rightPivot(f)

   #Turning left on 'head-on T' (TL)
   def turn_Tleft(self, f): 
      right_val = self.sensor_right.reading()
      
      while (right_val == 1):
         right_val = self.sensor_right.reading()
         self.leftPivot(f)

      while (right_val == 0):
         right_val = self.sensor_right.reading()
         self.leftPivot(f)

   #Continuing straight and ignoring any alternate paths (S)
   def ignore(self, previous_state, F1_ORIGINAL, current_f, state_counter, line_correction, state_counter_trip):
      Tleft_val = self.sensor_Tleft.reading()
      Tright_val = self.sensor_Tright.reading()

      while ((Tleft_val == 1) or (Tright_val == 1)):
         Tleft_val = self.sensor_Tleft.reading()
         Tright_val = self.sensor_Tright.reading()

         self.go_forward(previous_state, F1_ORIGINAL, current_f, state_counter, line_correction, state_counter_trip)

   #Go back and drive off left
   def reverse_left(self, f):
      right_val = self.sensor_right.reading()

      while (right_val == 0):
         right_val = self.sensor_right.reading()
         self.leftPivot(f)

      while (right_val == 1):
         right_val = self.sensor_right.reading()
         self.leftPivot(f)

      while (right_val == 0):
         right_val = self.sensor_right.reading()
         self.leftPivot(f)

   #Go back and drive off right
   def reverse_right(self, f):
      left_val = self.sensor_left.reading()

      while (left_val == 0):
         left_val = self.sensor_left.reading()
         self.rightPivot(f)

      while (left_val == 1):
         left_val = self.sensor_left.reading()
         self.rightPivot(f)

      while (left_val == 0):
         left_val = self.sensor_left.reading()
         self.rightPivot(f)

   #After dropping off the box, perform 180 degree turn to the left.
   def spin_left(self, f):
      state_counter = self.forward()
      sleep(0.5)

      self.reverse()
      sleep(0.5)

      right_val = self.sensor_right.reading()

      while (right_val == 1):
         right_val = self.sensor_right.reading()
         self.leftPivot(f)

      while (right_val == 0):
         right_val = self.sensor_right.reading()
         self.leftPivot(f)

   #After dropping off the box, perform 180 degree turn to the right.
   def spin_right(self, f):
      state_counter = self.forward()
      sleep(0.5)

      self.reverse()
      sleep(0.5)

      left_val = self.sensor_left.reading()

      while (left_val == 1):
         left_val = self.sensor_left.reading()
         self.rightPivot(f)

      while (left_val == 0):
         left_val = self.sensor_left.reading()
         self.rightPivot(f)

   #Picking up the box. (Takes f just to keep loop general)
   def box(self, f):
      Tleft_val = self.sensor_Tleft.reading()
      Tright_val = self.sensor_Tright.reading()

      while ((Tleft_val == 1) or (Tright_val == 1)):
         Tleft_val = self.sensor_Tleft.reading()
         Tright_val = self.sensor_Tright.reading()

         self.reverse()
   
   def get_box(self,f):
      pass
   
   def get_colour(self): #under construction
      colour_sensor_reading = self.colour_sensor.read('rgb')
      red, green, blue = colour_sensor_reading[0], colour_sensor_reading[1], colour_sensor_reading[2]

      only_colours = (red, green, blue)
      maximum_colour_reading = max(only_colours)

      #colour_sensor_reading format: (r, g, b, c)
      #Checking for green
      if (green == blue) or (abs(green - blue) == 1):
         RGB_inc = 2
      
      #Checking for blue
      elif maximum_colour_reading == blue:
         RGB_inc = 2

      #Checking for yellow
      elif maximum_colour_reading == green:
         RGB_inc = 1

      #Checking for red
      elif maximum_colour_reading == red:
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
   def start(self, previous_state, F1_ORIGINAL, current_f, state_counter, line_correction, state_counter_trip):
      Tleft_val = self.sensor_Tleft.reading()
      Tright_val = self.sensor_Tright.reading()

      while ((Tleft_val == 0) or (Tright_val == 0)):
         Tleft_val = self.sensor_Tleft.reading()
         Tright_val = self.sensor_Tright.reading()
         
         previous_state, state_counter, current_f = self.go_forward(previous_state, F1_ORIGINAL, current_f, state_counter, line_correction, state_counter_trip)

      while ((Tleft_val == 1) or (Tright_val == 1)):
         Tleft_val = self.sensor_Tleft.reading()
         Tright_val = self.sensor_Tright.reading()
         
         previous_state, state_counter, current_f = self.go_forward(previous_state, F1_ORIGINAL, current_f, state_counter, line_correction, state_counter_trip)

      return previous_state, state_counter, current_f


   #End up back in the box.
   def finish(self, previous_state, F1_ORIGINAL, current_f, state_counter, line_correction, state_counter_trip):
      Tleft_val = self.sensor_Tleft.reading()
      Tright_val = self.sensor_Tright.reading()

      while ((Tleft_val == 1) or (Tright_val == 1)):
         Tleft_val = self.sensor_Tleft.reading()
         Tright_val = self.sensor_Tright.reading()
         
         previous_state, state_counter, current_f = self.go_forward(previous_state, F1_ORIGINAL, current_f, state_counter, line_correction, state_counter_trip)

      dummy = self.forward()
      sleep(0.5)

      return previous_state, state_counter, current_f

   #Node detection function
   def detect_node(self):
      Tleft_val = self.sensor_Tleft.reading()
      Tright_val = self.sensor_Tright.reading()
      
      return ((Tright_val == 1) or (Tleft_val == 1))