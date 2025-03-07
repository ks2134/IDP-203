from machine import Pin, PWM, I2C
from time import sleep, sleep_ms
from colour_sensor import TCS34725

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
   def __init__(self, left_motor_dir, left_motor_pwm, right_motor_dir, right_motor_pwm, left_track, right_track, Tleft_track, Tright_track, led_pin, speed, i2c_sda, i2c_scl):
      
      #Setting up driving motors
      self.left_motor = Motor(left_motor_dir, left_motor_pwm)
      self.right_motor = Motor(right_motor_dir, right_motor_pwm)

      #Setting up line sensors
      self.sensor_left = TrackSensor(left_track)
      self.sensor_right = TrackSensor(right_track)
      self.sensor_Tleft = TrackSensor(Tleft_track)
      self.sensor_Tright = TrackSensor(Tright_track)

      #Setting up colour sensor:
      self.i2c_bus = I2C(0, sda=i2c_sda, scl=i2c_scl)
      self.colour_sensor = TCS34725(self.i2c_bus)

      #Setting up miscellaneous peripheries
      self.led = Pin(led_pin, Pin.OUT)

      self.left_motor_speed = speed
      self.right_motor_speed = 0.97 * speed
      

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
   
   #Colour sensor function
   def sense_colour(self):
      