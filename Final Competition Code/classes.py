from machine import Pin, PWM, I2C
from time import sleep
from colour_sensor import TCS34725


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
   * off()
      * Turn motor off
   * Forward(speed: int)
      * Drive motor in the forward direction at specified speed (0 to 100)
   * Reverse(speed: int)
      * Drive motor in reverse direction at specified speed (0 to 100)   
   """

   def __init__(self, dir_pin: int, pwm_pin: int):
      self.m1Dir = Pin(dir_pin , Pin.OUT) # set motor direction
      self.pwm1 = PWM(Pin(pwm_pin)) # set speed
      self.pwm1.freq(1000) # set max frequency
      self.pwm1.duty_u16(0) # set duty cycle
      
   def off(self):
      """Turn off the motor (sets duty cycle to 0)"""

      self.pwm1.duty_u16(0)

   def Forward(self, speed: int):
      """Drives motor in forward direction at specified speed (0 to 100)"""
      self.m1Dir.value(1) # forward = 1 reverse = 0 motor 1
      self.pwm1.duty_u16(int(65535*speed/100)) # speed range 0-100 motor 1

   def Reverse(self, speed: int):
      """Drives motor in reverse direction at specified speed (0 to 100)"""
      self.m1Dir.value(0)
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
   * reading() -> int
      * Returns the sensor reading at that instant (1 - white, 0 - black)
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
   * forward(getting_box=1, previous_state=[0, 0], state_counter=0)
      * Drive both the motors in forward direction. Checks to see if this was the last command passed, and only recommands robot if 
      this is not the case.

   * reverse()
      * Drives both motors in reverse direction.
   
   * leftTurn(previous_state:list, state_counter:int, state_counter_trip:int, line_correction:int, F1_ORIGINAL:float, current_f:float, getting_box:float)
      * Makes slight adjustment left for line following, using proportional control.

   * rightTurn(previous_state:list, state_counter:int, state_counter_trip:int, line_correction:int, F1_ORIGINAL:float, current_f:float, getting_box:float)
      * Makes slight adjustment right for line following, using proportional control.

   * leftPivot(f:float, previous_state:list)
      * Perform a major turn towards the left. For use in turning left.

   * rightPivot(f:float, previous_state:list)
      * Perform a major turn towards the right. For use in turning right.

   * stop()
      * Birnging the robot to a complete halt.

   * go_forward(previous_state:list, F1_ORIGINAL:float, current_f:float, state_counter:int, line_correction:int, state_counter_trip:int, getting_box:float)
      * Normal forward driving using proportional control line following.
   
   * turn_Cright(f:float, previous_state:list)
      * Turning right algorithm when approaching a corner on the track.

   * turn_Cleft(f:float, previous_state:list)
      * Turning left algorithm when approaching a corner on the track.

   * turn_right(f:float, previous_state:list)
      * Turning right when approaching a T junction from along side branch of T, and needing to turn right.

   * turn_left(f:float, previous_state:list)
      * Turning left when approaching a T junction from along side branch of T, and needing to turn left.

   * turn_Tright(f:float, previous_state:list)
      * Turning right from a T junction head on.
   
   * turn T_left(f:float, previous_state:list)
      * Turning left from a T junction head on.
   
   * ignore(previous_state:list, F1_ORIGINAL:float, current_f:float, state_counter:int, line_correction:int, state_counter_trip:int, getting_box:float)
      * Operation performed on "S" node, when the Vehicle must continue straight and ignore a possible turn.

   * reverse_left(f:float, previous_state:list)
      * Reverse away from the box after collection, and turn to face left.
   
   * reverse_right(f:float, previous_state:list)
      * Reverse away from the box after colelction, and turn to face right.

   * spin_left(f:float, previous_state:list)
      * Perform 180 degree rotation to the left after dropping off box.

   * spin_right(f:float, previous_state:list)
      * Perform 180 degree rotation to the right after dropping off box.

   * box_right(f:float, previous_state:list)
      * Turn to face a box node approaching from the right hand side.

   * box_left(f:float, previous_state:list)
      * Turn to face a box node approaching from the left hand side.

   * get_box()
      * Pick up the box, and return increment based on the colour of the box, to calculate next node in the tree.

   * get_colour()
      * Uses the colour sensor to identify the colour of the box which has just been picked up. Return colour index to calculate next node 
      in the tree

   * start(previous_state:list, F1_ORIGINAL:float, current_f:float, state_counter:int, line_correction:int, state_counter_trip:int, getting_box:float)
      * Initial function to leave starting zone and onto main track.
   
   * finish(previous_state:list, F1_ORIGINAL:float, current_f:float, state_counter:int, line_correction:int, state_counter_trip:int, getting_box:float)
      * Come off the track, and end in the starting zone.
      
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
      self.servo.freq(50)                 #PWM Frequency set at 50Hz
      
   
   def forward(self, getting_box=1, previous_state: list = [0, 0], state_counter=0):
      """Drives the robot forwards. Only calls motor forward functions if the previous command was different. Returns state_counter
      which is only changed if the previous instruction was different."""
      
      if previous_state != [0, 0]:
         state_counter = 0
         
         self.left_motor.Forward(self.left_motor_speed * getting_box)
         self.right_motor.Forward(self.right_motor_speed * getting_box)
      
      return state_counter
      
   
   def reverse(self):
      """Drives both motors in reverse, to send Vehicle backwards."""

      self.left_motor.Reverse(self.left_motor_speed)
      self.right_motor.Reverse(self.right_motor_speed)

   
   def leftTurn(self, previous_state:list, state_counter:int, state_counter_trip:int, line_correction:int,
                 F1_ORIGINAL:float, current_f:float, getting_box:float):
      """Makes a slight correction to the left to keep robot on line when line following. Uses proportional control to increase scale of 
      correction if vehicle is off the line for longer, by increasing the ratio of speeds for each wheel. Returns state_counter and
      current_f."""

      if previous_state != [0, 1]:
         state_counter = 0
         current_f = F1_ORIGINAL
         
      elif previous_state == [0, 1]:
         state_counter += 1
            
         self.right_motor.Forward((self.right_motor_speed) * getting_box)                #Potentially prevent repeated calling if not needed?
         self.left_motor.Forward(current_f * (self.left_motor_speed) * getting_box)

         if (state_counter >= state_counter_trip) and (current_f > 0.0):
            current_f -= line_correction
            state_counter = 0

      return state_counter, current_f

   
   def rightTurn(self, previous_state:list, state_counter:int, state_counter_trip:int, line_correction:int,
                  F1_ORIGINAL:float, current_f:float, getting_box:float):
      """Makes a slight correction to the right to keep robot on line when line following. Uses proportional control to increase scale of 
      correction if vehicle is off the line for longer, by increasing the ratio of speeds for each wheel. Returns state_counter and
      current_f."""

      if previous_state != [1, 0]:
         state_counter = 0
         current_f = F1_ORIGINAL
      
      elif previous_state == [1, 0]:
         state_counter += 1

         self.right_motor.Forward(current_f * (self.right_motor_speed) * getting_box)        #Same as in previous. Maybe change?
         self.left_motor.Forward((self.left_motor_speed) * getting_box)

         if (state_counter >= state_counter_trip) and (current_f > 0.0):
            current_f -= line_correction
            state_counter = 0

      return state_counter, current_f

   
   def leftPivot(self, f:float, previous_state:list):
      """Perform a major turn towards the left. For use in major turns left. Returns previous state=[1, 1]"""

      self.right_motor.Forward(self.right_motor_speed)
      self.left_motor.Reverse(f * self.left_motor_speed)

      previous_state = [1, 1]
      return previous_state

   
   def rightPivot(self, f:float, previous_state:list):
      """Perform a major turn towards the right. For use in major turns right. Returns previous state=[1, 1]"""

      self.right_motor.Reverse(f * self.right_motor_speed)
      self.left_motor.Forward(self.left_motor_speed)

      previous_state = [1, 1]
      return previous_state

   
   def stop(self):
      """Stop both motors and bring Vehicle to complete halt."""
      self.right_motor.off()
      self.left_motor.off()
               
   

   def go_forward(self, previous_state:list, F1_ORIGINAL:float, current_f:float, state_counter:int, line_correction:int,
                   state_counter_trip:int, getting_box:float):
      """Normal driving forward algorithm. Includes function calls allow for line following. Returns previous_state, state_counter and current_f
      which may change through line following control."""
      
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
   
   
   def turn_Cright(self, f:float, previous_state:list):
      """Method for right turn on a corner node. Corresponds to 'CR' node in tree. Returns previous state, which is changed by rightPivot method, which is internally called."""

      sleep(0.1)
      left_val = self.sensor_left.reading()
      while (left_val == 1):
         left_val = self.sensor_left.reading()
         previous_state = self.rightPivot(f, previous_state)

      while (left_val == 0):
         left_val = self.sensor_left.reading()
         previous_state = self.rightPivot(f, previous_state)

      return previous_state

   
   def turn_Cleft(self, f:float, previous_state:list):
      """Method for left turn on a corner node. Corresponds to 'CL' node in tree. Returns previous state, which is changed by leftPivot method, which is internally called."""

      sleep(0.1)
      right_val = self.sensor_right.reading()
      while (right_val == 1):
         right_val = self.sensor_right.reading()
         previous_state = self.leftPivot(f, previous_state)

      while (right_val == 0):
         right_val = self.sensor_right.reading()
         previous_state = self.leftPivot(f, previous_state)

      return previous_state
         
   
   def turn_right(self, f:float, previous_state:list):
      """Method to turn right when approaching a T junction 'from the side'. Corresponds to 'R' node in tree. Returns previous_state."""

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

   
   def turn_left(self, f:float, previous_state:list):
      """Method to turn left when approaching a T junction 'from the side'. Corresponds to 'L' node in tree. Returns previous_state."""

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

   
   def turn_Tright(self, f:float, previous_state:list):
      """Method to turn right when approaching a T junction 'head on'. Corresponds to 'TR' node in tree. Returns previous_state."""

      left_val = self.sensor_left.reading()
      
      while (left_val == 1):
         left_val = self.sensor_left.reading()
         previous_state = self.rightPivot(f, previous_state)

      while (left_val == 0):
         left_val = self.sensor_left.reading()
         previous_state = self.rightPivot(f, previous_state)

      return previous_state

   
   def turn_Tleft(self, f:float, previous_state:list):
      """Method to turn left when approaching a T junction 'head on'. Corresponds to 'TL' node in tree. Returns previous_state."""
      
      right_val = self.sensor_right.reading()
      
      while (right_val == 1):
         right_val = self.sensor_right.reading()
         previous_state = self.leftPivot(f, previous_state)

      while (right_val == 0):
         right_val = self.sensor_right.reading()
         previous_state = self.leftPivot(f, previous_state)

      return previous_state



   def ignore(self, previous_state:list, F1_ORIGINAL:float, current_f:float, state_counter:int, line_correction:int,
              state_counter_trip:int, getting_box:float):
      """Method to continue straight and ignore any possible branches on route. Corresponds to 'S' node in tree."""
      
      Tleft_val = self.sensor_Tleft.reading()
      Tright_val = self.sensor_Tright.reading()

      while ((Tleft_val == 1) or (Tright_val == 1)):
         Tleft_val = self.sensor_Tleft.reading()
         Tright_val = self.sensor_Tright.reading()

         self.go_forward(previous_state, F1_ORIGINAL, current_f, state_counter, line_correction, state_counter_trip, getting_box)

   
   def reverse_left(self, f:float, previous_state:list):
      """Reverse away from the box after collecting it, and turn to face left upon reaching the previous node. Corresponds to 'RL' node in tree. 
      Returns previous_state"""

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

   
   def reverse_right(self, f:float, previous_state:list):
      """Reverse away from the box after collecting it, and turn to face left upon reaching the previous node. Corresponds to 'RR' node in tree. 
      Returns previous_state"""

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

   
   def spin_left(self, f:float, previous_state:list):
      """Reverse away from the depot after dropping off a box, and turn 180 degrees to the left after leaving the depot.
      Corresponds to 'SL' node in tree. Returns previous_state."""

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

   
   def spin_right(self, f:float, previous_state:list):
      """Reverse away from the depot after dropping off a box, and turn 180 degrees to the right after leaving the depot.
      Corresponds to 'SR' node in tree. Returns previous_state."""

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

   
   def box_right(self, f:float, previous_state:list):
      """Method to turn in to face a box, when approaching a box collection from the right hand side. Corresponds to node 'BR' on the tree.
      Returns previous_state."""

      self.forward(previous_state)
      sleep(0.32)

      previous_state = self.rightPivot(f, previous_state)
      sleep(0.75)

      return previous_state
   
   def box_right_1(self, f:float, previous_state:list):
      """Method to turn in to face box closed to home zone, when approaching collection from the right hand side.
      Corresponds to node 'BR' on the tree. Returns previous_state."""

      self.forward(previous_state)
      sleep(0.37)

      previous_state = self.rightPivot(f, previous_state)
      sleep(0.75)

      return previous_state


   def box_left(self, f:float, previous_state:list):
      """Method to turn in to face a box, when approaching a box collection from the right hand side. Corresponds to node 'BL' on the tree.
      Returns previous_state."""
      
      self.forward(previous_state)
      sleep(0.28)

      previous_state = self.leftPivot(f, previous_state)
      sleep(0.7)

      return previous_state


   
   def get_box(self) -> int:
      """Method to pick up box, and reverse out of the collection zone towards the previous node. Returns colour increment for calculation
      of next branch of the tree. Corresponds to 'B' node in the tree."""

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
   
   
   def get_colour(self) -> int:
      """Calcualtes the colour of the box currently being picked up. Takes 5 readings using the colour sensor and takes an average, and then
      decides the colour value based off of this, and returns RGB_inc to calculate the next branch of the tree.
      """
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

      #colour_sensor_reading format: ,(r, g b, c)

      print(avg_colour_values)

      #Edge case where all 3 colour readings are the same.
      if (avg_green == avg_blue == avg_red):
         RGB_inc = 2
      
      #Checking for green
      elif (((avg_green == avg_blue) or (abs(avg_blue - avg_green) == 1)) and (avg_red != maximum_colour_reading)):
         RGB_inc = 2

      #Checking for red
      elif maximum_colour_reading == avg_red:
         RGB_inc = 1

      #Checking for blue
      elif maximum_colour_reading == avg_blue:
         RGB_inc = 2

      #Checking for yellow
      elif maximum_colour_reading == avg_green:
         RGB_inc = 1

      print(RGB_inc)
      return RGB_inc

   
   def start(self, previous_state:list, F1_ORIGINAL:float, current_f:float, state_counter:int, line_correction:int,
              state_counter_trip:int, getting_box:float):
      """Method to move out of the starting box, and onto the first node of the track. Returns previous_state, state_counter, current_f"""
      
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
   
   
   def finish(self, previous_state:list, F1_ORIGINAL:float, current_f:float, state_counter:int, line_correction:int,
               state_counter_trip:int, getting_box:float):
      """Method to move back into the starting box, off the main track. Returns previous_state, state_counter and current_f."""
      
      Tleft_val = self.sensor_Tleft.reading()
      Tright_val = self.sensor_Tright.reading()

      while ((Tleft_val == 1) or (Tright_val == 1)):
         Tleft_val = self.sensor_Tleft.reading()
         Tright_val = self.sensor_Tright.reading()
         
         previous_state, state_counter, current_f = self.go_forward(previous_state, F1_ORIGINAL, current_f, state_counter, line_correction, state_counter_trip, getting_box)

      dummy = self.forward()
      sleep(0.7)

      return previous_state, state_counter, current_f


   
   def detect_node(self):
      """Method to see if the next node has been reached."""

      Tleft_val = self.sensor_Tleft.reading()
      Tright_val = self.sensor_Tright.reading()
      
      return ((Tright_val == 1) or (Tleft_val == 1))