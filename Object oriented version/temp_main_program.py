from classes import *
from route_tree import *

#----------------------------Constants and definitions:----------------------------#
#Motor pins.
LEFT_MOTOR_DIR_PIN = 7
LEFT_MOTOR_PWM_PIN = 6

RIGHT_MOTOR_DIR_PIN = 4
RIGHT_MOTOR_PWM_PIN = 5

#Individual speeds for balancing.
LEFT_MOTOR_SPEED = 100
RIGHT_MOTOR_SPEED = 98

#Line following sensor pins.
OUTER_LEFT_TRACK_SENSOR_PIN = 26
LEFT_TRACK_SENSOR_PIN = 20
RIGHT_TRACK_SENSOR_PIN = 11
OUTER_RIGHT_TRACK_SENSOR_PIN = 8

#Pin definition for LED and Start/Stop button.
LED_PIN = 9
BUTTON_PIN = 12

#Motor speed ratios.
F1_ORIGINAL = 0.7
F2_ORIGINAL = 0.5
F3_ORIGINAL = 1.0

#Line following proportional control parameters.
LINE_CORRECTION = 0.2
STATE_COUNTER_TRIP = 500 

#Overall robot speed setting, and box collection speed setting.
SPEED = 100
BOX_SPEED_COEFFICIENT = 1

#Colour sensor I2C bus setup.
I2C0_SDA = 16
I2C0_SCL = 17
I2C0_BUS_NO = 0

#Servo postition parameters, and pin setup.
SERVO_0 = 1850
SERVO_10 = 1400 
SERVO_PIN = 15

#Time of Flight sensor I2C bus setup (there if needed).
I2C1_SDA = 18
I2C1_SCL = 19
I2C1_BUS_NO = 1


robot = Vehicle(LEFT_MOTOR_DIR_PIN, LEFT_MOTOR_PWM_PIN, 
                RIGHT_MOTOR_DIR_PIN, RIGHT_MOTOR_PWM_PIN, 
                LEFT_TRACK_SENSOR_PIN, RIGHT_TRACK_SENSOR_PIN,
                OUTER_LEFT_TRACK_SENSOR_PIN, OUTER_RIGHT_TRACK_SENSOR_PIN,
                LED_PIN, SPEED, I2C0_SDA, I2C0_SCL, I2C0_BUS_NO,
                SERVO_10, SERVO_0, SERVO_PIN, BUTTON_PIN)


previous_state = [1, 1]
state_counter = 0
current_f = F1_ORIGINAL
button_stop = False


while robot.button.value() != 1:
    pass


#--------------------------------------------------------Main loop:--------------------------------------------------------#


node_types = {"L":robot.turn_left, "R":robot.turn_right, "S":robot.ignore, "CR":robot.turn_Cright, "CL":robot.turn_Cleft,
              "TR":robot.turn_Tright,"TL":robot.turn_Tleft, "RR":robot.reverse_right, "RL":robot.reverse_left, 
              "SR":robot.spin_right, "SL":robot.spin_left, "B":robot.get_box,"FIN":robot.finish, "BR":robot.box_right,
              "BL":robot.box_left}

test_route = tree[0]

box_num = 0 #represents number of delivered boxes - 1
box_inc = 0 #0 for collecting, 1 for delivering
RGB_inc = 1 #1 for YR, 2 for GB

robot.led.value(1)
directions = [robot.go_forward, robot.reverse]

robot.servo.duty_u16(SERVO_0)
previous_state, state_counter, current_f = robot.start(previous_state, F1_ORIGINAL, current_f, state_counter, LINE_CORRECTION, STATE_COUNTER_TRIP, 1)
while (box_num < 9):
    cur = -1
    while cur < (len(test_route) - 1):
        current_button_value = robot.button.value()
        #print(current_button_value)
        
        if current_button_value == 1:
            button_stop = True
            break

        next_node = test_route[cur + 1]
        node = robot.detect_node()

        if (next_node == "RR") or (next_node == "RL"):
            cur_dir = 1 
        else:
            cur_dir = 0

        if node == False:
            if (test_route[cur + 1] == "B"):
                try:
                    previous_state, state_counter, current_f = directions[cur_dir](previous_state, F1_ORIGINAL, current_f, state_counter, 0, STATE_COUNTER_TRIP, 1)
                except:
                    robot.reverse()
            else:
                try:
                    previous_state, state_counter, current_f = directions[cur_dir](previous_state, F1_ORIGINAL, current_f, state_counter, LINE_CORRECTION, STATE_COUNTER_TRIP, 1)
                except:
                    robot.reverse()
        
        elif node == True:
            #print(next_node)
            if (test_route[cur + 1] == "S") or (test_route[cur + 1] == "FIN"):
                previous_state = node_types[next_node](previous_state, F1_ORIGINAL, current_f, state_counter, LINE_CORRECTION, STATE_COUNTER_TRIP, 1)

            elif (test_route[cur + 1] == "SL") or (test_route[cur + 1] == "SR"):
                previous_state = node_types[next_node](F3_ORIGINAL, previous_state)
            
            elif (test_route[cur + 1] == "BL") or (test_route[cur + 1] == "BR"):
                   previous_state = node_types[next_node](F3_ORIGINAL, previous_state)

            #elif ((test_route[cur + 2] == "B") and (cur < (len(test_route) - 1))):
                #   node_types[next_node](F3_ORIGINAL)
            elif (test_route[cur + 1] == "B"):
                RGB_inc = node_types[next_node]()

            else:
                previous_state = node_types[next_node](F2_ORIGINAL, previous_state)
                
            cur += 1

    if button_stop == True:
        break
    else:
            
        if (box_inc == 0): #fetching box
            if (box_num == 8):
                break
            #RGB_inc = robot.get_box()
            #print(RGB_inc) 
            test_route = tree[4 * box_num + 2 * box_inc + RGB_inc]
        elif (box_inc == 1): #dropping box or returning to home square
            test_route = tree[4 * box_num + 2 * box_inc + RGB_inc]
            box_num += 1
        box_inc = abs(box_inc - 1)


robot.led.value(0)
robot.stop()
