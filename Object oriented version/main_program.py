from time import sleep
from classes import *

#Constants and definitions:
LEFT_MOTOR_DIR_PIN = 7
LEFT_MOTOR_PWM_PIN = 6

LEFT_MOTOR_SPEED = 100
RIGHT_MOTOR_SPEED = 97

RIGHT_MOTOR_DIR_PIN = 4
RIGHT_MOTOR_PWM_PIN = 5

OUTER_LEFT_TRACK_SENSOR_PIN = 26
LEFT_TRACK_SENSOR_PIN = 20
RIGHT_TRACK_SENSOR_PIN = 11
OUTER_RIGHT_TRACK_SENSOR_PIN = 8

LED_PIN = 9

F1_ORIGINAL = 0.7
F2_ORIGINAL = 0.5
F3_ORIGINAL = 1.0

LINE_CORRECTION = 0.2
STATE_COUNTER_TRIP = 500

SPEED = 100


robot = Vehicle(LEFT_MOTOR_DIR_PIN, LEFT_MOTOR_PWM_PIN, 
                RIGHT_MOTOR_DIR_PIN, RIGHT_MOTOR_PWM_PIN, 
                LEFT_TRACK_SENSOR_PIN, RIGHT_TRACK_SENSOR_PIN,
                OUTER_LEFT_TRACK_SENSOR_PIN, OUTER_RIGHT_TRACK_SENSOR_PIN,
                LED_PIN, SPEED)


previous_state = [0, 0]
state_counter = 0
current_f = F1_ORIGINAL



node_types = {"L":robot.turn_left, "R":robot.turn_right, "S":robot.ignore, "CR":robot.turn_Cright, "CL":robot.turn_Cleft,
              "TR":robot.turn_Tright,"TL":robot.turn_Tleft, "RR":robot.reverse_right, "RL":robot.reverse_left, 
              "SR":robot.spin_right, "SL":robot.spin_left, "B":robot.box,"FIN":robot.finish}

#S for straight, CR CL for corners, TL TR for 'head on' t, L and R for side t
test_route = ["TL","R", "B", "RL", "TL", "SL", "S", "R", "S", "R", "B", "RL", "TR", "S", "SR", "S", "S", "CL", "L", "B", "RR", "S", "CL", "S", "S", "SL", "S", "R", "L", "L", "B", "RR", "TR", "S", "CR", "S", "S","SR", "L", "L", "FIN"]

robot.led.value(1)

cur = -1
directions = [robot.go_forward, robot.reverse]

previous_state, state_counter, current_f = robot.start(previous_state, F1_ORIGINAL, current_f, state_counter, LINE_CORRECTION, STATE_COUNTER_TRIP)

while cur < (len(test_route) - 1):
    next_node = test_route[cur + 1]
    node = robot.detect_node()

    if (next_node == "RR") or (next_node == "RL"):
        cur_dir = 1 
    else:
        cur_dir = 0

    if node == False:
        try:
            previous_state, state_counter, current_f = directions[cur_dir](previous_state, F1_ORIGINAL, current_f, state_counter, LINE_CORRECTION, STATE_COUNTER_TRIP)
        except:
            robot.reverse()
    
    elif node == True:
        if (test_route[cur + 1] == "S") or (test_route[cur + 1] == "FIN"):
            previous_state = node_types[next_node](previous_state, F1_ORIGINAL, current_f, state_counter, LINE_CORRECTION, STATE_COUNTER_TRIP)

        elif (test_route[cur + 1] == "SL") or (test_route[cur + 1] == "SR"):
            node_types[next_node](F3_ORIGINAL)

        else:
            node_types[next_node](F2_ORIGINAL)
        
        cur += 1


robot.stop()