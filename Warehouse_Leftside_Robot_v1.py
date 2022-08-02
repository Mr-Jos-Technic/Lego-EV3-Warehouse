#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, Image, ImageFile, Font
from pybricks.messaging import BluetoothMailboxServer, BluetoothMailboxClient, LogicMailbox, NumericMailbox, TextMailbox
from threading import Thread
from random import choice
from math import fmod
import sys
import os
import math
import struct

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.
# MIT License: Copyright (c) 2022 Mr Jos

#####################################################################
#####################################################################
##########~~~~~PROGRAM WRITTEN BY JOZUA VAN RAVENHORST~~~~~##########
##########~~~~~~~~~~~WAREHOUSE XL: LEFTSIDE 6DOF~~~~~~~~~~~##########
##########~~~~~~~~~~~~~YOUTUBE CHANNEL: MR JOS~~~~~~~~~~~~~##########
#####################################################################
#####################################################################


##########~~~~~~~~~~HARDWARE CONFIGURATION~~~~~~~~~~##########
ev3 = EV3Brick()
#   Motors definition
pitch_base          =   Motor(Port.A)                                                       #Motor for joint 2 (theta2) (Pitching the lower arm)
pitch_arm           =   Motor(Port.B)                                                       #Motor for joint 3 (theta3) (Pitching the upper arm)
roll_arm            =   Motor(Port.C)                                                       #Motor for joint 4 (theta4) (Rolling the wrist)
yaw_arm             =   Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)        #Motor for joint 5 (theta5) (Yawing the wrist)
#   Sensor definition
touch_pitch_base    =   TouchSensor(Port.S1)                                                #Touch sensor at back of the base for homing: theta2
touch_pitch_arm     =   TouchSensor(Port.S2)                                                #Touch sensor at the side of arm for homing:  theta3
touch_roll_arm      =   TouchSensor(Port.S3)                                                #Touch sensor at the end of arm for homing:   theta4
#                                                                                           #Stall detection used for for homing:         theta5

th2_switch          =      -1                                                               #Switching the direction of the motor for theta2 (1/-1) (Used when changing gearing for your robot)
th3_switch          =      -1                                                               #Switching the direction of the motor for theta3 (1/-1) (Used when changing gearing for your robot)


##########~~~~~~~~~~HOMING POSITION ANGLES WHEN SENSOR ACTIVATED~~~~~~~~~~##########
yaw_base_zeroing    =     -50                                                               #  -35   6/05/2021  Will be send by BT                                [  -25@version2]
pitch_base_zeroing  =   -2060                                                               # -850  31/01/2021  -1930 on 6/05/2021  -1880 on 8/05/2021            [-1860@version2]
pitch_arm_zeroing   =    1600                                                               # -255 originally, -225 on 19/01/2021 530 on 5/05/2021                [  550@Version2]
roll_arm_zeroing    =     625                                                               #  855  31/01/2021                                                    [ 1475@version2]
yaw_arm_zeroing     =   -2240                                                               #-1120  31/01/2021                                                    [-1040@version2]
roll_head_zeroing   =    -870                                                               #   50   6/05/2021  Will be send by BT   -60                          [  -30@version2]

roll_head_zeroing_pitch_base_angle = 25                                                     #Angle in degrees to tip forward to reach the color sensor with the head [33@version2]
roll_head_zeroing_pitch_arm_angle  = 78                                                     #Angle in degrees to tip forward to reach the color sensor with the head [77@Version2]


##########~~~~~~~~~~LENGTH FOR ALL COMPONENTS IN 'mm'~~~~~~~~~~##########
a1 =  160                                                                                   #Height from floor to center pitch_base [Z]                                                                         [175@version1]  [165@version2]
a2 =  295                                                                                   #Height from pitch_base to pitch_arm center [Z]                                                                     [145@version1]  [168@version2]
a3 =   40                                                                                   #Height from pitch_arm center to the center of the arm [Z]                                                          [ 47@version1]  [ 47@version2]
a45 = 280                                                                                   #Length from pitch_arm center to yaw_arm center (Center of wrist!) [X]                                              [152@version1]  [152@version2]
a67 =  70                                                                                   #Length from yaw_arm center (Center of wrist!) to front head center + desired distance in front of the head. [X]    [115@version1]  [115@version2]
a345 = math.sqrt((a3 ** 2) + (a45 ** 2))                                                    #Diagonal length from pitch_arm center to center of wrist
phi345 = math.degrees(math.atan(a45 / a3))                                                  #Angle for a345 to a3


##########~~~~~~~~~~GEARING~~~~~~~~~~##########
yaw_base_full_rot   =  4200                                                                 #theta1   360 /  12 *140  /  12 * 12                                      = 1/11.66666                           [ 4200@version2]
pitch_base_full_rot = 23625                                                                 #theta2   360 /  12 * 60  /   8 * 28  /   8 * 24  /  16 * 20              = 1/65.625                             [15120@version2]
pitch_arm_full_rot  = 18900                                                                 #theta3   360 /  12 * 60  /   8 * 28  /   8 * 24                          = 1/52.5                               [10500@version2]
roll_arm_full_rot   =  3000                                                                 #theta4   360 /  12 * 60  /  16 * 16  /  12 * 20  /  16 * 16              = 1/5            ####1800 = 1/8.333333 [ 3000@version2]
yaw_arm_full_rot    =  7200                                                                 #theta5   360 /  12 * 60  /  12 * 12  /  16 * 16  /  12 * 24  /  12 * 24  = 1/20                                 [ 3600@version2]   2880 V3 20-16 gear 1/8
roll_head_full_rot  =  2520                                                                 #theta6   360 /   8 * 56  /  12 * 12  /   4 * 4                           = 1/7                                  [ 2520@version2]

yaw_base_gear   = yaw_base_full_rot   / 360                                                 #Amount of rotations the motor needs to do for a 360degree rotation of the joint 1
pitch_base_gear = pitch_base_full_rot / 360                                                 #Amount of rotations the motor needs to do for a 360degree rotation of the joint 2
pitch_arm_gear  = pitch_arm_full_rot  / 360                                                 #Amount of rotations the motor needs to do for a 360degree rotation of the joint 3
roll_arm_gear   = roll_arm_full_rot   / 360                                                 #Amount of rotations the motor needs to do for a 360degree rotation of the joint 4
yaw_arm_gear    = yaw_arm_full_rot    / 360                                                 #Amount of rotations the motor needs to do for a 360degree rotation of the joint 5
roll_head_gear  = roll_head_full_rot  / 360                                                 #Amount of rotations the motor needs to do for a 360degree rotation of the joint 6


##########~~~~~~~~~~MAXIMUM SPEED, MAXIMUM ACCELERATION, MAXIMUM POWER~~~~~~~~~~##########
pitch_base.control.limits(900, 3600, 100)                                                   #Motor settings [Max speed allowed (deg/sec) / Acceleration (deg/sec²) / Power (%)]
pitch_arm.control.limits( 900, 3600, 100)                                                   #Motor settings [Max speed allowed (deg/sec) / Acceleration (deg/sec²) / Power (%)]
roll_arm.control.limits( 1200, 3600, 100)                                                   #Motor settings [Max speed allowed (deg/sec) / Acceleration (deg/sec²) / Power (%)]
yaw_arm.control.limits(  1200, 3600, 100)                                                   #Motor settings [Max speed allowed (deg/sec) / Acceleration (deg/sec²) / Power (%)]
max_speed = 1200                                                                            #Override for Maximum speed for all joints. (800)
step = 7                                                                                    #Distance in mm for each step in Inverse Kinematic mode, lower = more accurate but might start shaking due to slow calculations     !!!5!!!


##########~~~~~~~~~~MAXIMUM ACCELERATION AND MAXIMUM ANGLE TO SAY A MOVEMENT IS FINISHED~~~~~~~~~~##########
pitch_base.control.target_tolerances(1000, 10)                                              #Allowed deviation from the target before motion is considered complete. (degrees/second, degrees) (1000, 10)
pitch_arm.control.target_tolerances( 1000, 10)                                              #Allowed deviation from the target before motion is considered complete. (degrees/second, degrees) (1000, 10)
roll_arm.control.target_tolerances(  1200, 10)                                              #Allowed deviation from the target before motion is considered complete. (degrees/second, degrees) (1000, 10)
yaw_arm.control.target_tolerances(   1200, 10)                                              #Allowed deviation from the target before motion is considered complete. (degrees/second, degrees) (1000, 10)


##########~~~~~~~~~~BLUETOOTH SETUP, SERVER SIDE~~~~~~~~~~##########
server                  = BluetoothMailboxServer()                                          #Defining the name for this master (Server) EV3 brick for sending/receiving bluetooth commands
commands_bt_text        = TextMailbox   ('commands text'        , server)                   #Main mailbox for sending commands and receiving feedback to other slave robot brick
yaw_base_bt_zeroing     = NumericMailbox('zero position yaw'    , server)                   #Mailbox for sending theta1 homing position
yaw_base_bt_num         = NumericMailbox('yaw base degree'      , server)                   #Mailbox for sending theta1 position
yaw_base_bt_sp          = NumericMailbox('yaw base speed'       , server)                   #Mailbox for sending theta1 speed
yaw_base_feedb          = NumericMailbox('yaw base feedback'    , server)                   #Mailbox with feedback from current theta1 angle
roll_head_bt_zeroing    = NumericMailbox('zero position roll'   , server)                   #Mailbox for sending theta6 homing position
roll_head_bt_num        = NumericMailbox('roll head degree'     , server)                   #Mailbox for sending theta6 position
roll_head_bt_sp         = NumericMailbox('roll head speed'      , server)                   #Mailbox for sending theta6 speed
roll_head_feedb         = NumericMailbox('roll head feedback'   , server)                   #Mailbox with feedback from current theta6 angle

feedback_commands_bt    = TextMailbox   ('feedback from 6dof'   , server)                   #Mailbox that is being forwarded by the left side robot EV3 brick, from the Master Conveyor EV3 brick and returns


##########~~~~~~~~~~CREATING AND STARTING A TIMER, FOR INVERSE KINEMATIC SMOOTH CONTROL~~~~~~~~~~##########
timer_movement = StopWatch()                                                                #Creating the timer
#timer_movement.time()                                                                      #Reading  the timer's current value (ms) [Not used, example]
#timer_movement.pause()                                                                     #Stopping the timer [Not used, example]
#timer_movement.resume()                                                                    #Resuming the timer [Not used, example]
timer_movement.reset()                                                                      #Putting  the timer back at 0, if not stopped it will just keep running but start from 0 again.


##########~~~~~~~~~~BUILDING GLOBAL VARIABLES~~~~~~~~~~##########
move_coor_list      =   [0, 0, 0, 0, 0, 0]
move_angle_list     =   [0, 0, 0, 0, 0, 0]
speed_list          =   [0, 0, 0, 0, 0, 0]
old_coor_list       =   [0, 0, 0, 0, 0, 0]
old_orientation     =   [0, 0, 0]
next_orientation    =   [0, 0, 0]
max_angle           =   0
theta1_rot          =   0
theta4_rot          =   0
theta6_rot          =   0
old_speed           =   int(max_speed)
stepspeed           =   70
overlaptime         =   200
emergency_stop      =   False
job                 =   0
scissor_up          =   False
esp_adj             =   [0, 0, 0, 0, 0, 0]
program_start       =   True
old_positions       =   [0, 0, 0]
actuation_list      =   []


##########~~~~~~~~~~BRICK STARTUP SETTINGS~~~~~~~~~~##########
ev3.speaker.set_volume          (volume=90, which='_all_')                                  #Set the volume for all sounds (speaking and beeps etc)
ev3.speaker.set_speech_options  (language='en', voice='m7', speed=None, pitch=None)         #Select speaking language, and a voice (male/female)
small_font  = Font(size=6)                                                                  #Creating a font with  6 pixel height for text on screen    [Not used]
normal_font = Font(size=10)                                                                 #Creating a font with 10 pixel height for text on screen    [Not used]
big_font    = Font(size=16)                                                                 #Creating a font with 16 pixel height for text on screen
ev3.screen.set_font(big_font)                                                               #Choose a preset font for writing next texts
ev3.screen.clear()                                                                          #Make the screen empty (all pixels white)
ev3.speaker.beep()                                                                          #Brick will make a beep sound 1 time
#ev3.light.on(Color.GREEN)                                                                  #Turns the green lights on the brick on                     [Not used, example]
ev3.light.off()                                                                             #Turn the lights off on the brick
#ev3.screen.draw_text(4, 2, "", text_color=Color.BLACK, background_color=Color.WHITE)       #X/Y position for writing on the screen, textstring, text color, background color   [Example]


##########~~~~~~~~~~CREATING A FILE THAT IS SAVED OFFLINE~~~~~~~~~~##########
#Not used right now, it can be used to store coordinates of positions to go to, and call them back when restarting the program
#os.remove("saveddata.txt")                                                                 #Removing the wms file for whatever reason you might need to delete it  [KEEP # for normal operation!]
#create_file = open("saveddata.txt", "a")                                                   #Create a file if it does not exist and open it, if it does exist just open it
#create_file.write("")                                                                      #Write "Nothing" to the file to have atleast 1 line in the file
#create_file.close()                                                                        #Close the .txt file


##########~~~~~~~~~~DEFINE SUB-ROUTINES [FUNCTIONS] FOR THE OPERATION OF THE ROBOT~~~~~~~~~~##########
def check_emergency_stop():                                                                 #This function checks if there is a need to stop the next movement
    global emergency_stop                                                                   #Using this global variable in this local function (if not defined to be global, it will make a local variable)

    if emergency_stop == True:                                                              #If there is an emergency state
        motor_braking()                                                                     #Call the function that will brake the arm motors
        while emergency_stop == True: wait(250)                                             #If there is still an emergency stop, start this loop until the emergency stop has been reset


def manual_esp_homing():                                                                    #This function controls the manual input from the ESP32 touchscreen to make the robot move before homing
    homing_yaw_base_cur_angle = 0                                                           #Defining a local variable
    
    yaw_base_bt_sp.send(200)                                                                #The message is send by bluetooth communication to the slave robot brick, with the speed for joint 1
    while True:                                                                             #Start a forever loop
        new_task = feedback_commands_bt.wait_new()                                          #Wait for a new command to be received by bluetooth
        print(new_task)                                                                     #Print this feedback line when debugging
        if   new_task == "Stop motors":                                                     #Compare the received message
            pitch_base.hold()                                                               #Lock joint 2 in position
            pitch_arm.hold()                                                                #Lock joint 3 in position
        elif new_task == "Start homing":                                                    #Compare the received message
            pitch_base.hold()                                                               #Lock joint 2 in position
            pitch_arm.hold()                                                                #Lock joint 3 in position
            break                                                                           #Break out of the while loop
        elif new_task == "J1 CW":                                                           #Compare the received message
            while True:                                                                     #Start a forever loop
                homing_yaw_base_cur_angle -= 10                                             #Add -10 to the local variable
                yaw_base_bt_num.send(homing_yaw_base_cur_angle)                             #Send the angle by bluetooth to move joint 1
                wait(50)                                                                    #Wait 50ms (20Hz, running 10 degrees each pulse at 200°/s makes it smooth)
                if feedback_commands_bt.read() == "Stop motors": break                      #If the motor has to stop moving, break out of this loop
        elif new_task == "J1 CCW":                                                          #Compare the received message
            while True:                                                                     #Start a forever loop
                homing_yaw_base_cur_angle += 10                                             #Add 10 to the local variable
                yaw_base_bt_num.send(homing_yaw_base_cur_angle)                             #Send the angle by bluetooth to move joint 1
                wait(50)                                                                    #Wait 50ms (20Hz, running 10 degrees each pulse at 200°/s makes it smooth)
                if feedback_commands_bt.read() == "Stop motors": break                      #If the motor has to stop moving, break out of this loop
        elif new_task == "J2 CW":  pitch_base.run(600 * th2_switch)                         #Compare the received message, start the motor for joint 2 clockwise at a constant speed
        elif new_task == "J2 CCW": pitch_base.run(-600 * th2_switch)                        #Compare the received message, start the motor for joint 2 counter-clockwise at a constant speed
        elif new_task == "J3 CW":  pitch_arm.run(-600 * th3_switch)                         #Compare the received message, start the motor for joint 3 clockwise at a constant speed
        elif new_task == "J3 CCW": pitch_arm.run(600 * th3_switch)                          #Compare the received message, start the motor for joint 3 counter-clockwise at a constant speed



def motor_braking():                                                                        #This function is used to prevent the arm falling down by gravity (certainly if you use a low gearing for the arm)
    while pitch_base.control.done() != True or pitch_arm.control.done() != True or roll_arm.control.done() != True or yaw_arm.control.done() != True or commands_bt_text.read() != "No movement":
        if pitch_base.control.done() == True: pitch_base.hold()                             #While 1 of the motors is not finished moving, check if joint 2 has finished, if it does, lock the motor position
        if pitch_arm.control.done() == True: pitch_arm.hold()                               #While 1 of the motors is not finished moving, check if joint 3 has finished, if it does, lock the motor position


##########~~~~~~~~~~CALCULATE AMOUNT OF STEPS NEEDED TO REACH NEXT END-POINT~~~~~~~~~~##########
def next_coordinate_linear(x_pos, y_pos, z_pos, roll, pitch, yaw, maxspeed):                #Function to move in a straight line from the current position to the new given position (X Y Z[in mm, from bottom center measured], Roll Pitch Yaw[Degrees from flat])
    global old_orientation                                                                  #Using these global variables in this local function (if not defined to be global, it will make a local variable)
    global program_start
    global old_positions
    global step

    step = math.ceil(maxspeed / stepspeed)    #200 50=fast                                  #Global 'step' is set according to the given max speed for this movement and the current stepspeed
    step_lineair = 1                                                                        #Local step counter start at 1
    
    motor_braking()                                                                         #Check if the previous end-point has been reached completely yet, if not wait for it

    if program_start == True:                                                               #For the first movement after homing, perform the next tasks
        find_position()                                                                     #Perform Forward Kinematics to get current real XYZ position.
        dist_list = [x_pos - x_pos_fork, y_pos - y_pos_fork, z_pos - z_pos_fork]            #XYZ distances needed to move to the new endpoint
        program_start = False                                                               #Set the first move variable to False
    else: dist_list = [x_pos - old_positions[0], y_pos - old_positions[1], z_pos - old_positions[2]]    #If this is not the first movement, calculate the XYZ movement to the new endpoint
    step_orientation = [roll - old_orientation[0], pitch - old_orientation[1], yaw - old_orientation[2]]    #Calculate the orientation difference for roll, pitch and yaw at the new endpoint
    
    if max(dist_list) >= math.fabs(min(dist_list)): max_distance = max(dist_list)           #Find the largest displacement between X, Y and Z if it's a positive value
    else: max_distance = math.fabs(min(dist_list))                                          #Else save the largest displacement from the largest negative value
    if max(step_orientation) >= math.fabs(min(step_orientation)):                           #Find the largest twist from any orientation roll, pitch or yaw if it's a positive value
        max_rotation = max(step_orientation) / 2                                            #Orientation movement counts as half distance displacement (don't ask my why I did this, it works)
    else: max_rotation = math.fabs(min(step_orientation)) / 2                               #Orientation movement counts as half distance displacement (don't ask my why I did this, it works)
    if max_distance < max_rotation: max_distance = max_rotation                             #Check if the largest displacement from XYZ or the orientation change is the largest
        
    sub_steps = math.floor(math.fabs(math.ceil(max_distance / step)))                       #Calculate the amount of steps needed to reach the given end-point
    
    if sub_steps <= 0: print("Position to close to previous end-point")                     #If the new position is the same as current, or to close by, don't move. Print this feedback line when debugging
    else:                                                                                   #Send each sub-step XYZ position, orientation and maxspeed
        for i in range(-((sub_steps - 1) * step), step, step):                              #For loop with the amount of steps to get to the final destination
            next_pos(x_pos - (dist_list[0] / sub_steps) * (sub_steps - step_lineair), y_pos - (dist_list[1] / sub_steps) * (sub_steps - step_lineair), z_pos - (dist_list[2] / sub_steps) * (sub_steps - step_lineair), roll - (step_orientation[0] / sub_steps) * (sub_steps - step_lineair), pitch - (step_orientation[1] / sub_steps) * (sub_steps - step_lineair), yaw - (step_orientation[2] / sub_steps) * (sub_steps - step_lineair), maxspeed)  #XYZ, Roll Pitch Yaw and Speed for the next point
            step_lineair += 1                                                               #Add 1 finished step to the variable
            #print(x_pos - (dist_list[0] / sub_steps) * (sub_steps - step_lineair), y_pos - (dist_list[1] / sub_steps) * (sub_steps - step_lineair), z_pos - (dist_list[2] / sub_steps) * (sub_steps - step_lineair), roll - (step_orientation[0] / sub_steps) * (sub_steps - step_lineair), pitch - (step_orientation[1] / sub_steps) * (sub_steps - step_lineair), yaw - (step_orientation[2] / sub_steps) * (sub_steps - step_lineair), maxspeed)

    old_orientation = [roll, pitch, yaw]                                                    #After finishing the last movement command, save the endpoint as old endpoint
    old_positions   = [x_pos, y_pos, z_pos]                                                 #After finishing the last movement command, save the endpoint as old endpoint


##########~~~~~~~~~~CALCULATE THE NEW THETAS FOR 1 POINT~~~~~~~~~~##########                #Function to move to the new given position (X Y Z[in mm, from bottom center measured], Roll Pitch Yaw[Degrees from flat])
def next_pos(x_pos, y_pos, z_pos, roll, pitch, yaw, maxspeed):
    global max_speed                                                                        #Using these global variables in this local function (if not defined to be global, it will make a local variable)
    global next_orientation
    global theta1_rot
    
    x_wrist = x_pos - (a67 * math.cos(math.radians(pitch)) * math.cos(math.radians(yaw)))   #Calculate the X-position of the wrist for the next point
    y_wrist = y_pos - (a67 * math.cos(math.radians(pitch)) * math.sin(math.radians(yaw)))   #Calculate the Y-position of the wrist for the next point
    z_wrist = z_pos - (a67 * math.sin(math.radians(-pitch)))                                #Calculate the Z-position of the wrist for the next point
    a15 = math.sqrt(((math.sqrt((x_wrist ** 2) + (y_wrist ** 2))) ** 2) + ((z_wrist - a1) ** 2))   #Calculate the distance from the center of theta2 to the center of the wrist
    
    #Find the first 3 thetas [Yaw the whole robot and both Pitch angles for the arm]
    theta1 = math.degrees(math.atan2(y_wrist, x_wrist))                                     #Calculate the angle for theta 1 (joint 1) with four-quadrant inverse tangent
    theta3 = 180 - math.degrees(math.acos(((a2 ** 2) + (a345 **2) - (a15 ** 2)) / (2 * a2 * a345))) - phi345    #Calculate the angle for theta 3 (joint 3)
    theta2 = math.degrees(math.asin((z_wrist - a1) / a15)) + math.degrees(math.acos(((a2 ** 2) + (a15 ** 2) - (a345 ** 2)) / (2 * a2 * a15))) - 90  #Calculate the angle for theta 2 (joint 2)
    theta32 = theta3 - theta2                                                               #General angle for the pitch
    
    #As the robot moves from one quadrant to another the result will shift 360°, this will compensate for axis 1 [Infinite rotation axis]
    if old_coor_list[0] - (theta1_rot * 360) < -90 and theta1 > 90: theta1_rot -= 1         #Save the global variable that holds the amount of full rotations around its axis for joint 1 performed
    if old_coor_list[0] - (theta1_rot * 360) > 90 and theta1 < -90: theta1_rot += 1         #Save the global variable that holds the amount of full rotations around its axis for joint 1 performed
    
    #Find theta5 [Yaw inside the wrist]
    theta5 = -math.degrees(math.acos(math.cos(math.radians(math.cos(math.radians(pitch)) * (theta1 + (theta1_rot * 360) - yaw))) * math.cos(math.radians(theta32 - pitch))))    #Calculate the angle for theta 5 (joint 5)

    #####################################################################
    ##########~~~~~PROGRAM WRITTEN BY JOZUA VAN RAVENHORST~~~~~##########
    ##########~~~~~~~~~~~~~~~~!!! DISCLAIMER !!!~~~~~~~~~~~~~~~##########
    ##########~~~ONLY THE FOLLOWING 30 LINES OF CODE COST ME~~~##########
    ##########~~OVER 200 HOURS OF CALCULATION TO GET CORRECT~~~##########
    ##########~~~~~INVERSE KINEMATIC CONTROL WITHOUT NUMPY~~~~~##########
    ##########~~~~~~NUMPY DOES NOT WORK WITH MICROPYTHON~~~~~~~##########
    ##########~~PLEASE MENTION ME IF YOU USE ANY OF THIS CODE~~##########
    #####################################################################

    #Find theta4, Inverse Kinematic with quadrants [Roll of the wrist] 
    if theta1 + (theta1_rot * 360) - yaw > 0 and theta1 + (theta1_rot * 360) - yaw < 180 and pitch != 90 and pitch != -90:
        theta4 = math.degrees(math.acos(-math.sin(math.radians(theta32 - pitch)) * math.cos(math.radians(math.cos(math.radians(pitch)) * (theta1 + (theta1_rot * 360) - yaw))) / math.sin(math.radians(theta5))))    #### - pitch! both
    elif theta1 + (theta1_rot * 360) - yaw < 0 and theta1 + (theta1_rot * 360) - yaw > -180 and pitch != 90 and pitch != -90:
        theta4 = math.degrees(math.acos(-math.sin(math.radians(theta32 - pitch)) * math.cos(math.radians(math.cos(math.radians(pitch)) * (theta1 + (theta1_rot * 360) - yaw))) / math.sin(math.radians(-theta5)))) + 180
    elif theta32 - pitch > 0:
        theta4 = 0
    elif theta32 - pitch < 0:
        theta4 = 180
    else:
        theta4 = round(roll_arm.angle() / roll_arm_gear, 2)

    #Find theta6, Inverse Kinematic with quadrants [Roll of the head]
    if theta32 - pitch > 0 and theta5 != 0:
        theta6 = math.cos(math.radians(pitch)) * math.degrees(math.asin(-math.cos(math.radians(theta32 - pitch)) * math.sin(math.radians(math.cos(math.radians(pitch)) * (theta1 + (theta1_rot * 360) - math.fmod(yaw, 360)))) / math.sin(math.radians(-theta5)))) + \
            (math.sin(math.radians(pitch)) * (theta1 - yaw)) + roll
    elif theta32 - pitch < 0 and theta5 != 0:
        theta6 = math.cos(math.radians(pitch)) * math.degrees(math.asin(-math.cos(math.radians(theta32 - pitch)) * math.sin(math.radians(math.cos(math.radians(pitch)) * (theta1 + (theta1_rot * 360) - math.fmod(yaw, 360)))) / math.sin(math.radians(theta5)))) + \
            180 + (math.sin(math.radians(pitch)) * (theta1 + (theta1_rot * 360) - yaw)) + roll
    elif theta1 - math.fmod(yaw, 360) < 0:
        if math.fmod(yaw, 360) == 0:
            theta6 = 270 + roll
        elif math.fmod(yaw, 360) != 0:
            theta6 = 90 + roll
    elif theta1 - math.fmod(yaw, 360) > 0:
        if math.fmod(yaw, 360) == 0:
            theta6 = 90 + roll
        elif math.fmod(yaw, 360) != 0:
            theta6 = 270 + roll
    else:
        theta6 = round((roll_head_feedb.read() /roll_head_gear) + (round(roll_arm.angle() / roll_arm_gear, 2) / roll_head_gear) - (round((yaw_arm.angle() / yaw_arm_gear) - (round(roll_arm.angle() / roll_arm_gear, 2) / 5), 2) * 4 / roll_head_gear / roll_head_gear), 2) + roll

    #To debug print all values
    #print(theta1, theta2, theta3, theta4, theta5, theta6, roll, pitch, yaw, theta1_rot, theta4_rot, theta6_rot, theta32, x_pos_fork, y_pos_fork, z_pos_fork)

    #check that the values for theta2, theta3 and theta5 are within range of the mechanical capabilities
    possible_angles = "OK"                                                                  #Start the variable with no error
    if theta2 > 28 or theta2 < -86:     ###Max theta2 angles are 44° and -95° in the small robot
        possible_angles = "Theta2 out of range"                                             #Set an error for possible mechanical crash if this movement was to be performed
    elif theta2 > 8 and theta3 < -45 or theta2 <= 0 and theta3 < -78 or theta2 > 0 and theta2 <=8 and theta3 < -78 + 4 * theta2:    #Minimal theta3 angles check
        possible_angles = "Theta3 to small"                                                 #Set an error for possible mechanical crash if this movement was to be performed
    #elif theta2 >= -30 and theta3 > 75.2 or \                              
    #    theta2 >= -49 and theta2 < -30 and theta3 > 108 + theta2:                          ###105!!!
    #    possible_angles = "Theta3 to big"                                                  #TODO NOT ALL LIMITS ARE DEFINED, NEED TO ADD MORE LIMITS FOR THETA3 TO PREVENT CRASHES!!!
    elif theta5 > 5 or theta5 < -123:                                                       #Theta5 is always negative, roll of the wrist has to compensate for this
        possible_angles = "Theta5 out of range"                                             #Set an error for possible mechanical crash if this movement was to be performed
    
    #Send all thetas to find the correct matching speeds
    if possible_angles == "OK":                                                             #If there have been no possible mechanical crash errors found
        max_speed = int(maxspeed)                                                           #Set the global maximal speed with the given function maximal speed
        next_orientation = [roll, pitch, yaw]                                               #Save the orientation as a global variable list
        calc_speed_motors([theta1, theta2, theta3, theta4, theta5, theta6])                 #Call the function that will calculate the speeds for each motor to arrive at the same time
    else:
        print(possible_angles, theta1, theta2, theta3, theta4, theta5, theta6)              #If a possible mechanical crash has been found, print the error and angles for debugging
        ev3.speaker.beep()                                                                  #Make a beep sound to acknowledge an out of bounds movement was requested


##########~~~~~~~~~~CALCULATE SPEED FOR EACH MOTOR TO ARRIVE AT THE SAME TIME~~~~~~~~~~##########
def calc_speed_motors(new_coor_list):                                                       #Function to calculate each motor's speed to reach the new given position at the same time (6x axis angle in degrees (0-360°)
    global theta1_rot                                                                       #Using these global variables in this local function (if not defined to be global, it will make a local variable)
    global theta4_rot
    global theta6_rot
    global max_speed
    global old_speed
    global old_orientation
    global next_orientation
    global actuation_list
    global esp_adj
    
    #As the robot moves from one quadrant to another the result will shift 360°, this will compensate for axis 4 and 6 [Infinite rotation axis]
    if old_coor_list[0] - old_orientation[2] - (theta1_rot * 360) >= 0 and new_coor_list[0] - next_orientation[2] < 0 and new_coor_list[2] - new_coor_list[1] - old_orientation[1] > 0:
        theta4_rot -=  1
    if old_coor_list[0] - old_orientation[2] - (theta1_rot * 360) < 0 and new_coor_list[0] - next_orientation[2] >= 0 and new_coor_list[2] - new_coor_list[1] - old_orientation[1] > 0:
        theta4_rot += 1    
    if old_coor_list[2] - old_coor_list[1] - old_orientation[1] >= 0 and new_coor_list[2] - new_coor_list[1] - next_orientation[1]  < 0 and new_coor_list[0] - next_orientation[2] > 0:
        theta6_rot -= 1
    if old_coor_list[2] - old_coor_list[1] - old_orientation[1] < 0 and new_coor_list[2] - new_coor_list[1] - next_orientation[1] >= 0 and new_coor_list[0] - next_orientation[2] > 0:
        theta6_rot += 1
    new_coor_list[0] += (360 * theta1_rot)                                                  #Add 360° for every full rotation already done to the calculated angle
    new_coor_list[3] += (360 * theta4_rot)                                                  #Add 360° for every full rotation already done to the calculated angle
    new_coor_list[5] += (360 * theta6_rot)                                                  #Add 360° for every full rotation already done to the calculated angle
    
    #Calculate how much degrees thetas have to change with realtime current position feedback
    move_coor_list[0] = new_coor_list[0] - old_coor_list[0] - ( esp_adj[0] / yaw_base_gear )    #It takes the real current position of joint 1 NOT into account, if it lags behind due to friction, it will NOT get more speed/power this setpoint, the bluetooth feedback comes to slow and would make the error worse than without it. This joint usually never has friction/power problems
    move_coor_list[1] = new_coor_list[1] - round(pitch_base.angle() * th2_switch / pitch_base_gear, 2) - ( esp_adj[1] / pitch_base_gear )   #It takes the real current position of joint 2 into account, if it lags behind due to friction, it will get more speed/power this setpoint
    move_coor_list[2] = new_coor_list[2] - round(pitch_arm.angle() * th3_switch / pitch_arm_gear, 2) - ( esp_adj[2] / pitch_arm_gear )      #It takes the real current position of joint 3 into account, if it lags behind due to friction, it will get more speed/power this setpoint
    move_coor_list[3] = new_coor_list[3] - round(roll_arm.angle() / roll_arm_gear, 2) - ( esp_adj[3] / roll_arm_gear )                      #It takes the real current position of joint 4 into account, if it lags behind due to friction, it will get more speed/power this setpoint
    move_coor_list[4] = new_coor_list[4] - round((yaw_arm.angle() / yaw_arm_gear) - (round(roll_arm.angle() / roll_arm_gear * 20 / 12, 2) / roll_arm_gear), 2) - ( esp_adj[4] / yaw_arm_gear)   #It takes the real current position of joint 5 into account, if it lags behind due to friction, it will get more speed/power this setpoint
    move_coor_list[5] = new_coor_list[5] - old_coor_list[5] - ( esp_adj[5] / roll_head_gear )   #It takes the real current position of joint 6 NOT into account, if it lags behind due to friction, it will NOT get more speed/power this setpoint, the bluetooth feedback comes to slow and would make the error worse than without it. This joint usually never has friction/power problems
    
    #Save the new orientation as old, preparing for future calculations
    for i in range(3): old_orientation[i] = next_orientation[i]

    #Next line is for debugging, showing time needed to calculate and performing 1 step
    #print("Looptime", timer_movement.time(), "Time to execute previous movement", max(move_angle_list) / old_speed * 1000)
    #print(new_coor_list)
    
    check_emergency_stop()                                                                  #Check if there is an emergency state, if there is wait for reset (Movement will stop when the previous intermediate point is reached)
    #Waiting cycle, calculate the time it takes to perform the previous step and overlap for a smooth movement
    while timer_movement.time() < max(move_angle_list) / old_speed * 1000 - overlaptime: continue   #Calculating the time needed to finish the previous movement and set some overlap to make a smooth movement #-250ms overlap (-300 on 29/01/2021) (-350 on 5/03/2022)
    timer_movement.reset()                                                                  #Reset the timer to 0
    old_speed = int(max_speed)                                                              #Save current step speed as old speed

    #Calculate how many degrees each motor should turn (absolute)
    move_angle_list[0] = math.fabs(int( move_coor_list[0] * yaw_base_gear))                 #Angle that motor 1 should rotate for this step
    move_angle_list[1] = math.fabs(int( move_coor_list[1] * pitch_base_gear))               #Angle that motor 2 should rotate for this step
    move_angle_list[2] = math.fabs(int( move_coor_list[2] * pitch_arm_gear))                #Angle that motor 3 should rotate for this step
    move_angle_list[3] = math.fabs(int( move_coor_list[3] * roll_arm_gear))                 #Angle that motor 4 should rotate for this step
    move_angle_list[4] = math.fabs(int((move_coor_list[4] * yaw_arm_gear) + (move_coor_list[3] / roll_arm_gear * 20 / 12 * yaw_arm_gear)))          #Angle that motor 5 should rotate for this step, it has to compensate for joint 4 as the driveline is going through joint 4 and rotates with it
    move_angle_list[5] = math.fabs(int((move_coor_list[5] * roll_head_gear) - (move_coor_list[3] * 1) + (move_coor_list[4] * 4 / roll_head_gear)))  #Angle that motor 6 should rotate for this step, it has to compensate for joint 4 and 5 as the driveline is going through joint 4 and 5 and rotates with them

    max_angle = max(move_angle_list)                                                        #Find the most degrees any motor has to turn

    if max_angle != 0:
        for i in range(6):                                                                  #Looping for the 6 axis
            if int(move_angle_list[i] / max_angle * max_speed) < 50: speed_list[i] = 50     #If result for (current axis distance)/(max distance)*(max speed) < 20, set the speed to 20, to prevent stalling
            else: speed_list[i] = int((move_angle_list[i] / max_angle) * max_speed)         #Else, set current axis speed to the calculated value
            old_coor_list[i] = float(new_coor_list[i])                                      #Save current thetas as old thetas, preparing for future calculations

        actuation_list = [new_coor_list[0], new_coor_list[1], new_coor_list[2], new_coor_list[3], new_coor_list[4], new_coor_list[5], \
speed_list[0], speed_list[1], speed_list[2], speed_list[3], speed_list[4], speed_list[5]]   #Override the global variable that holds the current desired motor angles and speeds for all 6 motors
        #find_position()                                                                    #FOR BUGFIXING, this does slow down the program as it's a long calculation function
        sub_move_all_motors.start()                                                         #Start the function that starts all the motors. Non-blocking! Calculation for the next points can already begin now


def move_all_motors():                                                                      #Function that will send all 6 motors their target position and speed to go there
    global actuation_list                                                                   #Using these global variables in this local function (if not defined to be global, it will make a local variable)
    global esp_adj

    yaw_base_bt_sp.send(  int(max_speed))                                                   #Send the speed for joint 1 by bluetooth
    yaw_base_bt_num.send( int(actuation_list[0] * yaw_base_gear) + esp_adj[0])              #Send the target angle for joint 1 by bluetooth
    pitch_base.run_target(actuation_list[7], int(actuation_list[1] * pitch_base_gear * th2_switch) + esp_adj[1], then=Stop.HOLD, wait=False)    #Run to target at a given speed for joint 2. #then=Stop.COAST originally to prevent shocking movements, but angle may overshoot
    pitch_arm.run_target( actuation_list[8], int(actuation_list[2] * pitch_arm_gear *  th3_switch) + esp_adj[2], then=Stop.HOLD, wait=False)    #Run to target at a given speed for joint 3. #then=Stop.COAST originally to prevent shocking movements, but angle may overshoot
    roll_arm.run_target(  actuation_list[9], int(actuation_list[3] * roll_arm_gear) + esp_adj[3], then=Stop.COAST, wait=False)                  #Run to target at a given speed for joint 4.
    yaw_arm.run_target(   actuation_list[10], int((actuation_list[4] * yaw_arm_gear) + (actuation_list[3] / roll_arm_gear * 20 / 12 * yaw_arm_gear)) + esp_adj[4], then=Stop.COAST, wait=False) #Run to target at a given speed for joint 5.
    roll_head_bt_sp.send( int(max_speed))                                                   #Send the speed for joint 6 by bluetooth
    roll_head_bt_num.send(int((actuation_list[5] * roll_head_gear) - (actuation_list[3] * 1) + (actuation_list[4] * 4 / roll_head_gear)) + esp_adj[5])  #Send the target angle for joint 6 by bluetooth
   

##########~~~~~~~~~~FORWARD KINEMATICS FOR FINDING REALTIME XYZ POSITION~~~~~~~~~~##########
def find_position():                                                                        #Function to calculate the forward kinematics (Realtime XYZ position of the center of the forkboard, against the board)
    global x_pos_fork                                                                       #Using these global variables in this local function (if not defined to be global, it will make a local variable)
    global y_pos_fork
    global z_pos_fork
    yaw_base_angle =   math.radians(round(yaw_base_feedb.read() / yaw_base_gear, 2))        #Use the feedback value from the slave brick, to calculate the radian angle of joint 1
    pitch_base_angle = math.radians(round(pitch_base.angle() / pitch_base_gear * th2_switch, 2) + 90)   #Use the motor feedback angle from joint 2, to calculate the radian angle of joint 2
    pitch_arm_angle =  math.radians(round(pitch_arm.angle() / pitch_arm_gear * th3_switch, 2))          #Use the motor feedback angle from joint 3, to calculate the radian angle of joint 3
    roll_arm_angle =   math.radians(round(roll_arm.angle() / roll_arm_gear, 2))                         #Use the motor feedback angle from joint 4, to calculate the radian angle of joint 4
    yaw_arm_angle =    math.radians(round((yaw_arm.angle() / yaw_arm_gear) - (roll_arm_angle / roll_arm_gear), 2))  #Use the motor feedback angle from joint 5, to calculate the radian angle of joint 5
    roll_head_angle =  math.radians(round((roll_head_feedb.read() / roll_head_gear) + (roll_arm_angle / roll_head_gear) - (yaw_arm_angle * 4 / roll_head_gear / roll_head_gear), 2))    #Use the feedback value from the slave brick, to calculate the radian angle of joint 6

    yawbasecos = math.cos(yaw_base_angle)                                                   #Precalculate the cosine function, as it will be used many times
    yawbasesin = math.sin(yaw_base_angle)                                                   #Precalculate the sine   function, as it will be used many times
    pitbasecos = math.cos(pitch_base_angle)
    pitbasesin = math.sin(pitch_base_angle)
    pitarmcos  = math.cos(pitch_arm_angle)
    pitarmsin  = math.sin(pitch_arm_angle)
    rolarmcos  = math.cos(roll_arm_angle)
    rolarmsin  = math.sin(roll_arm_angle)
    yawarmcos  = math.cos(yaw_arm_angle)
    yawarmsin  = math.sin(yaw_arm_angle)

    x_pos_fork = round(- a67 * yawbasecos * pitbasecos * pitarmcos * rolarmcos * yawarmsin - a67 * yawbasecos * pitbasesin * pitarmsin * rolarmcos * yawarmsin \
- a67 * yawbasesin * rolarmsin * yawarmsin - a67 * yawbasecos * pitbasecos * pitarmsin * yawarmcos + a67 * yawbasecos * pitbasesin * pitarmcos * yawarmcos \
- a45 * yawbasecos * pitbasecos * pitarmsin + a45 * yawbasecos * pitbasesin * pitarmcos +  a3 * yawbasecos * pitbasecos * pitarmcos \
+  a3 * yawbasecos * pitbasesin * pitarmsin +  a2 * yawbasecos * pitbasecos)                #The current X-position calculated with kinematics
    y_pos_fork = round(- a67 * yawbasesin * pitbasecos * pitarmcos * rolarmcos * yawarmsin - a67 * yawbasesin * pitbasesin * pitarmsin * rolarmcos * yawarmsin \
+ a67 * yawbasecos * rolarmsin * yawarmsin - a67 * yawbasesin * pitbasecos * pitarmsin * yawarmcos + a67 * yawbasesin * pitbasesin * pitarmcos * yawarmcos \
- a45 * yawbasesin * pitbasecos * pitarmsin + a45 * yawbasesin * pitbasesin * pitarmcos +  a3 * yawbasesin * pitbasecos * pitarmcos\
+  a3 * yawbasesin * pitbasesin * pitarmsin +  a2 * yawbasesin * pitbasecos)                #The current Y-position calculated with kinematics
    z_pos_fork = round(\
- a67 * pitbasesin * pitarmcos * rolarmcos * yawarmsin + a67 * pitbasecos * pitarmsin * rolarmcos * yawarmsin - a67 * pitbasesin * pitarmsin * yawarmcos \
- a67 * pitbasecos * pitarmcos * yawarmcos - a45 * pitbasesin * pitarmsin - a45 * pitbasecos * pitarmcos +  a3 * pitbasesin * pitarmcos \
-  a3 * pitbasecos * pitarmsin +  a2 * pitbasesin + a1)                                     #The current Z-position calculated with kinematics

    #print(x_pos_fork, y_pos_fork, z_pos_fork, math.degrees(yaw_base_angle), math.degrees(pitch_base_angle) - 90, math.degrees(pitch_arm_angle), math.degrees(roll_arm_angle), math.degrees(yaw_arm_angle), math.degrees(roll_head_angle))
    

def change_stepspeed():                                                                     #Function that executes the robot tasks and manual adjust of speed/stepsize with EV3 buttons
    global lastpress                                                                        #Using these global variables in this local function (if not defined to be global, it will make a local variable)
    global stepspeed
    global overlaptime
    global job

    while True:                                                                             #Start a forever loop
        wait(1000)
        if job != 0: moving_scissor_and_zones()                                             #Function that makes the whole robot do a task. Blocking function
        lastpress = ev3.buttons.pressed()                                                   #Check if a button on the EV3 has been pressed
        if lastpress == [Button.UP] and stepspeed < 500: stepspeed += 10                    #If the up    button was pressed make the stepspeed higher
        elif lastpress == [Button.DOWN] and stepspeed > 20: stepspeed -= 10                 #If the down  button was pressed make the stepspeed lower
        elif lastpress == [Button.RIGHT] and overlaptime < 400: overlaptime += 10           #If the right button was pressed make the overlaptime higher
        elif lastpress == [Button.LEFT] and overlaptime > 20: overlaptime -= 10             #If the left  button was pressed make the overlaptime lower
        ev3.screen.draw_text(4, 40, stepspeed, text_color=Color.BLACK, background_color=Color.WHITE)    #X/Y position for writing on the screen, textstring, text color, background color
        ev3.screen.draw_text(4, 60, overlaptime, text_color=Color.BLACK, background_color=Color.WHITE)  #X/Y position for writing on the screen, textstring, text color, background color


##########~~~~~~~~~~CREATING MULTITHREADS~~~~~~~~~~##########
sub_find_position = Thread(target=find_position)                                            #This creates a thread, made from a previously defined function. No arguments can be given
sub_move_all_motors = Thread(target=move_all_motors)
sub_change_stepspeed = Thread(target=change_stepspeed)


#sub_find_position.start()                                                                  #This starts the thread for finding realtime XYZ position in the background (NOT USED)
sub_change_stepspeed.start()                                                                #This starts the loop thread for executing a job, or adjusting speeds. Non-blocking

##########~~~~~~~~~~WAIT UNTIL (1) BLUETOOTH DEVICE IS CONNECTED~~~~~~~~~~##########
##########ALWAYS START THIS SERVER-BRICK FIRST. THEN START THE SLAVE-BRICKS, OR THEY WILL TIMEOUT IF THEY CAN NOT CONNECT TO THIS BRICK BY BLUETOOTH (THIS PROGRAM NEEDS TO BE RUNNING##########
server.wait_for_connection(1)                                                               #Waiting for the amount of bluetooth devices connected
wait(500)


##########~~~~~~~~~~MANUAL CONTROL WITH ESP TOUCHSCREEN GUI FOR GOING TO A SAFE POSITION BEFORE HOMING~~~~~~~~~~##########
manual_esp_homing()                                                                         #Function to set the robot in a safe stance to start homing. Blocking


##########~~~~~~~~~~WAIT 1 SECOND BEFORE STARTING BLUETOOTH COMMUNICATION~~~~~~~~~~##########
wait(1000)      #TODO is this still needed? BT already online longtime

commands_bt_text.send('Initiate yaw base')                                                  #Send the command for homing theta1        
yaw_base_bt_sp.send(800)                                                                    #Send maximal speed for theta1             
roll_head_bt_sp.send(800)                                                                   #Send maximal speed for theta6                
yaw_base_bt_zeroing.send(int(yaw_base_zeroing))                                             #Send the angle to set after homing for theta1
roll_head_bt_zeroing.send(int(roll_head_zeroing))                                           #Send the angle to set after homing for theta6


##########~~~~~~~~~~EV3 HAS INCREMENTAL ENCODERS IN IT'S MOTORS, SO IT'S NEEDED TO CALIBRATE EVERY STARTUP~~~~~~~~~~##########
##########~~~~~~~~~~HOMING THETA3~~~~~~~~~~##########
if touch_pitch_base.pressed() == True:                                                      #If theta2 is on his touch sensor, make it go up first to avoid a crash
    while touch_pitch_base.pressed() == True:
        pitch_base.run(-400 * th2_switch)
    wait(3000)
pitch_base.hold()

if touch_pitch_arm.pressed() == True:                                                       #If theta3 is pressing its touch sensor, make it move forward until it does not touch anymore
    while touch_pitch_arm.pressed() == True:
        pitch_arm.run(600 * th3_switch)
    wait(100)
    pitch_arm.stop()
    wait(250)

while touch_pitch_arm.pressed() == False:                                                   #Homing theta3
    pitch_arm.run(-600 * th3_switch)
pitch_arm.hold()                                                                            #Active brake theta3
pitch_arm.reset_angle(pitch_arm_zeroing)                                                    #Set motor angle for theta3 to the homing angle

pitch_arm.run_target(800, 0)


##########~~~~~~~~~~HOMING THETA4~~~~~~~~~~##########
if touch_roll_arm.pressed() == True:                                                        #If theta4 is pressing its touch sensor, make it move back until it does not touch anymore
    while touch_roll_arm.pressed() == True:
        roll_arm.run(-200)                                                                  #Turning theta4 will make theta5 and theta6 turn as well
        yaw_arm.run(-200/2)                                                                 #Turn theta5 at 2.5x smaller speed to avoid a collision     #-80    #### -200/5 V3
    wait(750)
roll_arm.stop()                                                                             #Soft brake theta4
yaw_arm.stop()                                                                              #Soft brake theta5

while touch_roll_arm.pressed() != True:                                                     #Homing theta4
    roll_arm.run(600) #### 200
    yaw_arm.run(600/2)                                                                      #### 80              #### 600/5  V3
roll_arm.hold()                                                                             #Active brake theta4
yaw_arm.stop()                                                                              #Soft brake theta5
roll_arm.reset_angle(roll_arm_zeroing)                                                      #Set motor angle for theta4 to the homing angle


##########~~~~~~~~~~HOMING THETA5~~~~~~~~~~##########
yaw_arm.run_angle(800/5, (roll_arm_zeroing + (roll_arm_full_rot / 4)) / -2, wait=False)     #Turn theta5 to match theta4 rotations           ####*3 added         -5 V3 gear 20/16
roll_arm.run_target(800, (roll_arm_full_rot) / 4)                                           #Turn theta4 90° so theta5 can be safely zeroed
yaw_arm.run_until_stalled(-800, then=Stop.BRAKE, duty_limit=70)                             #Homing theta5 with stall detection, no brake at all after stall
wait(150)                                                                                   #Wait 150ms for easing theta5
yaw_arm.reset_angle(yaw_arm_zeroing)                                                        #Set motor angle for theta5 to the homing angle
yaw_arm.run_target(800, - yaw_arm_full_rot / 4)                                             #Theta5 go to safe position -90°
roll_arm.run_target(800, 0)                                                                 #Theta4 go to safe position   0°


##########~~~~~~~~~~HOMING THETA2~~~~~~~~~~##########
while commands_bt_text.read() != 'Initiated yaw base':                                      #Check if theta1 has finished homing
    continue
yaw_base_bt_num.send(0)                                                                     #Theta1 go to safe position 0°
pitch_arm.run_target(800, pitch_arm_full_rot / 8 * th3_switch)                              #Theta3 go to safe position 0°

if touch_pitch_base.pressed() == True:                                                      #If theta2 is pressing its touch sensor, make it move back until it does not touch anymore
    while touch_pitch_base.pressed() == True:
        pitch_base.run(-800 * th2_switch)
    wait(700)                                                                               #Time for running backwards
    pitch_base.stop()                                                                       #Soft brake theta2
    wait(250)

while touch_pitch_base.pressed() == False:                                                  #Homing theta2
    pitch_base.run(400 * th2_switch)
pitch_base.hold()                                                                           #Active brake theta2
pitch_base.reset_angle(pitch_base_zeroing)                                                  #Set motor angle for theta2 to the homing angle


##########~~~~~~~~~~HOMING THETA6~~~~~~~~~~##########
#Turning the wrist 90degrees, and bending the wrist 90degrees
yaw_arm.run_target(800, - yaw_arm_full_rot / 4 - (90 / 5 * yaw_arm_gear), wait=False)       #5 stands for the 60/12reduction that needs to counterturn the wrist roll
roll_arm.run_target(800, - roll_arm_full_rot / 4)
pitch_arm.run_target(800, pitch_arm_full_rot / 360 * roll_head_zeroing_pitch_arm_angle * th3_switch, wait=False)
pitch_base.run_target(800, - pitch_base_full_rot / 360 * roll_head_zeroing_pitch_base_angle * th2_switch)   
pitch_base.hold()
pitch_arm.hold()


commands_bt_text.send('Initiate roll head')                                                 #Send the command for homing theta6
while commands_bt_text.read() != 'Initiated roll head':
    continue                                                                                #Check if theta6 has finished homing
roll_head_bt_num.send(0)                                                                    #Theta6 go to position 0°


##########~~~~~~~~~~ALL THETAS GO TO 0°~~~~~~~~~~##########
pitch_arm.run_target(800, 10, wait=False)
pitch_base.run_target(800, 10)
yaw_arm.run_target(800, 0, wait=False) 
roll_arm.run_target(800, 0)
yaw_base_bt_num.send(0)

find_position()
wait(500)

#ev3.speaker.say("All motor positions at 0 degrees")        #[Not used]


#####################################################################
#####################################################################
##########~~~~~~~~~~~~~~~~HOMING ALL JOINTS~~~~~~~~~~~~~~~~##########
##########~~~~~~~~~~~~~~~FINISHED, ALL AT 0°~~~~~~~~~~~~~~~##########
##########~~~~~~~~~~~~~~~~SEND COORDINATES~~~~~~~~~~~~~~~~~##########
#####################################################################
##########~~~~~~~~~~~~~1) INVERSE KINEMATIC~~~~~~~~~~~~~~~~##########
##########~~~~~~~~~~~~~POSITION END-EFFECTOR~~~~~~~~~~~~~~~##########
##########~~~~X Y Z (mm) ROLL PITCH YAW (°) SPEED (°/s)~~~~##########
#####################################################################
##########~~~~~~~~~~~~~2) DIRECT POINT CONTROL~~~~~~~~~~~~~##########
##########~TH1 TH2 TH3 TH4 TH5 TH6 SP1 SP2 SP3 SP4 SP5 SP6~##########
#####################################################################
#####################################################################

# 1) Example             X     Y     Z    Roll  Pitch  Yaw  Speed
#next_coordinate_linear( 270,  100,  250,    0,    0,    0, 700) 

# 2) Example      TH1   TH2   TH3   TH4   TH5   TH6  SP1  SP2  SP3  SP4  SP5  SP6     THETAS in real angles for the joint, SPEED minimal 50 and maximal 800
#move_all_motors(  45,   10,  -20,   70,  -60,  180, 500, 700, 600, 600, 200, 100)
#THETAS [1: +Counter-clockwise from top; 2: +lean backwards; 3: +tip forward; 4: +clockwise from rear; 5: +tip down to zero-point arm; 6: +clockwise from rear]

next_coordinate_linear( 200, -100,  280,   -5,   -10,    0, 1000)                           #Going to safe start position after homing
feedback_commands_bt.send("Homing finished")
wait(1500)
feedback_commands_bt.send("Ready")

def moving_scissor_and_zones():
    global job                                                                              #Using this global variable in this local function (if not defined to be global, it will make a local variable)
    
    if   0 < job < 100:                                                                     #Pickup at scissor and dropoff on the floor
        scissor_loc("pickup")
        dropoff_loc(job)
    elif 100 < job < 200:                                                                   #Pickup on the floor and dropoff at scissor standard height
        pickup_loc(job - 100)
        scissor_loc("dropoff")


def scissor_loc(mode):
    global scissor_up                                                                       #Using this global variable in this local function (if not defined to be global, it will make a local variable)
    
    if   mode == "pickup":
        next_coordinate_linear( 190, -345,  290,  -10,   -12,    0,  900)       #1000       #next_coordinate_linear( 200, -345,  290,  -10,   -12,    0,  900)
        next_coordinate_linear( 320, -345,  290,  -10,   -12,    0,  300)       # 600
        next_coordinate_linear( 320, -345,  400,  -10,   -12,    0,  900)       # 800
        feedback_commands_bt.send("Picked up")
    elif mode == "dropoff":
        next_coordinate_linear( 310, -340,  400,  -10,   -12,    0,  900)       #1000       #next_coordinate_linear( 320, -340,  400,  -10,   -12,    0,  900)
        while scissor_up == False: wait(200)
        next_coordinate_linear( 310, -340,  290,   -5,   -12,    0,  400)       # 800       #next_coordinate_linear( 320, -340,  290,   -5,   -12,    0,  400)
        next_coordinate_linear( 180, -340,  290,  -10,   -12,    0,  300)                   #next_coordinate_linear( 190, -340,  290,  -10,   -12,    0,  300)
        feedback_commands_bt.send("Ready")

    
def dropoff_loc(pos):
    global job                                                                              #Using this global variable in this local function (if not defined to be global, it will make a local variable)

    print("started to go to dropoff", pos)
    if   pos == 1:
        next_coordinate_linear( 335,   40,  400,    0,   -10,    0,  900)                   #TODO Roll and Pitch have some issues with the XL robot, need to find why still
        next_coordinate_linear( 335,   40,   70,   -5,   -10,    0,  800)
        next_coordinate_linear( 200,   40,   70,   -5,    -4,    0,  300)
        next_coordinate_linear( 200,   40,  290,    0,    -5,    0,  900)       # 800
    elif pos == 2:
        next_coordinate_linear( 335,  175,  400,    0,   -10,   -5,  900)
        next_coordinate_linear( 335,  175,   80,    0,   -10,   -5,  800)
        next_coordinate_linear( 200,  175,   80,    0,    -4,   -5,  300)
        next_coordinate_linear( 200,  175,  290,    0,    -5,   -5,  900)       # 800
    elif pos == 3:
        next_coordinate_linear( 300,  290,  400,    0,   -10,   -5,  900)
        next_coordinate_linear( 300,  290,  400,    0,   -10,   40,  400)
        next_coordinate_linear( 300,  290,   80,    0,   -10,   40,  800)
        next_coordinate_linear( 200,  190,   80,    0,    -4,   40,  300)
        next_coordinate_linear( 200,  190,  290,    0,    -5,   40,  900)       # 800
        next_coordinate_linear( 200,  190,  290,    0,    -5,   -5,  900)
    elif pos == 4:
        next_coordinate_linear( 150,  290,  400,    0,   -10,   -5,  900)
        next_coordinate_linear( 150,  290,  400,  -20,   -25,  130,  400)
        next_coordinate_linear( 150,  290,   90,  -30,   -25,  135,  800)
        next_coordinate_linear( 225,  170,   90,  -35,   -30,  135,  300)
        next_coordinate_linear( 225,  170,  290,  -20,    -5,  135,  900)       # 800
        next_coordinate_linear( 225,  170,  290,    0,    -5,   -5, 1200)
    elif pos == 5:
        next_coordinate_linear(-100, -320,  400,    5,   -12, -179,  900)
        next_coordinate_linear(-100, -320,   80,   10,   -12, -185,  800)
        next_coordinate_linear(  35, -320,   80,   10,   -12, -185,  300)
        next_coordinate_linear(  35, -320,  290,   15,   -12, -179,  900)       # 800
        next_coordinate_linear(  35, -320,  290,  -10,   -12,    0, 1200)
        next_coordinate_linear( 200, -345,  290,  -10,   -12,    0,  900)
    elif pos == 6:
        next_coordinate_linear(-120, -185,  400,    5,   -12, -179,  900)
        next_coordinate_linear(-120, -185,   75,   10,   -12, -189,  800)
        next_coordinate_linear(  15, -185,   70,   10,   -12, -187,  300)
        next_coordinate_linear(  15, -185,  290,   10,   -12, -179,  900)       # 800
        next_coordinate_linear(  15, -185,  290,  -10,   -12,    0, 1200)
        next_coordinate_linear( 200, -345,  290,  -10,   -12,    0,  900)
    job = 0
    feedback_commands_bt.send("Ready")
    

def pickup_loc(pos):
    global job                                                                              #Using this global variable in this local function (if not defined to be global, it will make a local variable)
    
    if   pos == 1:
        next_coordinate_linear( 200,   40,  290,    0,    -5,    0,  900)       #0  -5   0
        next_coordinate_linear( 200,   40,   60,   -5,    -2,    0,  900)       #0  -4   0      70  800
        next_coordinate_linear( 335,   40,   60,   -5,    -5,    0,  300)       #0 -10   0      70
        next_coordinate_linear( 335,   40,  400,    0,   -10,    0,  900)       #800
    elif pos == 2:
        next_coordinate_linear( 200,  175,  290,    0,    -5,   -5,  900)
        next_coordinate_linear( 200,  175,   70,    0,    -4,   -5,  900)       #               80  800
        next_coordinate_linear( 335,  175,   70,    0,   -10,   -5,  300)       #               80
        next_coordinate_linear( 335,  175,  400,    0,   -10,   -5,  900)       #800
    elif pos == 3:
        next_coordinate_linear( 200,  190,  290,    0,    -5,   -5,  900)
        next_coordinate_linear( 200,  190,  290,    0,    -5,   40, 1200)
        next_coordinate_linear( 200,  190,   65,    0,    -5,   40,  900)       #0  -2  40      80  800
        next_coordinate_linear( 300,  290,   65,    0,    -4,   40,  300)       #0 -10  40      80
        next_coordinate_linear( 300,  290,  400,    0,   -10,   40,  900)       #800
        next_coordinate_linear( 300,  290,  400,    0,   -10,   -5,  400)
    elif pos == 4:
        next_coordinate_linear( 225,  170,  290,    0,    -5,   -5,  900)
        next_coordinate_linear( 225,  170,  290,  -25,    -5,  135, 1200)
        next_coordinate_linear( 225,  170,  105,  -20,   -25,  135,  900)       #-20  -25  135  90  800
        next_coordinate_linear( 150,  290,  105,  -20,   -25,  135,  300)       #-20  -25  135  90
        next_coordinate_linear( 150,  290,  400,  -20,   -25,  130,  900)       #800
        next_coordinate_linear( 150,  290,  400,    0,   -10,   -5,  400)
    elif pos == 5:
        next_coordinate_linear( 200, -345,  290,  -10,   -12,    0,  900)
        next_coordinate_linear(  35, -320,  290,  -10,   -12,    0,  900)
        next_coordinate_linear(  35, -320,  290,   15,   -12, -179, 1200)
        next_coordinate_linear(  35, -320,   80,   10,   -12, -185,  900)       #15  -12  -185  80  800
        next_coordinate_linear(-100, -320,   80,   10,   -12, -185,  300)       #15  -12  -185  80
        next_coordinate_linear(-100, -320,  400,   10,   -12, -179,  900)       #800
    elif pos == 6:
        next_coordinate_linear( 200, -345,  290,  -10,   -12,    0,  900)
        next_coordinate_linear(  15, -185,  290,  -10,   -12,    0,  900)
        next_coordinate_linear(  15, -185,  290,   15,   -12, -179, 1200)
        next_coordinate_linear(  15, -185,   75,   10,   -12, -187,  900)       #15  -12  -185  80  800
        next_coordinate_linear(-120, -185,   70,   10,   -12, -189,  300)       #15  -12  -185  80
        next_coordinate_linear(-120, -185,  400,   10,   -12, -179,  900)       #800
    job = 0
    feedback_commands_bt.send("Picked up")


while True:                                                                                 #Start a forever loop
    new_task = feedback_commands_bt.wait_new()                                              #Wait for a new incoming bluetooth command on this channel TODO try to use %s for shorter code
    print(new_task)                                                                         #Print this feedback line when debugging
    if   new_task == "Scissor standard to zone 110": job = 1                                #Set the job to 1 (Moving from scissor to storage place 1)
    elif new_task == "Scissor standard to zone 111": job = 2                                #Set the job to 2 (Moving from scissor to storage place 2)
    elif new_task == "Scissor standard to zone 112": job = 3                                #Set the job to 3 (Moving from scissor to storage place 3)
    elif new_task == "Scissor standard to zone 113": job = 4                                #Set the job to 4 (Moving from scissor to storage place 4)
    elif new_task == "Scissor standard to zone 114": job = 5                                #Set the job to 5 (Moving from scissor to storage place 5)
    elif new_task == "Scissor standard to zone 115": job = 6                                #Set the job to 6 (Moving from scissor to storage place 6)
    elif new_task == "Zone 110 to scissor standard": job = 101                              #Set the job to 101 (Moving storage place 1 to the scissor)
    elif new_task == "Zone 111 to scissor standard": job = 102                              #Set the job to 102 (Moving storage place 2 to the scissor)
    elif new_task == "Zone 112 to scissor standard": job = 103                              #Set the job to 103 (Moving storage place 3 to the scissor)
    elif new_task == "Zone 113 to scissor standard": job = 104                              #Set the job to 104 (Moving storage place 4 to the scissor)
    elif new_task == "Zone 114 to scissor standard": job = 105                              #Set the job to 105 (Moving storage place 5 to the scissor)
    elif new_task == "Zone 115 to scissor standard": job = 106                              #Set the job to 106 (Moving storage place 6 to the scissor)
    elif new_task == "Emergency stop pushed":        emergency_stop = True                  #Set the Emergency state
    elif new_task == "Emergency stop reset":         emergency_stop = False                 #Reset the Emergency state
    elif new_task == "Scissor is up":                scissor_up     = True                  #Set the global variable scissor position up as False (scissor is down)
    elif new_task == "Scissor is down":              scissor_up     = False                 #Set the global variable scissor position up as True
    elif "Adjust J1: " in new_task:                                                         #Check if the string is in the new task string
        adj_val = new_task.split(": ")                                                      #Split the incoming command and save the results in a list
        esp_adj[0] = int(adj_val[1]) * yaw_base_gear                                        #Read the second argument of the list, it contains the new angle adjustment, save it in the global variable
    elif "Adjust J2: " in new_task:                                                         #Check if the string is in the new task string
        adj_val = new_task.split(": ")                                                      #Split the incoming command and save the results in a list
        esp_adj[1] = - int(adj_val[1]) * pitch_base_gear                                    #Read the second argument of the list, it contains the new angle adjustment, save it in the global variable
    elif "Adjust J3: " in new_task:                                                         #Check if the string is in the new task string
        adj_val = new_task.split(": ")                                                      #Split the incoming command and save the results in a list
        esp_adj[2] = int(adj_val[1]) * pitch_arm_gear                                       #Read the second argument of the list, it contains the new angle adjustment, save it in the global variable
    elif "Adjust J4: " in new_task:                                                         #Check if the string is in the new task string
        adj_val = new_task.split(": ")                                                      #Split the incoming command and save the results in a list
        esp_adj[3] = - int(adj_val[1]) * roll_arm_gear                                      #Read the second argument of the list, it contains the new angle adjustment, save it in the global variable
    elif "Adjust J5: " in new_task:                                                         #Check if the string is in the new task string
        adj_val = new_task.split(": ")                                                      #Split the incoming command and save the results in a list
        esp_adj[4] = - int(adj_val[1]) * yaw_arm_gear                                       #Read the second argument of the list, it contains the new angle adjustment, save it in the global variable
    elif "Adjust J6: " in new_task:                                                         #Check if the string is in the new task string
        adj_val = new_task.split(": ")                                                      #Split the incoming command and save the results in a list
        esp_adj[5] = int(adj_val[1]) * roll_head_gear                                       #Read the second argument of the list, it contains the new angle adjustment, save it in the global variable
    if "Adjust" in new_task:                                                                #Check if the string is in the new task string
        sub_move_all_motors.start()                                                         #If an adjustment angle was in the new message, make all motors move to this new angle. Non-blocking

    ev3.screen.draw_text(4,  2, new_task, text_color=Color.BLACK, background_color=Color.WHITE) #X/Y position for writing on the screen, textstring, text color, background color






##########~~~~~~~~~~EXTRA'S NOT USED IN NORMAL PROGRAM, FOR SAVING PURPOSES~~~~~~~~~~##########                                                                        
ev3.speaker.say("Thank you for watching, subscribe for more Technic Machines")              #The EV3 speaker will read out this textstring


#Measurement control, move lineair for 10centimeter
next_coordinate_linear(200, 10, 400, 0, 0, 0, 700)
wait(15000)
next_coordinate_linear(200, 10, 300, 0, 0, 0, 700)
wait(15000)
next_coordinate_linear(200, 10, 200, 0, 0, 0, 700)
wait(15000)
next_coordinate_linear(300, 10, 200, 0, 0, 0, 700)
wait(15000)
next_coordinate_linear(300, 10, 100, 0, 0, 0, 700)
wait(15000)
next_coordinate_linear(300, 10, 300, 0, 0, 0, 700)
wait(15000)
next_coordinate_linear(150, 10, 300, 0, 0, 0, 700)
wait(1000000)


while True:
    for x in range(6):
        job = x + 1
        while job != 0: continue
    for x in range(6):
        job = x + 101
        while job != 0: continue


while True:
    for x in range(6):
        job = x + 1
        moving_scissor_and_zones()
    for x in range(6):
        job = x + 101
        moving_scissor_and_zones()


#demo going infinite on axis 4 and 6
while True:
    next_coordinate_linear( 260, -30,  430,    0,    0,    0, max_speed)
    next_coordinate_linear( 260, -30,  530,    0,    0,    0, max_speed)
    next_coordinate_linear( 260,  30,  530,    0,    0,    0, max_speed)
    next_coordinate_linear( 260,  30,  430,    0,    0,    0, max_speed)

#small demo for V3
next_coordinate_linear( 260, -100,  445,    0,    0,    0, max_speed)
next_coordinate_linear( 260, -100,  200,    0,   -5,    0, max_speed)
next_coordinate_linear( 460, -100,  200,    0,   -10,   0, max_speed)
next_coordinate_linear( 560, -100,  200,    0,   -10,   0, max_speed)
next_coordinate_linear( 260, -100,  445,    0,    0,    0, max_speed)
next_coordinate_linear( 260, -100,  445,    0,    0,   89, max_speed)
next_coordinate_linear( 260, -100,  445,    0,    0,  -89, max_speed)
next_coordinate_linear( 260, -100,  445,    0,    0,    0, max_speed)
next_coordinate_linear( 260, -100,  445,    0,   89,    0, max_speed)
next_coordinate_linear( 260, -100,  445,  720,   89,    0, max_speed)

#lifting test
while True:
    next_coordinate_linear( 150,  150,   70,    0,  -10,    0, max_speed)
    next_coordinate_linear( 150,  150,  680,    0,  -10,    0, max_speed)
    next_coordinate_linear( 150, -150,  680,    0,  -10,    0, max_speed)
    next_coordinate_linear( 150,  150,  680,    0,  -10,    0, max_speed)

#inf loop for theta 4 + 6
next_coordinate_linear( 225, -345,  400,    0,    0,    0, 1200)
while True:
    next_coordinate_linear( 225,   50,  400,    0,    0,    0, 1200)
    next_coordinate_linear( 225,   50,  500,    0,    0,    0, 1200)
    next_coordinate_linear( 225,  -50,  500,    0,    0,    0, 1200)
    next_coordinate_linear( 225,  -50,  400,    0,    0,    0, 1200)

motor_braking()