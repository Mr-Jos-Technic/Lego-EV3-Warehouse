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

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.
# MIT License: Copyright (c) 2022 Mr Jos

#####################################################################
#####################################################################
##########~~~~~PROGRAM WRITTEN BY JOZUA VAN RAVENHORST~~~~~##########
##########~~~~~~~~~~WAREHOUSE XL: RIGHTSIDE 6DOF~~~~~~~~~~~##########
##########~~~~~~~~~~~~~YOUTUBE CHANNEL: MR JOS~~~~~~~~~~~~~##########
#####################################################################
#####################################################################


##########~~~~~~~~~~HARDWARE CONFIGURATION~~~~~~~~~~##########
ev3 = EV3Brick()
#   Motors definition
roll_head       =   Motor(Port.A)                                                           #Motor for joint 6 (Rolling the head of the wrist)
yaw_base        =   Motor(Port.B)                                                           #Motor for joint 1 (Yaw the whole robot)
green_lights    =   Motor(Port.D)                                                           #Power functions lights connected with a custom cable as a motor to the EV3 brick
#   Sensor definition
touch_yaw_base  =   TouchSensor(Port.S1)                                                    #Touch sensor for homing joint 1
color_roll_head =   ColorSensor(Port.S2)                                                    #Color sensor for homing joint 6


##########~~~~~~~~~~HOMING POSITION ANGLES WHEN SENSOR ACTIVATED~~~~~~~~~~##########


##########~~~~~~~~~~GEARING~~~~~~~~~~##########


##########~~~~~~~~~~MAXIMUM SPEED, MAXIMUM ACCELERATION, MAXIMUM POWER~~~~~~~~~~##########
roll_head.control.limits(1400,3600,100)                                                     #Motor settings [Max speed allowed (deg/sec) / Acceleration (deg/sec²) / Power (%)]
yaw_base.control.limits (1400,1400,100)                                                     #Motor settings [Max speed allowed (deg/sec) / Acceleration (deg/sec²) / Power (%)]


##########~~~~~~~~~~BLUETOOTH SETUP, CLIENT SIDE~~~~~~~~~~##########
client_robot = BluetoothMailboxClient()                                                     #Defining the name for this slave (Client) EV3 brick for sending/receiving bluetooth commands

#TODO this program was written long ago, as one of my first, need to simplify the amount of bluetooth mailboxes
commands_bt_text            = TextMailbox   ('commands text',            client_robot)      #Sending and receiving the bluetooth commands to/from the master Robot EV3 brick on this channel
roll_head_bt_zeroing        = NumericMailbox('zero position roll',       client_robot)      #Receiving the bluetooth commands from the master Robot EV3 brick on this channel
roll_head_bt_num            = NumericMailbox('roll head degree',         client_robot)      #Receiving the bluetooth commands from the master Robot EV3 brick on this channel
roll_head_bt_sp             = NumericMailbox('roll head speed',          client_robot)      #Receiving the bluetooth commands from the master Robot EV3 brick on this channel
roll_head_feedb             = NumericMailbox('roll head feedback',       client_robot)      #Sending   the bluetooth commands to   the master Robot EV3 brick on this channel
yaw_base_bt_zeroing         = NumericMailbox('zero position yaw',        client_robot)      #Receiving the bluetooth commands from the master Robot EV3 brick on this channel
yaw_base_bt_num             = NumericMailbox('yaw base degree',          client_robot)      #Receiving the bluetooth commands from the master Robot EV3 brick on this channel
yaw_base_bt_sp              = NumericMailbox('yaw base speed',           client_robot)      #Receiving the bluetooth commands from the master Robot EV3 brick on this channel
yaw_base_feedb              = NumericMailbox('yaw base feedback',        client_robot)      #Sending   the bluetooth commands to   the master Robot EV3 brick on this channel

feedback_commands_bt        = TextMailbox   ('feedback from 6dof',       client_robot)      #Sending and receiving the bluetooth commands to/from the master Robot EV3 brick on this channel

conv_status_to_robot_mbox   = TextMailbox   ('conveyor update to robot', client_robot)      #Receiving the bluetooth commands from the master Conveyor EV3 brick on this channel
robot_status_to_conv_mbox   = TextMailbox   ('robot update to conveyor', client_robot)      #Sending   the bluetooth commands to   the master Conveyor EV3 brick on this channel


##########~~~~~~~~~~CREATING AND STARTING A TIMER~~~~~~~~~~##########   [Not used]
#timer_movement = StopWatch()                                                               #Creating a timer
#timer_movement.time()                                                                      #Reading  a timer's current value (ms)
#timer_movement.pause()                                                                     #Stopping a timer
#timer_movement.resume()                                                                    #Resuming a timer
#timer_movement.reset()  


##########~~~~~~~~~~BUILDING GLOBAL VARIABLES~~~~~~~~~~##########
old_task        =   "no task"                                                               #Used to save the last task that was received from the master Conveyor brick
old_feedback    =   "no feedback"                                                           #Used to save the last task that was received from the master Robot brick
light_mode      =   "startup"                                                               #Used to see in what mode the robot is, and accordingly set/flash the power function lights


##########~~~~~~~~~~BRICK STARTUP SETTINGS~~~~~~~~~~##########
ev3.speaker.set_volume(volume=90, which='_all_')                                            #Set the volume for all sounds (speaking and beeps etc)
ev3.speaker.set_speech_options(language='en', voice='m7', speed=None, pitch=None)           #Select speaking language, and a voice (male/female)
small_font  = Font(size=6)                                                                  #Creating a font with  6 pixel height for text on screen    [Not used]
normal_font = Font(size=10)                                                                 #Creating a font with 10 pixel height for text on screen
big_font    = Font(size=16)                                                                 #Creating a font with 16 pixel height for text on screen    [Not used]
ev3.screen.set_font(normal_font)                                                            #Choose a preset font for writing next texts
ev3.screen.clear()                                                                          #Make the screen empty (all pixels white)
ev3.speaker.beep()                                                                          #Brick will make a beep sound 1 time
#ev3.light.on(Color.GREEN)                                                                  #Turns the green lights on the brick on                     [Not used, example]
ev3.light.off()                                                                             #Turn the lights off on the brick
#ev3.screen.draw_text(4, 2, "", text_color=Color.BLACK, background_color=Color.WHITE)       #X/Y position for writing on the screen, textstring, text color, background color   [Example]


##########~~~~~~~~~~CREATING A FILE THAT IS SAVED OFFLINE~~~~~~~~~~##########   [Not used]
#Not used right now, it can be used to store coordinates of positions to go to, and call them back when restarting the program
#os.remove("saveddata.txt")                                                                 #Removing the wms file for whatever reason you might need to delete it  [KEEP # for normal operation!]
#create_file = open("saveddata.txt", "a")                                                   #Create a file if it does not exist and open it, if it does exist just open it
#create_file.write("")                                                                      #Write "Nothing" to the file to have atleast 1 line in the file
#create_file.close() 


##########~~~~~~~~~~DEFINE SUB-ROUTINES [FUNCTIONS] FOR THE OPERATION OF THE ROBOT~~~~~~~~~~##########
##########~~~~~~~~~~BLUETOOTH SENDING COMMUNICATION COMMANDS~~~~~~~~~~##########
def angle_feedback():                                                                       #The master Robot EV3 brick will receive with this function all motor angle feedback Bluetooth commands
    while True:                                                                             #Start a forever loop
        yaw_base_feedb.send(yaw_base.angle())                                               #The message is send by bluetooth communication to the master robot brick
        roll_head_feedb.send(roll_head.angle())                                             #The message is send by bluetooth communication to the master conveyor brick
        wait(250)                                                                           #Allow the receiver some time to read the previous message first


def move_yaw_base():                                                                        #The master Robot EV3 brick will send in this function all joint 1 motor angles by Bluetooth commands
    global yaw_base_bt_num                                                                  #Using these global variables in this local function (if not defined to be global, it will make a local variable) TODO is this one needed?
    global yaw_base

    while True:                                                                             #Start a forever loop
        yaw_base_bt_num.wait_new()                                                          #Wait for a new message to be received by bluetooth from the Master Robot EV3 brick
        yaw_base.run_target(yaw_base_bt_sp.read(), yaw_base_bt_num.read(), wait=False)      #Read the last message received by bluetooth from the Master Robot EV3 brick on this channel


def move_roll_head():                                                                       #The master Robot EV3 brick will send in this function all joint 6 motor angles by Bluetooth commands
    global roll_head_bt_num                                                                 #Using these global variables in this local function (if not defined to be global, it will make a local variable) TODO is this one needed?
    global roll_head

    while True:                                                                             #Start a forever loop
        roll_head_bt_num.wait_new()                                                         #Wait for a new message to be received by bluetooth from the Master Robot EV3 brick
        roll_head.run_target(roll_head_bt_sp.read(), roll_head_bt_num.read(), wait=False)   #Read the last message received by bluetooth from the Master Robot EV3 brick on this channel


def control_check():
    global old_task                                                                         #Using these global variables in this local function (if not defined to be global, it will make a local variable)
    global old_feedback
    global light_mode

    if yaw_base.control.done() == True and roll_head.control.done() == True:                #Check if both motors have finished their movement and reached the set motor angles
        commands_bt_text.send("No movement")                                                #If finished, send to the Master Robot EV3 brick that there is no more movement
    else: commands_bt_text.send("Moving")                                                   #If not finished, send to the Master Robot EV3 brick that there is still movement
    wait(100)                                                                               #Allow the receiver some time to read the previous message first
    new_task = conv_status_to_robot_mbox.read()                                             #Read the last message received by bluetooth from the Master Conveyor EV3 brick on this channel
    if new_task != old_task:                                                                #Compare the received message, if it is a new message
        feedback_commands_bt.send(new_task)                                                 #Send the new message to the Master Robot EV3 brick
        if new_task != "emergency stop pushed" and new_task != "emergency stop reset": light_mode = "moving"    #Change the lights mode
        old_task = new_task                                                                 #Set the new message as old message
        print(new_task)                                                                     #Print this feedback line when debugging
        wait(50)                                                                            #Allow the receiver some time to read the previous message first
    new_feedback = feedback_commands_bt.read()                                              #Read the last message received by bluetooth from the Master Robot EV3 brick on this channel
    if new_feedback != old_feedback:                                                        #Compare the received message, if it is a new message
        print(new_feedback)                                                                 #Print this feedback line when debugging
        if new_feedback == "ready": light_mode = "off"                                      #Change the lights mode
        robot_status_to_conv_mbox.send(new_feedback)                                        #Send the new message to the Master Conveyor EV3 brick
        old_feedback = new_feedback                                                         #Set the new message as old message
        wait(50)                                                                            #Allow the receiver some time to read the previous message first


##########~~~~~~~~~~DEFINE SUB-ROUTINES~~~~~~~~~~##########
def light_operation():                                                                      #This function will turn the modified Power Functions lights on and off
    global light_mode                                                                       #Using this global variable in this local function (if not defined to be global, it will make a local variable)

    while True:                                                                             #Start a forever loop
        wait(200)                                                                           #Allow this EV3 brick some time to perform other tasks more precisely
        if light_mode == "startup":                                                         #If the light mode is set to startup
            while light_mode == "startup":                                                  #While the mode remains
                for x in range(0, 105, 5):                                                  #In a for loop go from 0 to 100, in steps of 5
                    green_lights.dc(x)                                                      #Set the lights intensity with the for loop value (%)
                    wait(100)                                                               #Wait between changing intensity (10Hz)
                wait(150)
                for x in range(95, -5, -5):                                                 #In a for loop go from 95 to 0, in steps of 5
                    green_lights.dc(x)                                                      #Set the lights intensity with the for loop value (%)
                    wait(100)                                                               #Wait between changing intensity (10Hz)
                wait(150)
        elif light_mode == "homing":                                                        #If the light mode is set to homing
            while light_mode == "homing":                                                   #While the mode remains flash the lights
                green_lights.dc(100)                                                        #Keep the lights on (100%)
                wait(250)                                                                   #Wait between changing intensity (2Hz)
                green_lights.dc(0)                                                          #Turn the lights off (0%)
                wait(250)                                                                   #Wait between changing intensity (2Hz)
        elif light_mode == "moving":                                                        #If the light mode is set to moving
            for x in range(0, 105, 5):                                                      #In a for loop go from 0 to 100, in steps of 5
                    green_lights.dc(x)                                                      #Set the lights intensity with the for loop value (%)
                    wait(100)                                                               #Wait between changing intensity (10Hz)
            while light_mode == "moving": wait(200)                                         #Keep the lights on (100%)
        else: green_lights.dc(0)                                                            #If the light mode is not set to any before, turn the lights off (0%)


##########~~~~~~~~~~CREATING MULTITHREADS~~~~~~~~~~##########
sub_light_operation = Thread(target=light_operation)                                        #This creates a thread, made from a previously defined function. No arguments can be given
sub_yaw_base        = Thread(target=move_yaw_base)
sub_roll_head       = Thread(target=move_roll_head)
sub_angle_feedback  = Thread(target=angle_feedback)


##########~~~~~~~~~~PROGRAM STARTING~~~~~~~~~~##########
sub_light_operation.start()                                                                 #This starts the loop thread that controls the power function lights. Non-blocking


##########~~~~~~~~~~CONNECT TO THE MASTER EV3'S~~~~~~~~~~##########
##########ALWAYS START THE SERVER-BRICKS FIRST. THEN START THIS SLAVE-BRICK, OR IT WILL TIMEOUT IF IT CAN NOT CONNECT TO THE MASTER BRICK BY BLUETOOTH ##########
client_robot.connect('left6dof')                                                            #Tries to connect to the bluetooth device with the arguments name, if it can't it will stop this program
client_robot.connect('inputconveyors')                                                      #Argument is the name of the master brick this slave will try to connect with, by default the name is "ev3dev", if you rename it, change it here    

light_mode = "homing"


##########~~~~~~~~~~STARTUP ALL BLUETOOTH RX AND TX~~~~~~~~~~##########
sub_yaw_base.start()                                                                        #This starts the loop thread that controls the base yaw  (joint 1). Non-blocking
sub_roll_head.start()                                                                       #This starts the loop thread that controls the head roll (joint 6). Non-blocking
sub_angle_feedback.start()                                                                  #This starts the loop thread that sends all the bluetooth communication to the Master Robot EV3 brick. Non-blocking

while commands_bt_text.read() != 'Initiate yaw base': control_check()                       #Wait for the start homing command from the Master Robot EV3 brick by bluetooth


##########~~~~~~~~~~EV3 HAS INCREMENTAL ENCODERS IN IT'S MOTORS, SO IT'S NEEDED TO CALIBRATE EVERY STARTUP~~~~~~~~~~##########
if touch_yaw_base.pressed() == True:                                                        #If the touch sensor is already pressed
    while touch_yaw_base.pressed() == True: yaw_base.run(-200)                              #Turn joint 1 clockwise at a constant speed until the sensor is released
    wait(500)                                                                               #Keep turning 500ms to be far enough away from the sensor

yaw_base.run(200)                                                                           #Turn joint 1 counter-clockwise at a constant speed
while touch_yaw_base.pressed() != True: continue                                            #While the touch sensor is not pressed, keep turning joint 1 counter-clockwise
yaw_base.hold()                                                                             #Stop the base motor and hold the motor angle fixed
yaw_base.reset_angle(yaw_base_bt_zeroing.read())                                            #Read the homing angle from the bluetooth command and reset the base motor angle accordingly

commands_bt_text.send('Initiated yaw base')                                                 #The message is added to the bluetooth communication waiting list for the master Robot brick

while commands_bt_text.read() != 'Initiate roll head': wait(100)                            #Wait for a specific bluetooth command

if color_roll_head.color() == Color.RED:                                                    #Check the color seen by the homing color sensor for joint 6, if it is red
    while color_roll_head.color() == Color.RED: roll_head.run(1400)                         #While the color seen is red, turn the head clockwise at a constant high speed
    wait(200)                                                                               #Keep turning 200ms to be far enough away from the sensor

while True:                                                                                 #Start a forever loop
    while color_roll_head.color() != Color.RED: roll_head.run(-600)                         #Start a loop that turns the head counter-clockwise at a constant speed until the sensor sees the color red
    roll_head.hold()                                                                        #Stop the head motor and hold the motor angle fixed
    wait(50)                                                                                #Wait 50ms
    if color_roll_head.color() != Color.RED: continue                                       #Check the color again, if not red, restart the loop (motor will start again)
    wait(50)                                                                                #Wait 50ms
    if color_roll_head.color() != Color.RED: continue                                       #Check the color again, if not red, restart the loop (motor will start again)
    wait(50)                                                                                #Wait 50ms
    if color_roll_head.color() != Color.RED: continue                                       #Check the color again, if not red, restart the loop (motor will start again)
    wait(50)                                                                                #Wait 50ms
    if color_roll_head.color() != Color.RED: continue                                       #Check the color again, if not red, restart the loop (motor will start again)
    break                                                                                   #If the scanned color was 5 times in row red, the homing was succesfull, break out of the loop

roll_head.reset_angle(roll_head_bt_zeroing.read())                                          #Read the homing angle from the bluetooth command and reset the head motor angle accordingly

commands_bt_text.send('Initiated roll head')                                                #The message is added to the bluetooth communication waiting list for the master Robot brick

wait(500)                                                                                   #Wait 500ms TODO check why this is needed
light_mode = "moving"                                                                       #Set the light mode to be moving


##########~~~~~~~~~~PROGRAM RUNNING THE SLAVE EV3 ROBOT BRICK~~~~~~~~~~##########
while True:                                                                                 #Start a forever loop
    control_check()                                                                         #This calls the function for checking new messages. Blocking


##########~~~~~~~~~~EXTRA'S NOT USED IN NORMAL PROGRAM, FOR SAVING PURPOSES~~~~~~~~~~##########                                                                        
ev3.speaker.say("Thank you for watching, subscribe for more Technic Machines")              #The EV3 speaker will read out this textstring
