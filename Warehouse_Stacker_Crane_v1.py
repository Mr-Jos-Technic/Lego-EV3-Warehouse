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
##########~~~~~~~~~~~WAREHOUSE XL: STACKER CRANE~~~~~~~~~~~##########
##########~~~~~~~~~~~~~YOUTUBE CHANNEL: MR JOS~~~~~~~~~~~~~##########
#####################################################################
#####################################################################


##########~~~~~~~~~~HARDWARE CONFIGURATION~~~~~~~~~~##########
ev3 = EV3Brick()
#   Motors definition
tele_fork_motor         =   Motor(Port.A)                                                   #Telescopic fork motor
lift_platform           =   Motor(Port.C, positive_direction=Direction.COUNTERCLOCKWISE)    #Lifting the platform
driving_motor           =   Motor(Port.D)                                                   #Driving the stacker crane on the rail
#   Sensor definition
tele_fork_clr           =   ColorSensor(Port.S1)                                            #Colorsensor used for finding the center position of the telescopic fork
touch_base              =   TouchSensor(Port.S2)                                            #Touchsensor used for homing the driving on the rail and the platform lowest point
rail_clr                =   ColorSensor(Port.S3)                                            #Colorsensor used for fine positioning the driving on the rail
rack_clr                =   ColorSensor(Port.S4)                                            #Colorsensor used for calibrating the descend and ascend positions for the platform


##########~~~~~~~~~~HOMING POSITION ANGLES WHEN SENSOR ACTIVATED~~~~~~~~~~##########
fork_homing_black       =    230                                                            #Motor angle that will be used when centering the telescopic fork from the black side
fork_homing_yellow      =    300                                                            #Motor angle that will be used when centering the telescopic fork from the yellow side
adjust_side             =     50                                                            #Motor angle adjustment when extending the fork back to the same side it came from
drive_homing_adjust     =      0                                                            #Motor angle that will be used when resetting the homing position


##########~~~~~~~~~~GEARING~~~~~~~~~~##########
max_speed_lift          =   1000                                                            #Maximal speed (deg/sec) that the lifting motor is allowed to rotate at
max_speed_drive         =   1000                                                            #Maximal speed (deg/sec) that the driving motor is allowed to rotate at
max_speed_fork          =   1400                                                            #Maximal speed (deg/sec) that the telescopic fork motor is allowed to rotate at
fork_ext_coord          =   2525                                                            #Motor angle to extend the telescopic fork from centered to one side
dropoff_height          =    160                                                            #Motor angle to add to the pickup location for dropoff height
remote_height_adjust    =      0                                                            #Adjustment for the lifting height (-10 <= Value <= 10), adjusted by manual control on the touchscreen TODO add to lifting height
remote_speed_adjust     =      1                                                            #Multiplier for the conveyor speed (0 < Value <= 1), adjusted by manual control on the touchscreen


##########~~~~~~~~~~MAXIMUM SPEED, MAXIMUM ACCELERATION, MAXIMUM POWER~~~~~~~~~~##########
tele_fork_motor.control.limits(1400,  800, 100)                                             #Motor settings [Max speed allowed (deg/sec) / Acceleration (deg/sec²) / Power (%)]
lift_platform.control.limits  (1000,  500, 100)                                             #Motor settings [Max speed allowed (deg/sec) / Acceleration (deg/sec²) / Power (%)]
driving_motor.control.limits  (1000,  400, 100)                                             #Motor settings [Max speed allowed (deg/sec) / Acceleration (deg/sec²) / Power (%)]


##########~~~~~~~~~~BLUETOOTH SETUP, SERVER SIDE~~~~~~~~~~##########
client_crane = BluetoothMailboxClient()                                                     #Defining the name for this slave (Client) EV3 brick for sending/receiving bluetooth commands

conv_status_to_crane_mbox   = TextMailbox('conveyor update to crane' , client_crane)        #Receiving the bluetooth commands from the master EV3 brick on this channel
crane_status_to_conv_mbox   = TextMailbox('crane update to conveyor' , client_crane)        #Sending   the bluetooth commands to   the master EV3 brick on this channel


##########~~~~~~~~~~CREATING AND STARTING A TIMER~~~~~~~~~~##########
#timer_movement = StopWatch()                                                               #Creating a timer
#timer_movement.time()                                                                      #Reading  a timer's current value (ms)
#timer_movement.pause()                                                                     #Stopping a timer
#timer_movement.resume()                                                                    #Resuming a timer
#timer_movement.reset()                                                                     #Putting  a timer back at 0, if not stopped it will just keep running but start from 0 again.
timer_timeout = StopWatch()                                                                 #Creating the timer to see if there is a problem with the centering of the telescopic fork
timer_timeout.reset()                                                                       #Putting the timer back at 0, if not stopped it will just keep running but start from 0 again.


##########~~~~~~~~~~BUILDING GLOBAL VARIABLES~~~~~~~~~~##########
rack_coord              =   [210, 806, 1297, 1796, 2296, 2795]                              #List with distances from the homing points to [0] = Chain conveyor pos, [1-5] = Rack numbers starting from front [208, 795, 1295, 1795, 2295, 2795] 
ascending_coord         =   [65]                                                            #List that will be later filled more with all positions when going up with the platform
descending_coord        =   [55]                                                            #List that will be later filled more with all positions when going down with the platform
fork_timeout_time       =   4000                                                            #Time in ms before triggering an error when trying to home the telescopic fork
#Locations are shifted by 1 in the stacker crane program, as position 1 in the lists is the chain conveyor position for pickup and dropoff
rack_pos   = [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5]    #Depending on the target location drive to a rack number (location 0,1,2,...,59)
height_pos = [0, 1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 5, 6]    #Depending on the target location lift to a height number (location 0,1,2,...,59)
side_pos   = [0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2]    #Depending on the target location extend the fork to the left or right(location 0,1,2,...,59)

emergency_stop          =   False                                                           #Variable to see if the emergency stop has been pushed / reset
comm_list               =   []                                                              #List with all commands that still need to be send to the master conveyor brick
crane_task              =   "None"                                                          #Variable that holds the next task to be done
error_homing_fork       =   False                                                           #Variable to see if the fork homing sensor has triggered an error / has been reset
error_driving_pos       =   False                                                           #Variable to see if the driving sensor has triggered an error / has been reset
correct_dropoff         =   True                                                            #Variable that gets set True if the output chain conveyor sensor has confirmed the dropoff of the box
last_homing_left        =   False                                                           #Variable to know at what side the fork was extended the last time


##########~~~~~~~~~~BRICK STARTUP SETTINGS~~~~~~~~~~##########
ev3.speaker.set_volume(volume=90, which='_all_')                                            #Set the volume for all sounds (speaking and beeps etc)
ev3.speaker.set_speech_options(language='en', voice='m7', speed=None, pitch=None)           #Select speaking language, and a voice (male/female)
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
#create_file.close()                                                                        #Close the file again, to be able to call it later again


##########~~~~~~~~~~DEFINE SUB-ROUTINES~~~~~~~~~~##########
##########~~~~~~~~~~BLUETOOTH SENDING COMMUNICATION COMMANDS~~~~~~~~~~##########
def communication_control():                                                                #The master EV3 brick will receive with this function all Bluetooth commands
    global comm_list                                                                        #TODO is this global needed? BT lists are not global but work in master
    
    wait(500)                                                                               #Give the master 1second time to start up the bluetooth receiving function
    while True:                                                                             #Start a forever loop
        wait(300)                                                                           #Allow the receiver some time to read the previous message first
        if len(comm_list) > 0:                                                              #Check if there is a command in the master conveyor communication list
            print("outgoing message: %s."%comm_list[0])                                     #Print this feedback line when debugging
            crane_status_to_conv_mbox.send(comm_list[0])                                    #The message is send by bluetooth communication to the master conveyor brick
            wait(50)                                                                        #TODO is this needed?
            del comm_list[0]                                                                #If the message is send, delete it from the communication list


##########~~~~~~~~~~DEFINE SUB-ROUTINES [FUNCTIONS] FOR THE OPERATION OF THE WAREHOUSE~~~~~~~~~~##########
def check_emergency_stop():                                                                 #This function checks if there is a need to stop the current/next movement
    global emergency_stop                                                                   #Using this global variable in this local function (if not defined to be global, it will make a local variable)

    while emergency_stop == True:                                                           #If there is an emergency stop, start this loop until the emergency stop has been reset
        ev3.light.on(Color.RED)                                                             #Turn the red lights on the EV3 brick on (2Hz)
        wait(250)                                                                           #Wait 250ms
        ev3.light.off()                                                                     #Turn all the lights on the EV3 brick off
        wait(250)                                                                           #Wait 250ms


def crane_control():                                                                        #A loop that checks if the stacker crane can do a job
    global crane_task                                                                       #Using these global variables in this local function (if not defined to be global, it will make a local variable)
    global correct_dropoff

    next_task = ""                                                                          #Defining a local variable
    while True:                                                                             #Start a forever loop
        check_emergency_stop()                                                              #Check if the movement is allowed to start
        while correct_dropoff == False: wait(100)                                           #If the last task was a dropoff, wait for the confirmation of a good dropoff
        if   "Store at" in crane_task:                                                      #Check if a stacker crane task has been set
            next_task = crane_task.split(",")                                               #Split the incoming command and save the results in a list
            store_box(int(next_task[1]) + 1)                                                #Read the second argument of the list, it contains a location, start with it a function
        elif "Retrieve at" in crane_task:                                                   #Check if a stacker crane task has been set
            next_task = crane_task.split(",")                                               #Split the incoming command and save the results in a list
            retrieve_box(int(next_task[1]) + 1)                                             #Read the second argument of the list, it contains a location, start with it a function
        elif "Move between" in crane_task:                                                  #Check if a stacker crane task has been set
            next_task = crane_task.split(",")                                               #Split the incoming command and save the results in a list
            move_box_crane(int(next_task[1]) + 1, int(next_task[2]) + 1)                    #Read the second and third argument of the list, they contain a location, start with them a function
        elif "Drive to" in crane_task:                                                      #Check if a stacker crane task has been set
            next_task = crane_task.split(",")                                               #Split the incoming command and save the results in a list
            manual_driving(int(next_task[1]) + 1)                                           #Read the second argument of the list, it contains a location, start with it a function
        elif "Startup" in crane_task:                                                       #Check if a stacker crane task has been set
            next_task = crane_task.split(",")                                               #Split the incoming command and save the results in a list
            startup_box(int(next_task[1]) + 1)                                              #Read the second argument of the list, it contains a location, start with it a function


def store_box(store_pos):                                                                   #This function is used to take a box from the input chain conveyor and store it in the high bay racks (location 105 -> 0-59)
    global crane_task                                                                       #Using this global variable in this local function (if not defined to be global, it will make a local variable)
    crane_task = ""                                                                         #Clear the global variable that had the task
    drive_to_pos(0, 0, "Pickup", 1)                                                         #Call the function to drive to a certain location and perform a fork movement
    comm_list.append("Picked up")                                                           #The message is added to the bluetooth communication waiting list for the master conveyor brick
    drive_to_pos(rack_pos[store_pos], height_pos[store_pos] , "Dropoff", side_pos[store_pos])   #Call the function to drive to a certain location and perform a fork movement
    comm_list.append("Dropped off")                                                         #The message is added to the bluetooth communication waiting list for the master conveyor brick


def retrieve_box(retrieve_pos):                                                             #This function is used to take a box out of the high bay racks and bring it to the output chain conveyor (locations 0-59 -> 101)
    global crane_task                                                                       #Using these global variables in this local function (if not defined to be global, it will make a local variable)
    global correct_dropoff
    crane_task = ""                                                                         #Clear the global variable that had the task
    correct_dropoff = False                                                                 #If a box needs to be taken out of the rack to the conveyor, a feedback will come to confirm good dropoff, resetting the good dropoff here
    print(retrieve_pos)                                                                     #Print this feedback line when debugging
    drive_to_pos(rack_pos[retrieve_pos], height_pos[retrieve_pos], "Pickup", side_pos[retrieve_pos])    #Call the function to drive to a certain location and perform a fork movement
    comm_list.append("Picked up")                                                           #The message is added to the bluetooth communication waiting list for the master conveyor brick
    drive_to_pos(0, 0, "Dropoff", 2)                                                        #Call the function to drive to a certain location and perform a fork movement
    comm_list.append("Dropped off")                                                         #The message is added to the bluetooth communication waiting list for the master conveyor brick


def move_box_crane(retrieve_pos, store_pos):                                                #This function is used to take a box out of the high bay racks and store it on another place in the racks (locations 0-59)
    global crane_task                                                                       #Using this global variable in this local function (if not defined to be global, it will make a local variable)
    crane_task = ""                                                                         #Clear the global variable that had the task
    drive_to_pos(rack_pos[retrieve_pos], height_pos[retrieve_pos], "Pickup", side_pos[retrieve_pos])    #Call the function to drive to a certain location and perform a fork movement
    comm_list.append("Picked up")                                                           #The message is added to the bluetooth communication waiting list for the master conveyor brick
    drive_to_pos(rack_pos[store_pos], height_pos[store_pos] , "Dropoff", side_pos[store_pos])   #Call the function to drive to a certain location and perform a fork movement
    comm_list.append("Dropped off")                                                         #The message is added to the bluetooth communication waiting list for the master conveyor brick


def manual_driving(driving_pos):                                                            #This function is used to drive to a manual selected position in the high bay racks and not extend the fork
    global crane_task                                                                       #Using this global variable in this local function (if not defined to be global, it will make a local variable)
    crane_task = ""                                                                         #Clear the global variable that had the task
    comm_list.append("Started moving")                                                      #The message is added to the bluetooth communication waiting list for the master conveyor brick
    drive_to_pos(rack_pos[driving_pos], height_pos[driving_pos], "Pickup", 0)               #Call the function to drive to a certain location and perform a fork movement
    comm_list.append("Ready")                                                               #The message is added to the bluetooth communication waiting list for the master conveyor brick


def startup_box(store_pos):                                                                 #This function is used store a box that was on the stacker crane when homing, in the high bay racks (location 100 -> 0-59)
    global crane_task                                                                       #Using this global variable in this local function (if not defined to be global, it will make a local variable)
    crane_task = ""                                                                         #Clear the global variable that had the task
    drive_to_pos(rack_pos[store_pos], height_pos[store_pos] , "Dropoff", side_pos[store_pos])   #Call the function to drive to a certain location and perform a fork movement
    comm_list.append("Dropped off")                                                         #The message is added to the bluetooth communication waiting list for the master conveyor brick


def drive_to_pos(rack_nr, height_nr, pos_mode, fork_mode):                                  #This function is used to drive to a certain location and perform a fork movement
    global last_homing_left                                                                 #Using these global variables in this local function (if not defined to be global, it will make a local variable)
    global emergency_stop
    global error_homing_fork
    global error_driving_pos
    
    check_emergency_stop()                                                                  #Check if the movement is allowed to start
    if rack_coord[rack_nr] > driving_motor.angle(): direction = 1                           #Check if the stacker crane is currently in front of the location it will go to next, or behind it
    else: direction = -1                                                                    #If behind set the local variable to negative
    driving_motor.run_target(max_speed_drive * remote_speed_adjust, rack_coord[rack_nr], then=Stop.COAST, wait=False)   #Send the driving motor run to the target command, don't wait for finishing
    current_height = lift_platform.angle()                                                  #Save the current platform lifting motor angle in a local variable
    if   pos_mode == "Pickup":                                                              #Depending on the task, picking up or dropping off a box, select the lifting height
        if   current_height < ascending_coord[height_nr]:  lift_angle = ascending_coord[height_nr]  #Depending on the current position of the lifting platform, select the destination motor angle
        elif current_height > descending_coord[height_nr]: lift_angle = descending_coord[height_nr] #Depending on the current position of the lifting platform, select the destination motor angle
    elif pos_mode == "Dropoff":                                                             #Depending on the task, picking up or dropping off a box, select the lifting height
        if   current_height < ascending_coord[height_nr] + dropoff_height:                  #Depending on the current position of the lifting platform, select the destination motor angle
            if height_nr <= 6:                             lift_angle = ascending_coord[height_nr]  +  dropoff_height   #If the location is under floor 6, select the destination motor angle
            else:                                          lift_angle = ascending_coord[height_nr]  + (dropoff_height / 3 * 2)  #If the location is above floor 6, select the destination motor angle [Not used, used to be 5]
        elif current_height > descending_coord[height_nr] + dropoff_height:                 #Depending on the current position of the lifting platform, select the destination motor angle
            if height_nr <= 6:                             lift_angle = descending_coord[height_nr] +  dropoff_height   #If the location is under floor 6, select the destination motor angle
            else:                                          lift_angle = descending_coord[height_nr] + (dropoff_height / 3 * 2)  #If the location is above floor 6, select the destination motor angle [Not used, used to be 5]

    lift_platform.run_target(max_speed_lift * remote_speed_adjust, lift_angle , then=Stop.HOLD, wait=False) #Send the platform lifting motor run to the target command, don't wait for finishing

    while math.fabs(rack_coord[rack_nr] - driving_motor.angle()) > 100 and driving_motor.control.done() != True:    #Wait for the driving motor to be within 100°
        if emergency_stop == True:                                                          #If during the driving an Emergency state happens
            driving_motor.run(0)                                                            #Set the driving speed to 0degrees/second (This allows a set deceleration so the stacker crane does not fall over from a hard brake)
            lift_platform.run(0)                                                            #Set the lifting speed to 0degrees/second (This allows a set deceleration so the box does not fall off from a hard brake)
            check_emergency_stop()                                                          #Check if the movement is allowed to restart
            driving_motor.run_target(max_speed_drive * remote_speed_adjust, rack_coord[rack_nr], then=Stop.COAST, wait=False)   #Resend the driving motor run to the target command, don't wait for finishing
            lift_platform.run_target(max_speed_lift * remote_speed_adjust, lift_angle , then=Stop.HOLD, wait=False) #Resend the platform lifting motor run to the target command, don't wait for finishing
    if math.fabs(rack_coord[rack_nr] - driving_motor.angle()) > 5:                          #If the crane is already positioned in the correct place do not correct (staying in same row)
        while True:                                                                         #Start a forever loop
            if driving_motor.control.done() == True and error_driving_pos == False:         #Check if the driving is finished and that there is no error
                comm_list.append("Crane positioning error")                                 #The message is added to the bluetooth communication waiting list for the master conveyor brick
                error_driving_pos = True                                                    #The driving motor has stopped, but not reached its position yet, set an error
                while error_driving_pos == True or emergency_stop == True:                  #Start a loop that checks if the error has been reset and that there is no emergency state
                    if emergency_stop == True and lift_platform.control.done() == False:    #If there is an emergency state and the lifting motor has not finished moving yet
                        lift_platform.run(0)                                                #Set the lifting speed to 0degrees/second (This allows a set deceleration so the box does not fall off from a hard brake)
                        check_emergency_stop()                                              #Check if the movement is allowed to restart
                        lift_platform.run_target(max_speed_lift * remote_speed_adjust, lift_angle , then=Stop.HOLD, wait=False) #Send the platform lifting motor run to the target command, don't wait for finishing
                driving_motor.run(50* direction)                                            #If the error is resolved, restart the driving in the same direction

            current_rail_clr = rail_clr.color()                                             #Look at the rail color and save it in a local variable
            if   current_rail_clr == Color.YELLOW: continue                                 #If the rail color is yellow (standard) then restart the loop
            elif current_rail_clr == Color.BLACK:                                           #If the rail color is black then the crane comes from the front
                if rack_nr == 0: driving_motor.run_angle(100,   2, then=Stop.HOLD, wait=True)   #If the location is a chain conveyor, send the driving motor the fine adjustment run to the target command, wait for finishing
                else:            driving_motor.run_angle(100,  10, then=Stop.HOLD, wait=True)   #If the location is a rack  location, send the driving motor the fine adjustment run to the target command, wait for finishing
                break                                                                       #Break out of the forever loop
            elif current_rail_clr == Color.RED:                                             #If the rail color is red then the crane comes from the back
                if rack_nr == 0: driving_motor.run_angle(300, -55, then=Stop.HOLD, wait=True)   #If the location is a chain conveyor, send the driving motor the fine adjustment run to the target command, wait for finishing
                else:            driving_motor.run_angle(300, -40, then=Stop.HOLD, wait=True)   #If the location is a rack  location, send the driving motor the fine adjustment run to the target command, wait for finishing
                break                                                                       #Break out of the forever loop
        driving_motor.reset_angle(rack_coord[rack_nr])                                      #Reset the driving motor angle to the theoretical value to counter slipping of the drive wheel at long/short term

    while lift_platform.control.done() != True:                                             #Start a loop that wait until the platform lifting is finished
        if emergency_stop == True:                                                          #If there is an emergency state
            lift_platform.run(0)                                                            #Set the lifting speed to 0degrees/second (This allows a set deceleration so the box does not fall off from a hard brake)
            check_emergency_stop()                                                          #Check if the movement is allowed to restart
            lift_platform.run_target(max_speed_lift * remote_speed_adjust, lift_angle , then=Stop.HOLD, wait=False) #Resend the platform lifting motor run to the target command, don't wait for finishing

    check_emergency_stop()                                                                  #Check if the movement is allowed to start
    #fork_mode == 0 means no extension, 1 = left side, 2 = right side
    if fork_mode > 0:                                                                       #Check if the telescopic fork needs to extend or not
        while tele_fork_clr.color() != Color.WHITE or error_homing_fork == True:            #Start a loop while the result color is not white from scanning the underside of the telescopic fork
            if error_homing_fork == False:                                                  #Check if the error has already been set
                comm_list.append("Clear message")                                           #The message is added to the bluetooth communication waiting list for the master conveyor brick, to be able to resend a same message if needed
                comm_list.append("Homing fork error")                                       #The message is added to the bluetooth communication waiting list for the master conveyor brick
                error_homing_fork = True                                                    #The telescopic fork was not homed correctly, set an error
            ev3.speaker.say("Fork was not homed correctly!")                                #Let the EV3 brick speaker read out this string of text
        if   fork_mode == 1:                                                                #Check if the telescopic fork needs to extend to the left
            if last_homing_left == True: fork_angle = -fork_ext_coord                       #Depending to which side the telescopic fork extended the last time, save an extension angle
            else:                        fork_angle = -fork_ext_coord + adjust_side         #Depending to which side the telescopic fork extended the last time, save an extension angle
        elif fork_mode == 2:                                                                #Check if the telescopic fork needs to extend to the right
            if last_homing_left == True: fork_angle = fork_ext_coord - 50 - adjust_side     #Depending to which side the telescopic fork extended the last time, save an extension angle
            else:                        fork_angle = fork_ext_coord - 50                   #Depending to which side the telescopic fork extended the last time, save an extension angle

        tele_fork_motor.run_target(max_speed_fork * remote_speed_adjust, fork_angle, then = Stop.HOLD, wait=False)  #Send the telescopic fork motor run to the target command, don't wait for finishing
        while tele_fork_motor.control.done() != True:                                       #Start a loop until the telescopic fork motor has reached the target angle
            if emergency_stop == True:                                                      #Check if there is an emergency state
                tele_fork_motor.stop()                                                      #Stop the telescopic fork motor
                check_emergency_stop()                                                      #Check if the movement is allowed to restart
                tele_fork_motor.run_target(max_speed_fork * remote_speed_adjust, fork_angle, then = Stop.HOLD, wait=False)  #Resend the telescopic fork motor run to the target command, don't wait for finishing
        check_emergency_stop()                                                              #Check if the movement is allowed to start
        if   pos_mode == "Pickup":                                                          #Depending on the current position of the lifting platform, select the destination motor angle
            if height_nr <= 6:      lift_platform.run_target(max_speed_lift * remote_speed_adjust, ascending_coord[height_nr] +  dropoff_height         , then=Stop.HOLD, wait=True)    #Send the platform lifting motor run to the target command, wait for finishing
            else:                   lift_platform.run_target(max_speed_lift * remote_speed_adjust, ascending_coord[height_nr] + (dropoff_height / 4 * 3), then=Stop.HOLD, wait=True)    #Send the platform lifting motor run to the target command, wait for finishing
        elif pos_mode == "Dropoff": lift_platform.run_target(max_speed_lift * remote_speed_adjust, descending_coord[height_nr]                          , then=Stop.HOLD, wait=True)    #Send the platform lifting motor run to the target command, wait for finishing
        
        check_emergency_stop()                                                              #Check if the movement is allowed to start
        if   fork_mode == 1: tele_fork_motor.run_angle(max_speed_fork * remote_speed_adjust,  fork_ext_coord, then=Stop.HOLD, wait=False)   #Depending on the side extended, send the telescopic fork motor run to the target command, don't wait for finishing
        elif fork_mode == 2: tele_fork_motor.run_angle(max_speed_fork * remote_speed_adjust, -fork_ext_coord, then=Stop.HOLD, wait=False)   #Depending on the side extended, send the telescopic fork motor run to the target command, don't wait for finishing
        timer_timeout.reset()                                                               #Putting the timer back at 0
        while tele_fork_clr.color() != Color.WHITE:                                         #Start a loop while the result color is not white
            if timer_timeout.time() > fork_timeout_time / remote_speed_adjust and error_homing_fork == False:   #Check if the time for centering has already passed
                error_homing_fork = True                                                    #The telescopic fork was not homed correctly, set an error
                comm_list.append("Clear message")                                           #The message is added to the bluetooth communication waiting list for the master conveyor brick, to be able to resend a same message if needed
                comm_list.append("Homing fork error")                                       #The message is added to the bluetooth communication waiting list for the master conveyor brick
                tele_fork_motor.stop()                                                      #Stop the telescopic fork motor
                ev3.speaker.say("Homing fork taking to long, please check mechanical construction") #Let the EV3 brick speaker read out this string of text
                while error_homing_fork == True: wait(100)                                  #Start a loop while the error has not been reset for the telescopic fork
                if   fork_mode == 1: tele_fork_motor.run_angle(max_speed_fork * remote_speed_adjust,  fork_ext_coord, then=Stop.HOLD, wait=False)   #Depending on the side extended, resend the telescopic fork motor run to the target command, don't wait for finishing
                elif fork_mode == 2: tele_fork_motor.run_angle(max_speed_fork * remote_speed_adjust, -fork_ext_coord, then=Stop.HOLD, wait=False)   #Depending on the side extended, resend the telescopic fork motor run to the target command, don't wait for finishing
                timer_timeout.reset()                                                       #Putting the timer back at 0
            if emergency_stop == True:                                                      #Check if there is an emergency state
                timer_timeout.pause()                                                       #Pausing the timer
                tele_fork_motor.stop()                                                      #Stop the telescopic fork motor
                check_emergency_stop()                                                      #Check if the movement is allowed to start
                if   fork_mode == 1: tele_fork_motor.run_angle(max_speed_fork * remote_speed_adjust,  fork_ext_coord, then=Stop.HOLD, wait=False)   #Depending on the side extended, resend the telescopic fork motor run to the target command, don't wait for finishing
                elif fork_mode == 2: tele_fork_motor.run_angle(max_speed_fork * remote_speed_adjust, -fork_ext_coord, then=Stop.HOLD, wait=False)   #Depending on the side extended, resend the telescopic fork motor run to the target command, don't wait for finishing
                timer_timeout.resume()                                                      #Resuming the timer
        if   fork_mode == 1:                                                                #Check if the telescopic fork was extended to the left
            tele_fork_motor.run_angle( max_speed_fork * remote_speed_adjust, fork_homing_yellow, then=Stop.HOLD, wait=True) #Depending on the side extended, send the telescopic fork motor the homing run to the target command, wait for finishing
            last_homing_left = True                                                         #Set the homing as coming from the left side
        elif fork_mode == 2:                                                                #Check if the telescopic fork was extended to the right
            tele_fork_motor.run_angle(-max_speed_fork * remote_speed_adjust, fork_homing_black,  then=Stop.HOLD, wait=True) #Depending on the side extended, send the telescopic fork motor the homing run to the target command, wait for finishing
            last_homing_left = False                                                        #Set the homing as coming from the right side
        if math.fabs(tele_fork_motor.angle()) > 200:                                        #If the angle difference between before extending, and after is greater than 200degrees, set an error (gearskipping)
            error_homing_fork = True                                                        #The telescopic fork was not homed correctly, set an error
            comm_list.append("Clear message")                                               #The message is added to the bluetooth communication waiting list for the master conveyor brick, to be able to resend a same message if needed
            comm_list.append("Homing fork error")                                           #The message is added to the bluetooth communication waiting list for the master conveyor brick
            while error_homing_fork == True: wait(100)                                      #Start a loop while the error has not been reset for the telescopic fork
        tele_fork_motor.reset_angle(0)                                                      #Reset the telescopic fork motor to 0degrees

    
def homing_fork(side_adjust_angle):                                                         #This function is used to put the telescopic fork back in center
    global measured_color_fork                                                              #Using this global variable in this local function (if not defined to be global, it will make a local variable)

    measured_color_fork = tele_fork_clr.color()                                             #Take a color measurement with the color sensor and store it in a local variable
    while measured_color_fork != Color.WHITE:                                               #If the color is not white start this loop
        measured_color_fork = tele_fork_clr.color()                                         #Keep taking color measurements until it is white
    tele_fork_motor.run_angle(max_speed_fork * remote_speed_adjust, side_adjust_angle, then=Stop.HOLD, wait=True)   #Perform a final motor angle adjustment to put it perfectly in center
    tele_fork_motor.reset_angle(0)                                                          #Reset the telescopic fork motor to 0degrees


def homing_crane():                                                                         #This function is used for homing all the motors when starting this program and calibration of the height
    global last_homing_left                                                                 #Using these global variables in this local function (if not defined to be global, it will make a local variable)
    global ascending_coord
    global descending_coord
    global error_homing_fork

    measured_reflect_fork = tele_fork_clr.reflection()                                      #Measure the reflection (0-100) by using the fork homing color sensor
    #print(measured_reflect_fork)                                                           #Print this feedback line when debugging
    if measured_reflect_fork >= 8 and measured_reflect_fork <= 13:                          #If the reflection value is between 8 and 13, a black jumper tile is above the color sensor
        tele_fork_motor.run(-400)                                                           #Start moving the telescopic fork towards the center
        homing_fork(-fork_homing_black)                                                     #Start the fork centering function
        last_homing_left = False                                                            #Set the homing as not coming from the left side
    elif measured_reflect_fork < 7:                                                         #If the reflection value is lower than 7, the yellow jumper tile is above the color sensor or no brick at all
        tele_fork_motor.run(400)                                                            #Start moving the telescopic fork towards the center
        homing_fork(fork_homing_yellow)                                                     #Start the fork centering function
        last_homing_left = True                                                             #Set the homing as coming from the left side
    elif measured_reflect_fork > 13:                                                        #If the reflection value is bigger than 13, the white jumper tile is above the color sensor
        tele_fork_motor.run_angle(400, 400)                                                 #Start moving the telescopic fork away from the center
        tele_fork_motor.run(-400)                                                           #Start moving the telescopic fork towards the center
        homing_fork(-fork_homing_black)                                                     #Start the fork centering function
        last_homing_left = False                                                            #Set the homing as not coming from the left side

    counter = 0                                                                             #Defining a local variable
    while True:                                                                             #Start a forever loop
        if touch_base.pressed() == True:                                                    #If the homing touch sensor is being pressed
            driving_motor.run_angle(200, 180)                                               #Drive a bit backwards
        if touch_base.pressed() == True:                                                    #If the homing touch sensor is still being pressed
            lift_platform.run_angle(max_speed_lift, 720)                                    #Lift the platform a bit
        if touch_base.pressed() == True:                                                    #If the homing touch sensor is still being pressed
            counter += 1                                                                    #Add 1 to the error variable counter
            ev3.speaker.say("Sensor should not be pressed right now, check crane failure")  #Let the EV3 brick speaker read out this string of text
            print("Sensor should not be pressed right now, check crane failure")            #Print this feedback line when debugging
            if counter >= 2:                                                                #If there are 2 errors in row
                comm_list.append("Crane sensor remains pushed error")                       #The message is added to the bluetooth communication waiting list for the master conveyor brick
                conv_status_to_crane_mbox.wait_new()                                        #Wait for a message that clears the error
                comm_list.append("Retry")                                                   #The clearing message is added to the bluetooth communication waiting list for the master conveyor brick so a same command can be resend
            continue                                                                        #Restart the loop to try again
        break                                                                               #If the sensor in not pressed now, break out of the loop

    driving_motor.run(-100)                                                                 #Drive slowly towards the beginning of the warehouse
    while touch_base.pressed() != True: continue                                            #While the touch sensor is not pressed, continue this loop
    driving_motor.hold()                                                                    #Hold the motor in place directly after pressing the sensor with the rack shock damper
    driving_motor.reset_angle(drive_homing_adjust)                                          #Reset the driving motor angle to a preset value
    driving_motor.run_target(max_speed_drive, 100)                                          #Drive away from the shock that pressed the touch sensor to release it again

    lift_platform.run(-100)                                                                 #Start lowering the platform slowly
    init_red = False                                                                        #Make a local variable that is set False
    while touch_base.pressed() != True:                                                     #Start a loop that continues until the touch sensor is pressed
        init_clr = rack_clr.rgb()                                                           #Read the RGB color from the rack color sensor
        if init_clr[0] > 20 and init_red == False:                                          #Red is the first color, if the value is greater than 20, and red has not been seen yet
            init_red = True                                                                 #Set the local variable as red has been seen
            timer_timeout.reset()                                                           #Reset the timer back to 0
        if init_red == True and timer_timeout.time() > 700:                                 #If red has been seen, and the timer is over 700ms
            print("Homing taking to long")                                                  #Print this feedback line when debugging
            break                                                                           #Break out of the while loop, the platform is now down at homing position
    lift_platform.hold()                                                                    #Hold the platform motor on this angle
    lift_platform.reset_angle(0)                                                            #Set the motor angle to 0

    lift_platform.run_target( max_speed_lift, 700, then=Stop.HOLD, wait=True)               #Lift the platform to a safe position to be able to extend the telescopic fork
    tele_fork_motor.run_angle(max_speed_fork, -fork_ext_coord, then = Stop.HOLD, wait=True) #Extend the telescopic fork
    tele_fork_motor.run_angle(max_speed_fork, fork_ext_coord, then=Stop.HOLD, wait=False)   #Retract the telescopic fork
    timer_timeout.reset()                                                                   #Reset the timer back to 0
    while tele_fork_clr.color() != Color.WHITE:                                             #Start a loop while the result color is not white
        if timer_timeout.time() > fork_timeout_time:                                        #Check if the time for centering has already passed
            ev3.speaker.say("Homing fork taking to long, please check mechanical construction") #Let the EV3 brick speaker read out this string of text
            error_homing_fork = True                                                        #Set the error for centering taking to long
            comm_list.append("Homing fork error")                                           #The message is added to the bluetooth communication waiting list for the master conveyor brick
            conv_status_to_crane_mbox.wait_new()                                            #Wait for a message that clears the error
            comm_list.append("Retry")                                                       #The clearing message is added to the bluetooth communication waiting list for the master conveyor brick so a same command can be resend
            tele_fork_motor.run_angle(max_speed_fork,  fork_ext_coord,  then=Stop.HOLD, wait=False) #Extend the telescopic fork again
            timer_timeout.reset()                                                           #Reset the timer back to 0
    tele_fork_motor.run_angle( max_speed_fork, fork_homing_yellow, then=Stop.HOLD, wait=True)   #Move the last degrees after seeing the white jumper brick to center the telescopic fork
    last_homing_left = True                                                                 #Set the last telescopic fork center as coming from the left
    driving_motor.run_target( max_speed_drive, rack_coord[1],   then=Stop.HOLD, wait=False) #Drive to the first rack (locations 0 / 6)
    
    while True:                                                                             #Start a forever loop
        error_homing_fork = False                                                           #Reset the error for centering taking to long
        ascending_coord  = [65]                                                             #Reset the global variable list (In case the loop returns to the beginning this is needed)
        lift_platform.run_target( max_speed_lift,  0, then=Stop.HOLD, wait=True)            #Bring the platform to motor angle 0 (Should already be there, unless the loop returned to the beginning)
        for x in range(6):                                                                  #Perform 6 times
            lift_platform.run(300)                                                          #Make the lifting platform go up at constant speed (300degrees/second)
            position_clr = rack_clr.rgb()                                                   #Scan the rack for RGB colors
            while position_clr[0] < 40: position_clr = rack_clr.rgb()                       #Start a loop checking the intensity of the color red scanned, wait for it to be higher than 40 (0-100)
            while position_clr[0] > 30: position_clr = rack_clr.rgb()                       #Start a loop checking the intensity of the color red scanned, wait for it to be lower  than 30 (0-100)
            lift_platform.hold()                                                            #Lock the lifting motor in this position
            ascending_coord.append(lift_platform.angle() - 10)                              #Add this motor angle to the ascending list for the current height position
            if (x == 0 and ascending_coord[-1] - ascending_coord[-2] > 200) or (x > 0 and (ascending_coord[-1] - ascending_coord[-2] < 500 or ascending_coord[-1] - ascending_coord[-2] > 650)):    #Check if the position is possible
                error_homing_fork = True                                                    #If the angle is out of bounds, a possible double scan / non-scan has occured
                break                                                                       #Break out of the for(6)
            ev3.speaker.beep()                                                              #Make a beep sound everytime a floor has been scanned correctly
        if error_homing_fork == True: continue                                              #Redo the homing of the rack calibration if an error has been found
        lift_platform.run_angle(max_speed_lift, dropoff_height / 3 * 2)                     #If all floor levels have been scanned correctly, go up to dropoff height TODO is this need? It will be done in the next forever loop anyway
        break                                                                               #Break out of the forever loop

    while True:                                                                             #Start a forever loop
        error_homing_fork = False                                                           #Reset the error for centering taking to long
        descending_coord = [55]                                                             #Reset the global variable list (In case the loop returns to the beginning this is needed)
        lift_platform.run_target(max_speed_lift, ascending_coord[-1] + dropoff_height / 3 * 2)  #Bring the platform above the heighest rack calibration point so it can be scanned
        for x in range(6):                                                                  #Perform 6 times
            lift_platform.run(-300)                                                         #Make the lifting platform go down at constant speed (300degrees/second)
            position_clr = rack_clr.rgb()                                                   #Scan the rack for RGB colors
            while position_clr[0] < 40: position_clr = rack_clr.rgb()                       #Start a loop checking the intensity of the color red scanned, wait for it to be lower than 40 (0-100)
            lift_platform.hold()                                                            #Lock the lifting motor in this position
            descending_coord.insert(1, lift_platform.angle() + 5)                           #Add this motor angle to the descending list for the current height position
            if x > 0:                                                                       #Check after the first loop
                if descending_coord[2] - descending_coord[1] > 650 or descending_coord[2] - descending_coord[1] < 500:  #Check if the position is possible
                    error_homing_fork = True                                                #If the angle is out of bounds, a possible double scan / non-scan has occured
                    break                                                                   #Break out of the for(6)
            ev3.speaker.beep()                                                              #Make a beep sound everytime a floor has been scanned correctly
            if x < 5:                                                                       #If the loop is not on the last run
                lift_platform.run(-300)                                                     #Restart the lifting platform motor to go down at constant speed
                while position_clr[0] > 30: position_clr = rack_clr.rgb()                   #Start a loop checking the intensity of the color red scanned, wait for it to be lower  than 30 (0-100)
        if error_homing_fork == True: continue                                              #Redo the homing of the rack calibration if an error has been found
        break                                                                               #Break out of the forever loop

    ascending_coord[0]  = descending_coord[1] - 35                                          #Override the first value in the ascending  list (conveyor position) with the measured value from floor level 1 with a constant added
    descending_coord[0] = descending_coord[1] - 45                                          #Override the first value in the descending list (conveyor position) with the measured value from floor level 1 with a constant added
            
    print(ascending_coord, descending_coord)                                                #Print this feedback line when debugging
    #[31, 108, 711, 1302, 1898, 2485, 3020] [21, 66, 670, 1271, 1863, 2453, 2986]           #Example of the motor angles for all lifting positions (Conveyors 101,105 / rack level 1 / rack level 2 / rack level 3 / rack level 4 / rack level 5 / rack level 6)


##########~~~~~~~~~~CREATING MULTITHREADS~~~~~~~~~~##########
sub_crane_control = Thread(target=crane_control)                                            #This creates a thread, made from a previously defined function. No arguments can be given
sub_communication_control = Thread(target=communication_control)                            


##########~~~~~~~~~~CONNECT TO THE MASTER WITH BLUETOOTH~~~~~~~~~~##########
client_crane.connect('inputconveyors')                                                      #Tries to connect to the bluetooth device with the arguments name, if it can't it will stop this program
sub_communication_control.start()                                                           #This starts the loop thread that sends all the bluetooth communication to the Master EV3 brick. Non-blocking


##########~~~~~~~~~~EV3 HAS INCREMENTAL ENCODERS IN IT'S MOTORS, SO IT'S NEEDED TO CALIBRATE EVERY STARTUP~~~~~~~~~~##########
homing_crane()


##########~~~~~~~~~~PROGRAM RUNNING THE CRANE~~~~~~~~~~##########
sub_crane_control.start()                                                                   #This starts the loop thread that controls the stacker crane operations. Non-blocking

comm_list.append("Ready")                                                                   #The message is added to the bluetooth communication waiting list for the master conveyor brick
comm_list.append("Homing finished")                                                         #The message is added to the bluetooth communication waiting list for the master conveyor brick

while True:                                                                                 #Start a forever loop
    conv_status_to_crane_mbox.wait_new()                                                    #Wait for a new message to be received by bluetooth from the Master EV3 brick
    updated_pos = conv_status_to_crane_mbox.read()                                          #Read the last message received by bluetooth from the Master EV3 brick
    print(updated_pos)                                                                      #Print this feedback line when debugging
    if   "Store at"     in updated_pos: crane_task = updated_pos                            #Compare the received message and set a new crane task
    elif "Retrieve at"  in updated_pos: crane_task = updated_pos                            #Compare the received message and set a new crane task
    elif "Move between" in updated_pos: crane_task = updated_pos                            #Compare the received message and set a new crane task
    elif "Drive to"     in updated_pos: crane_task = updated_pos                            #Compare the received message and set a new crane task
    elif "Startup"      in updated_pos: crane_task = updated_pos                            #Compare the received message and set a new crane task
    elif updated_pos == "Emergency stop pushed":    emergency_stop      = True              #Compare the received message and set the Emergency state
    elif updated_pos == "Emergency stop reset":     emergency_stop      = False             #Compare the received message and reset the Emergency state
    elif updated_pos == "Reset error fork homing":  error_homing_fork   = False             #Compare the received message and reset the error to try again
    elif updated_pos == "Reset drive error":        error_driving_pos   = False             #Compare the received message and reset the error to try again
    elif updated_pos == "Input received":           correct_dropoff     = True              #Compare the received message and set a correct dropoff at location 101
    elif "Height adjust" in updated_pos:                                                    #Compare the received message
        adj_val = updated_pos.split(": ")                                                   #Split the incoming command and save the results in a list
        remote_height_adjust = int(adj_val[1]) * 3                                          #Read the second argument of the list, it contains the new height, save it in the global variable
    elif "Speed adjust"  in updated_pos:                                                    #Compare the received message
        adj_val = updated_pos.split(": ")                                                   #Split the incoming command and save the results in a list
        remote_speed_adjust = int(adj_val[1]) / 100                                         #Read the second argument of the list, it contains the new speed, save it in the global variable


##########~~~~~~~~~~EXTRA'S NOT USED IN NORMAL PROGRAM, FOR SAVING PURPOSES~~~~~~~~~~##########                                                                        
ev3.speaker.say("Thank you for watching, subscribe for more Technic Machines")              #The EV3 speaker will read out this textstring
