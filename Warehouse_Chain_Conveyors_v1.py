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
##########~~~~~~~~~~WAREHOUSE XL: CHAIN CONVEYORS~~~~~~~~~~##########
##########~~~~~~~~~~~~~YOUTUBE CHANNEL: MR JOS~~~~~~~~~~~~~##########
#####################################################################
#####################################################################


##########~~~~~~~~~~HARDWARE CONFIGURATION~~~~~~~~~~##########
ev3 = EV3Brick()
#   Motors definition
lift_inp        =   Motor(Port.A)                                                           #Corner transfer lifting motor  [Position 104]
chain_inp       =   Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)            #Conveyor with chains           [Position 105]
lift_outp       =   Motor(Port.C)                                                           #Corner transfer lifting motor  [Position 102]
chain_outp      =   Motor(Port.D)                                                           #Conveyor with chains           [Position 101]
#   Sensor definition
chain_inp_sens  =   UltrasonicSensor(Port.S1)                                               #Ultrasonic distance sensor for scanning if the box has left/arrived on the input  chain conveyor   [Position 105]
chain_outp_sens =   UltrasonicSensor(Port.S4)                                               #Ultrasonic distance sensor for scanning if the box has left/arrived on the output chain conveyor   [Position 101]


##########~~~~~~~~~~HOMING POSITION ANGLES WHEN SENSOR ACTIVATED~~~~~~~~~~##########
lift_homing     =   -30                                                                     #Motor angle that will be used when resetting the homing position


##########~~~~~~~~~~GEARING~~~~~~~~~~##########
max_speed_chain_conv =  1200                                                                #Maximal speed (deg/sec) that a chain conveyor motor is allowed to rotate at
max_speed_lifting    =   900                                                                #Maximal speed (deg/sec) that a corner transfer lifting motor is allowed to rotate at
lifting_height       =  2500                                                                #Motor angle to bring the corner transfer rolls on the same height as stationary rolls
chain_outp_dist      =  3200                                                                #Motor angle to bring a centered pallet from the rack output position to the corner transfer position
chain_inp_dist       =  3200                                                                #Motor angle to bring a centered pallet from the corner transfer position to the rack input position
chain_to_scis_dist   = -2300                                                                #Motor angle to bring a centered pallet from the corner transfer position to the scissorlift position
chain_from_scis_dist =  5500                                                                #Motor angle to bring a centered pallet from the scissorlift position to the rack input position
remote_speed_adjust  =     1                                                                #Multiplier for the conveyor speed (0 < Value <= 1), adjusted by manual control on the touchscreen


##########~~~~~~~~~~MAXIMUM SPEED, MAXIMUM ACCELERATION, MAXIMUM POWER~~~~~~~~~~##########
lift_inp.control.limits  (1000,  300, 100)                                                  #Motor settings [Max speed allowed (deg/sec) / Acceleration (deg/sec²) / Power (%)]
chain_inp.control.limits (1400,  720, 100)                                                  #Motor settings [Max speed allowed (deg/sec) / Acceleration (deg/sec²) / Power (%)]
lift_outp.control.limits (1000,  300, 100)                                                  #Motor settings [Max speed allowed (deg/sec) / Acceleration (deg/sec²) / Power (%)]
chain_outp.control.limits(1400,  720, 100)                                                  #Motor settings [Max speed allowed (deg/sec) / Acceleration (deg/sec²) / Power (%)]


##########~~~~~~~~~~BLUETOOTH SETUP, CLIENT SIDE~~~~~~~~~~##########
client_chain = BluetoothMailboxClient()                                                     #Defining the name for this slave (Client) EV3 brick for sending/receiving bluetooth commands

conv_status_to_chain_mbox   = TextMailbox('conveyor update to chain' , client_chain)        #Receiving the bluetooth commands from the master EV3 brick on this channel
chain_status_to_conv_mbox   = TextMailbox('chain update to conveyor' , client_chain)        #Sending   the bluetooth commands to   the master EV3 brick on this channel


##########~~~~~~~~~~CREATING AND STARTING A TIMER~~~~~~~~~~##########   [Not used]
#timer_movement = StopWatch()                                                               #Creating a timer
#timer_movement.time()                                                                      #Reading  a timer's current value (ms)
#timer_movement.pause()                                                                     #Stopping a timer
#timer_movement.resume()                                                                    #Resuming a timer
#timer_movement.reset()                                                                     #Putting  a timer back at 0, if not stopped it will just keep running but start from 0 again.


##########~~~~~~~~~~BUILDING GLOBAL VARIABLES~~~~~~~~~~##########
box_cha_outp        = False                                                                 #Box status of the output chain  conveyor [True (box present) / False (no box)] (location 101)
box_cha_inp         = False                                                                 #Box status of the input  chain  conveyor [True (box present) / False (no box)] (location 105)
box_rol_outp        = False                                                                 #Box status of the output corner transfer [True (box present) / False (no box)] (location 102)
box_rol_mid         = False                                                                 #Box status of the middle roll   conveyor [True (box present) / False (no box)] (location 103)
box_rol_inp         = False                                                                 #Box status of the input  corner transfer [True (box present) / False (no box)] (location 104)
box_scis_inp        = "None"                                                                #Box status of the scissorlift        [None / Delivered / Bring to input chain] (location 106)
conveyor_status     = "Conveyors off"                                                       #Status of the conveyors mode [Off / Automatic] Moving pallets between conveyors locations 101,102,103,104,105,106
comm_list           = []                                                                    #List with all commands that still need to be send to the master conveyor brick
pallet_taken_out    = 0                                                                     #Amount of pallets taken out of the high bay racks during the runtime of this program
pallet_taken_in     = 0                                                                     #Amount of pallets taken in to  the high bay racks during the runtime of this program
inp_sens_err        = False                                                                 #Variable to see if the input  chain conveyor sensor has triggered an error / has been reset
outp_sens_err       = False                                                                 #Variable to see if the output chain conveyor sensor has triggered an error / has been reset
scissor_down        = True                                                                  #Variable to see if the scissorlift is down and the input chain conveyor can run or not (if False the box on the scissorlift will not move by the chains)
robot_used          = False                                                                 #Status of the robot mode [Off(False) / On(True) ] Moving pallets between location 106 and 107
emergency_stop      = False                                                                 #Variable to see if the emergency stop has been pushed / reset


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
#create_file.close()                                                                        #Close the file again, to be able to call it later again


##########~~~~~~~~~~DEFINE SUB-ROUTINES~~~~~~~~~~##########
##########~~~~~~~~~~BLUETOOTH SENDING COMMUNICATION COMMANDS~~~~~~~~~~##########
def communication_control():                                                                #The master EV3 brick will receive with this function all Bluetooth commands
    global comm_list                                                                        #TODO is this global needed? BT lists are not global but work in master
    wait(1000)                                                                              #Give the master 1second time to start up the bluetooth receiving function
    while True:                                                                             #Start a forever loop
        wait(300)                                                                           #Allow the receiver some time to read the previous message first
        if len(comm_list) > 0:                                                              #Check if there is a command in the master conveyor communication list
            print("outgoing message: %s."%comm_list[0])                                     #Print this feedback line when debugging
            chain_status_to_conv_mbox.send(comm_list[0])                                    #The message is send by bluetooth communication to the master conveyor brick
            wait(50)                                                                        #TODO is this needed?
            del comm_list[0]                                                                #If the message is send, delete it from the communication list


##########~~~~~~~~~~DEFINE SUB-ROUTINES [FUNCTIONS] FOR THE OPERATION OF THE WAREHOUSE~~~~~~~~~~##########
def check_emergency_stop():                                                                 #This function checks if there is a need to stop the current/next movement
    global emergency_stop                                                                   #Using this global variable in this local function (if not defined to be global, it will make a local variable)
    
    while emergency_stop == True or conveyor_status == "Conveyors off": wait(250)           #If there is an emergency stop, start this loop until the emergency stop has been reset


def auto_corner_transfer_output():                                                          #A loop that checks if the output corner transfer can do a job
    while True:                                                                             #Start a forever loop
        if conveyor_status == "Conveyors automatic":                                        #Check if conveyors are not turned off
            if box_cha_outp == True and box_rol_outp == False: move_box_chain("output to corner")   #Check if the chain locations need to start transferring (location 101 to 102)
            if box_rol_outp == False and lift_outp.angle() > 50:                            #Check if the output corner transfer is empty and up
                check_emergency_stop()                                                      #Check if a movement is allowed to start
                while lift_outp.angle() > 50:                                               #While the corner transfer is up
                    lift_outp.run_target(max_speed_lifting * remote_speed_adjust, 0, then=Stop.COAST, wait=False)   #Send the corner lifting motor run to the target command, don't wait for finishing
                    while lift_outp.angle() > 50:                                           #Start a loop until the motor is near 50degrees of finishing the rotation
                        if emergency_stop == True:                                          #If during this loop an emergency stop occurs or the conveyors are turned off
                            lift_outp.stop()                                                #Stop the lifting motor
                            check_emergency_stop()                                          #Check if the movement is allowed to restart
                            break                                                           #Close this loop, so motors will restart again

def auto_corner_transfer_input():                                                           #A loop that checks if the input corner transfer can do a job
    while True:                                                                             #Start a forever loop
        if conveyor_status == "Conveyors automatic":                                        #Check if conveyors are not turned off
            if box_rol_inp == True and robot_used == False:                     move_box_chain("corner to input")               #Check what chain conveyor locations need to start transferring
            if box_rol_inp == True and robot_used == True:                      move_box_chain("move to scissorlift for 6DoF")  #Check what chain conveyor locations need to start transferring
            if box_rol_inp == False and box_scis_inp == "Bring to input chain": move_box_chain("move from scissorlift")         #Check what chain conveyor locations need to start transferring


def move_box_chain(pos):                                                                    #Transferring a pallet from one to another chain conveyor position
    global box_cha_outp                                                                     #Using these global variables in this local function (if not defined to be global, it will make a local variable)
    global box_cha_inp
    global box_rol_outp
    global box_rol_inp
    global box_scis_inp
    global scissor_down

    check_emergency_stop()                                                                  #Check if a movement is allowed to start
    print("The move request has started for %s."%pos)                                       #Print this feedback line when debugging
    if pos == "output to corner":                                                           #Check what chain conveyor locations need to start transferring
        check_free_chain_outp("Full")                                                       #Check if the box is at the start position
        while lift_outp.angle() > 50:                                                       #While the corner transfer is up
            lift_outp.run_target(max_speed_lifting * remote_speed_adjust, 0, then=Stop.COAST, wait=False)   #Send the corner lifting motor run to the target command, don't wait for finishing
            while lift_outp.angle() > 50:                                                   #While the corner transfer is up
                if emergency_stop == True:                                                  #If during this loop an emergency stop occurs or the conveyors are turned off
                    lift_outp.stop()                                                        #Stop the lifting motor
                    check_emergency_stop()                                                  #Check if the movement is allowed to restart
                    break                                                                   #Close this loop, so motors will restart again
        check_emergency_stop()                                                              #Check if a movement is allowed to start
        m1_angle = chain_outp.angle() + chain_outp_dist                                     #Get the current motor angle for the chain conveyor and add the movement, save it as a local variable
        while chain_outp.angle() < m1_angle - 50:                                           #Start a loop until the chain conveyor is near 50degrees of finishing the rotation
            chain_outp.run_target(max_speed_chain_conv * remote_speed_adjust, m1_angle, then=Stop.COAST, wait=False)    #Send the chain motor run to the target command, don't wait for finishing
            while chain_outp.angle() < m1_angle - 50:                                       #Start a loop until the chain conveyor is near 50degrees of finishing the rotation
                if emergency_stop == True:                                                  #If during this loop an emergency stop occurs or the conveyors are turned off
                    chain_outp.stop()                                                       #Stop the chain conveyor motor
                    check_emergency_stop()                                                  #Check if the movement is allowed to restart
                    break                                                                   #Close this loop, so motors will restart again
        check_emergency_stop()                                                              #Check if a movement is allowed to start
        while chain_outp.control.done() == False: continue                                  #While the chain conveyor motor has not reached its destination
        chain_outp.run_angle(max_speed_chain_conv * remote_speed_adjust, -80, then=Stop.COAST, wait=True)   #Send the chain motor run to the target command, wait for finishing. This returns the box to the center after alignment
        check_free_chain_outp("Free")                                                       #Check if the box has left the start position
        box_cha_outp = False                                                                #Remove the box present at the old location (location 101)
        comm_list.append("Chain out empty")                                                 #The message is added to the bluetooth communication waiting list for the master conveyor brick 
        check_emergency_stop()                                                              #Check if a movement is allowed to start
        while lift_outp.angle() < lifting_height - 50:                                      #While the corner transfer is not up
            lift_outp.run_target(max_speed_lifting * remote_speed_adjust, lifting_height, then=Stop.COAST, wait=False)  #Send the corner lifting motor run to the target command, don't wait for finishing
            while lift_outp.angle() < lifting_height - 50:                                  #While the corner transfer is not up
                if emergency_stop == True:                                                  #If during this loop an emergency stop occurs or the conveyors are turned off
                    lift_outp.stop()                                                        #Stop the lifting motor
                    check_emergency_stop()                                                  #Check if the movement is allowed to restart
                    break                                                                   #Close this loop, so motors will restart again
        box_rol_outp = True                                                                 #Set the box present at the new location (location 102)
        comm_list.append("Roll out full")                                                   #The message is added to the bluetooth communication waiting list for the master conveyor brick
    elif pos == "corner to input":                                                          #Check what chain conveyor locations need to start transferring
        while lift_inp.angle() > 50:                                                        #While the corner transfer is up
            lift_inp.run_target(max_speed_lifting * remote_speed_adjust, 0, then=Stop.COAST, wait=False)    #Send the corner lifting motor run to the target command, don't wait for finishing
            while lift_inp.angle() > 50:                                                    #While the corner transfer is up
                if emergency_stop == True:                                                  #If during this loop an emergency stop occurs or the conveyors are turned off
                    lift_inp.stop()                                                         #Stop the lifting motor
                    check_emergency_stop()                                                  #Check if the movement is allowed to restart
                    break                                                                   #Close this loop, so motors will restart again
        while box_cha_inp == True: wait(500)                                                #Wait for the input chain conveyor to be empty (location 105)
        check_free_chain_inp("Free")                                                        #Check if there is no box at the end position
        check_emergency_stop()                                                              #Check if a movement is allowed to start
        m1_angle = chain_inp.angle() + chain_inp_dist                                       #Get the current motor angle for the chain conveyor and add the movement, save it as a local variable
        while chain_inp.angle() < m1_angle - 50:                                            #Start a loop until the chain conveyor is near 50degrees of finishing the rotation
            chain_inp.run_target(max_speed_chain_conv * remote_speed_adjust, m1_angle, then=Stop.COAST, wait=False) #Send the chain motor run to the target command, don't wait for finishing
            while chain_inp.angle() < m1_angle - 50:                                        #Start a loop until the chain conveyor is near 50degrees of finishing the rotation
                if emergency_stop == True:                                                  #If during this loop an emergency stop occurs or the conveyors are turned off
                    chain_inp.stop()                                                        #Stop the chain conveyor motor
                    check_emergency_stop()                                                  #Check if the movement is allowed to restart
                    break                                                                   #Close this loop, so motors will restart again
        check_free_chain_inp("Full")                                                        #Check if the box has arrived at the end position
        box_cha_inp = True                                                                  #Set the box present at the new location (location 105)
        comm_list.append("Chain in full from corner")                                       #The message is added to the bluetooth communication waiting list for the master conveyor brick
        check_emergency_stop()                                                              #Check if a movement is allowed to start
        while lift_inp.angle() < lifting_height - 50:                                       #While the corner transfer is not up
            lift_inp.run_target(max_speed_lifting * remote_speed_adjust, lifting_height, then=Stop.COAST, wait=False)   #Send the corner lifting motor run to the target command, don't wait for finishing
            while lift_inp.angle() < lifting_height - 50:                                   #While the corner transfer is not up
                if emergency_stop == True:                                                  #If during this loop an emergency stop occurs or the conveyors are turned off
                    lift_inp.stop()                                                         #Stop the lifting motor
                    check_emergency_stop()                                                  #Check if the movement is allowed to restart
                    break                                                                   #Close this loop, so motors will restart again
        box_rol_inp = False                                                                 #Remove the box present at the old location (location 104)
        comm_list.append("Roll in empty")                                                   #The message is added to the bluetooth communication waiting list for the master conveyor brick
    elif pos == "move to scissorlift for 6DoF":                                             #Check what chain conveyor locations need to start transferring
        while lift_inp.angle() > 50:                                                        #While the corner transfer is up
            lift_inp.run_target(max_speed_lifting * remote_speed_adjust, 0, then=Stop.COAST, wait=False)    #Send the corner lifting motor run to the target command, don't wait for finishing
            while lift_inp.angle() > 50:                                                    #While the corner transfer is up
                if emergency_stop == True:                                                  #If during this loop an emergency stop occurs or the conveyors are turned off
                    lift_inp.stop()                                                         #Stop the lifting motor
                    check_emergency_stop()                                                  #Check if the movement is allowed to restart
                    break                                                                   #Close this loop, so motors will restart again
        while scissor_down == False or box_cha_inp == True or box_scis_inp != "None":       #Wait for the scissorlift to be down and free
            #print("Not moving, status:", scissor_down, box_cha_inp, box_scis_inp)          #Print this feedback line when debugging
            wait(500)                                                                       #2Hz interval checking the status
            if robot_used == False: return                                                  #If in the meantime the robot mode is turned off, break out of the function and stop feeding the box to the scissorlift
        check_emergency_stop()                                                              #Check if a movement is allowed to start
        m1_angle = chain_inp.angle() + chain_to_scis_dist                                   #Get the current motor angle for the chain conveyor and add the movement, save it as a local variable
        while chain_inp.angle() > m1_angle + 50:                                            #Start a loop until the chain conveyor is near 50degrees of finishing the rotation
            chain_inp.run_target(max_speed_chain_conv * remote_speed_adjust, m1_angle, then=Stop.COAST, wait=False) #Send the chain motor run to the target command, don't wait for finishing
            while chain_inp.angle() > m1_angle + 50:                                        #Start a loop until the chain conveyor is near 50degrees of finishing the rotation
                if emergency_stop == True:                                                  #If during this loop an emergency stop occurs or the conveyors are turned off
                    chain_inp.stop()                                                        #Stop the chain conveyor motor
                    check_emergency_stop()                                                  #Check if the movement is allowed to restart
                    break                                                                   #Close this loop, so motors will restart again
        box_scis_inp = "Delivered"                                                          #Set the box present at the new location (location 106)
        comm_list.append("Scissor full")                                                    #The message is added to the bluetooth communication waiting list for the master conveyor brick
        check_emergency_stop()                                                              #Check if a movement is allowed to start
        while lift_inp.angle() < lifting_height - 50:                                       #While the corner transfer is not up
            lift_inp.run_target(max_speed_lifting * remote_speed_adjust, lifting_height, then=Stop.COAST, wait=False)   #Send the corner lifting motor run to the target command, don't wait for finishing
            while lift_inp.angle() < lifting_height - 50:                                   #While the corner transfer is not up
                if emergency_stop == True:                                                  #If during this loop an emergency stop occurs or the conveyors are turned off
                    lift_inp.stop()                                                         #Stop the lifting motor
                    check_emergency_stop()                                                  #Check if the movement is allowed to restart
                    break                                                                   #Close this loop, so motors will restart again
        box_rol_inp = False                                                                 #Remove the box present at the old location (location 104)
        comm_list.append("Roll in empty")                                                   #The message is added to the bluetooth communication waiting list for the master conveyor brick
    elif pos == "move from scissorlift":                                                    #Check what chain conveyor locations need to start transferring
        while lift_inp.angle() > 50:                                                        #While the corner transfer is up
            lift_inp.run_target(max_speed_lifting * remote_speed_adjust, 0, then=Stop.COAST, wait=False)    #Send the corner lifting motor run to the target command, don't wait for finishing
            while lift_inp.angle() > 50:                                                    #While the corner transfer is up
                if emergency_stop == True:                                                  #If during this loop an emergency stop occurs or the conveyors are turned off
                    lift_inp.stop()                                                         #Stop the lifting motor
                    check_emergency_stop()                                                  #Check if the movement is allowed to restart
                    break                                                                   #Close this loop, so motors will restart again
        while box_cha_inp == True or scissor_down == False: wait(500)                       #Wait for the input chain conveyor to be empty (location 105)
        check_free_chain_inp("Free")                                                        #Check if there is no box at the end position
        check_emergency_stop()                                                              #Check if a movement is allowed to start
        m1_angle = chain_inp.angle() + chain_from_scis_dist                                 #Get the current motor angle for the chain conveyor and add the movement, save it as a local variable
        while chain_inp.angle() < m1_angle - 50:                                            #Start a loop until the chain conveyor is near 50degrees of finishing the rotation
            chain_inp.run_target(max_speed_chain_conv * remote_speed_adjust, m1_angle, then=Stop.COAST, wait=False) #Send the chain motor run to the target command, don't wait for finishing
            while chain_inp.angle() < m1_angle - 50:                                        #Start a loop until the chain conveyor is near 50degrees of finishing the rotation
                if emergency_stop == True:                                                  #If during this loop an emergency stop occurs or the conveyors are turned off
                    chain_inp.stop()                                                        #Stop the chain conveyor motor
                    check_emergency_stop()                                                  #Check if the movement is allowed to restart
                    break                                                                   #Close this loop, so motors will restart again
        check_free_chain_inp("Full")                                                        #Check if the box has arrived at the end position
        box_cha_inp = True                                                                  #Set the box present at the new location (location 105)
        box_scis_inp = "None"                                                               #Remove the box present at the old location (location 106)
        comm_list.append("Chain in full from scissor")                                      #The message is added to the bluetooth communication waiting list for the master conveyor brick
        check_emergency_stop()                                                              #Check if a movement is allowed to start
        while lift_inp.angle() < lifting_height - 50:                                       #While the corner transfer is not up
            lift_inp.run_target(max_speed_lifting * remote_speed_adjust, lifting_height, then=Stop.COAST, wait=False)   #Send the corner lifting motor run to the target command, don't wait for finishing
            while lift_inp.angle() < lifting_height - 50:                                   #While the corner transfer is not up
                if emergency_stop == True:                                                  #If during this loop an emergency stop occurs or the conveyors are turned off
                    lift_inp.stop()                                                         #Stop the lifting motor
                    check_emergency_stop()                                                  #Check if the movement is allowed to restart
                    break                                                                   #Close this loop, so motors will restart again
        box_rol_inp = False                                                                 #Remove the box present at the old location (location 104)
        comm_list.append("Roll in empty")                                                   #The message is added to the bluetooth communication waiting list for the master conveyor brick


def draw_counters():                                                                        #This function writes information on the EV3 screen
    takeout_msg = "{} taken out of the racks: {}              "                             #Prepare a line of text (string) to be formatted TODO change out with %s to shorten the code
    takein_msg = "{} taken to the racks: {}                "                                #Prepare a line of text (string) to be formatted TODO change out with %s to shorten the code
    if pallet_taken_out < 2: ev3.screen.draw_text(4,  8, takeout_msg.format("pallet", pallet_taken_out), text_color=Color.BLACK, background_color=Color.WHITE)  #Write a line of text on the EV3 screen
    else:                    ev3.screen.draw_text(4,  8, takeout_msg.format("pallets", pallet_taken_out), text_color=Color.BLACK, background_color=Color.WHITE) #Write a line of text on the EV3 screen
    if pallet_taken_in < 2:  ev3.screen.draw_text(4, 22, takein_msg.format("pallet", pallet_taken_in), text_color=Color.BLACK, background_color=Color.WHITE)    #Write a line of text on the EV3 screen
    else:                    ev3.screen.draw_text(4, 22, takein_msg.format("pallets", pallet_taken_in), text_color=Color.BLACK, background_color=Color.WHITE)   #Write a line of text on the EV3 screen


def check_free_chain_inp(position):                                                         #This function checks if there is a box or not by a sensor on the chain conveyor position (location 105)
    global inp_sens_err                                                                     #Using this global variable in this local function (if not defined to be global, it will make a local variable)

    counter = 0                                                                             #Defining a local variable
    if position == "Free":                                                                  #If the mode tells that the conveyor should be free (empty location 105)
        while chain_inp_sens.distance() < 200 or inp_sens_err == True:                      #Start a loop if the sensor does see something within 20cm (There should be nothing normally)
            counter += 1                                                                    #Every loop add 1 to the counter (sometimes these sensors give a false value on first read)
            if counter > 5:                                                                 #After more than 5 loops
                if inp_sens_err == False and chain_inp_sens.distance() < 200:               #Check if the error is not set True yet and the sensor does see something within 20cm still
                    comm_list.append("Reset")                                               #The message is added to the bluetooth communication waiting list for the master conveyor brick deleting previous message to be able to resend the same
                    comm_list.append("Inp not free error")                                  #The message is added to the bluetooth communication waiting list for the master conveyor brick
                    inp_sens_err = True                                                     #Set the error to be True
                #ev3.speaker.say("Input conveyor not free")                                 #This makes the EV3 brick talk [Not used anymore, slows the program to much]
            wait(50)
    elif position == "Full":                                                                #If the mode tells that the conveyor should be full (box at location 105)
        while chain_inp_sens.distance() > 70 or inp_sens_err == True:                       #Start a loop if the sensor does not see something within 7cm (There should be a box normally)
            counter += 1                                                                    #Every loop add 1 to the counter (sometimes these sensors give a false value on first read)
            if counter > 5:                                                                 #After more than 5 loops
                if inp_sens_err == False and chain_inp_sens.distance() > 70:                #Check if the error is not set True yet and the sensor does not see something within 7cm still
                    comm_list.append("Reset")                                               #The message is added to the bluetooth communication waiting list for the master conveyor brick deleting previous message to be able to resend the same
                    comm_list.append("Inp not full error")                                  #The message is added to the bluetooth communication waiting list for the master conveyor brick
                    inp_sens_err = True                                                     #Set the error to be True
                #ev3.speaker.say("Input conveyor not full")                                 #This makes the EV3 brick talk [Not used anymore, slows the program to much]
            wait(50)


def check_free_chain_outp(position):                                                        #This function checks if there is a box or not by a sensor on the chain conveyor position (location 101)
    global outp_sens_err                                                                    #Using this global variable in this local function (if not defined to be global, it will make a local variable)

    counter = 0                                                                             #Defining a local variable
    if position == "Free":                                                                  #If the mode tells that the conveyor should be free (empty location 101)
        while chain_outp_sens.distance() < 200 or outp_sens_err == True:                    #Start a loop if the sensor does see something within 20cm (There should be nothing normally)
            counter += 1                                                                    #Every loop add 1 to the counter (sometimes these sensors give a false value on first read)
            if counter > 5:                                                                 #After more than 5 loops
                if outp_sens_err == False and chain_outp_sens.distance() < 200:             #Check if the error is not set True yet and the sensor does see something within 20cm still
                    comm_list.append("Reset")                                               #The message is added to the bluetooth communication waiting list for the master conveyor brick deleting previous message to be able to resend the same
                    comm_list.append("Outp not free error")                                 #The message is added to the bluetooth communication waiting list for the master conveyor brick
                    outp_sens_err = True                                                    #Set the error to be True
                #ev3.speaker.say("Output conveyor not free")                                #This makes the EV3 brick talk [Not used anymore, slows the program to much]
            wait(50)
    elif position == "Full":                                                                #If the mode tells that the conveyor should be full (box at location 101)
        while chain_outp_sens.distance() > 70 or outp_sens_err == True:                     #Start a loop if the sensor does not see something within 7cm (There should be a box normally)
            counter += 1                                                                    #Every loop add 1 to the counter (sometimes these sensors give a false value on first read)
            if counter > 5:                                                                 #After more than 5 loops
                if outp_sens_err == False and chain_outp_sens.distance() > 70:              #Check if the error is not set True yet and the sensor does not see something within 7cm still
                    comm_list.append("Reset")                                               #The message is added to the bluetooth communication waiting list for the master conveyor brick deleting previous message to be able to resend the same
                    comm_list.append("Outp not full error")                                 #The message is added to the bluetooth communication waiting list for the master conveyor brick
                    outp_sens_err = True                                                    #Set the error to be True
                #ev3.speaker.say("Output conveyor not full")                                #This makes the EV3 brick talk [Not used anymore, slows the program to much]
            wait(50)
        comm_list.append("Input received")                                                  #The message is added to the bluetooth communication waiting list for the master conveyor brick


##########~~~~~~~~~~CREATING MULTITHREADS~~~~~~~~~~##########
sub_corner_transfer_output = Thread(target=auto_corner_transfer_output)                     #This creates a thread, made from a previously defined function. No arguments can be given
sub_corner_transfer_input  = Thread(target=auto_corner_transfer_input)
sub_communication_control  = Thread(target=communication_control)


##########~~~~~~~~~~PROGRAM STARTING~~~~~~~~~~##########
draw_counters()                                                                             #This calls the function for drawing counters on the EV3 screen. Blocking


##########~~~~~~~~~~CONNECT TO THE MASTER EV3~~~~~~~~~~##########
##########ALWAYS START THE SERVER-BRICK FIRST. THEN START THIS SLAVE-BRICK, OR IT WILL TIMEOUT IF IT CAN NOT CONNECT TO THE MASTER BRICK BY BLUETOOTH ##########
client_chain.connect('inputconveyors')                                                      #Tries to connect to the bluetooth device with the arguments name, if it can't it will stop this program


##########~~~~~~~~~~EV3 HAS INCREMENTAL ENCODERS IN IT'S MOTORS, SO IT'S NEEDED TO CALIBRATE EVERY STARTUP~~~~~~~~~~##########
### 40% Torque is used now with the fixed axles driveline and reinforced angle gearing. It does sometimes let the internal clutches slip of the linear actuators whilst homing, if it does it to much, lower the homing torque
lift_outp.run_until_stalled(-max_speed_lifting, then=Stop.COAST, duty_limit=40)             #Start turning the corner lift motor at maximal 40% power (downwards). Wait until the motor is stalled and release the brake (Coast Homing)
lift_outp.reset_angle(lift_homing)                                                          #Reset the corner lift motor angle to a preset value, lower than 0, so in normal operation the motor will never reach this endstop
lift_outp.run_target(max_speed_lifting, lifting_height, then=Stop.COAST, wait=False)        #Send the corner lift motor run to the target command (up position), don't wait for finishing

lift_inp.run_until_stalled(-max_speed_lifting, then=Stop.COAST, duty_limit=40)              #Start turning the corner lift motor at maximal 40% power (downwards). Wait until the motor is stalled and release the brake (Coast Homing)
lift_inp.reset_angle(lift_homing)                                                           #Reset the corner lift motor angle to a preset value, lower than 0, so in normal operation the motor will never reach this endstop
lift_inp.run_target(max_speed_lifting, lifting_height, then=Stop.COAST, wait=True)          #Send the corner lift motor run to the target command (up position), wait for finishing


##########~~~~~~~~~~PROGRAM RUNNING THE CHAIN CONVEYORS~~~~~~~~~~##########
sub_corner_transfer_output.start()                                                          #This starts the loop thread that controls the output chain conveyor. Non-blocking
sub_corner_transfer_input.start()                                                           #This starts the loop thread that controls the input  chain conveyor. Non-blocking
sub_communication_control.start()                                                           #This starts the loop thread that sends all the bluetooth communication to the Master EV3 brick. Non-blocking
comm_list.append("Homing finished")                                                         #The message is added to the bluetooth communication waiting list for the master conveyor brick

while True:                                                                                 #Start a forever loop
    conv_status_to_chain_mbox.wait_new()                                                    #Wait for a new message to be received by bluetooth from the Master EV3 brick
    updated_pos = conv_status_to_chain_mbox.read()                                          #Read the last message received by bluetooth from the chain conveyor brick
    print("incoming message: %s."%updated_pos)                                              #Print this feedback line when debugging
    if   updated_pos == "Chain out full":                                                   #Compare the received message
        box_cha_outp = True                                                                 #Set the box present at this location (location 105)
        pallet_taken_out += 1                                                               #Count 1 extra box to have been taken out of the rack
        draw_counters()                                                                     #This calls the function for drawing counters on the EV3 screen
    elif updated_pos == "Chain in empty":                                                   #Compare the received message
        box_cha_inp  = False                                                                #Set the box not present at this location (location 105)
        pallet_taken_in += 1                                                                #Count 1 extra box to have been taken into the rack
        draw_counters()                                                                     #This calls the function for drawing counters on the EV3 screen
    elif updated_pos == "Roll out empty":        box_rol_outp   = False                     #Compare the received message and set the box not present at this location (location 104)
    elif updated_pos == "Roll in full":          box_rol_inp    = True                      #Compare the received message and set the box present at this location (location 101)
    elif updated_pos == "Input from scissor":    box_scis_inp   = "Bring to input chain"    #Compare the received message and set the box present at this location (location 106)
    elif updated_pos == "Scissor empty":         box_scis_inp   = "None"                    #Compare the received message and set the box not present at this location (location 106)
    elif updated_pos == "Scissor is down":       scissor_down   = True                      #Compare the received message and set the scissorlift in the down state
    elif updated_pos == "Scissor is up":         scissor_down   = False                     #Compare the received message and set the scissorlift in the not down state
    elif updated_pos == "Robot used":            robot_used     = True                      #Compare the received message and set the robot in the used state
    elif updated_pos == "Robot not used":        robot_used     = False                     #Compare the received message and set the robot in the not used state
    elif updated_pos == "Emergency stop pushed": emergency_stop = True                      #Compare the received message and set the Emergency stop in the True state
    elif updated_pos == "Emergency stop reset":  emergency_stop = False                     #Compare the received message and set the Emergency stop in the False state
    elif updated_pos == "Roll out full":         box_rol_outp   = True                      #Compare the received message and set the box present at this location (location 104)
    elif updated_pos == "Chain in full":         box_cha_inp    = True                      #Compare the received message and set the box present at this location (location 101)
    elif updated_pos == "Outp error reset":      outp_sens_err  = False                     #Compare the received message and set the Output error in the False state to try reset location 101
    elif updated_pos == "Inp error reset":       inp_sens_err   = False                     #Compare the received message and set the Output error in the False state to try reset location 105
    elif updated_pos == "Conveyors off":         conveyor_status = "Conveyors off"          #Compare the received message and set the conveyors in the not used state
    elif updated_pos == "Conveyors automatic":   conveyor_status = "Conveyors automatic"    #Compare the received message and set the conveyors in the used state
    elif updated_pos == "Chain out empty":       box_cha_outp   = False                     #Compare the received message and set the box not present at this location (location 105)
    elif updated_pos == "Roll in empty":         box_rol_inp    = False                     #Compare the received message and set the box not present at this location (location 102)
    elif "Speed adjustment" in updated_pos:                                                 #Compare the received message
        adj_val = updated_pos.split(": ")                                                   #Split the incoming command and save the results in a list
        remote_speed_adjust = int(adj_val[1]) / 100                                         #Read the second argument of the list, it contains the new speed, save it in the global variable


##########~~~~~~~~~~EXTRA'S NOT USED IN NORMAL PROGRAM, FOR SAVING PURPOSES~~~~~~~~~~##########                                                                        
ev3.speaker.say("Thank you for watching, subscribe for more Technic Machines")              #The EV3 speaker will read out this textstring
