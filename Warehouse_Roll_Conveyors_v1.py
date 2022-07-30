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

from pybricks.iodevices import UARTDevice
from utime import ticks_ms
from uartremote import *

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.
# Any (parts) of program taken from Anton's Mindstorms Hacks are used under the;
# MIT License: Copyright (c) 2021 Anton's Mindstorms
# MIT License: Copyright (c) 2022 Mr Jos for the rest of the code

#####################################################################
#####################################################################
##########~~~~~PROGRAM WRITTEN BY JOZUA VAN RAVENHORST~~~~~##########
##########~~UARTREMOTE WRITTEN BY ANTON'S MINDSTORMS HACKS~##########
##########~~~~~HIGHBAY WAREHOUSE XL: ROLLER CONVEYORS~~~~~~##########
##########~~~~~~~~~~~~~YOUTUBE CHANNEL: MR JOS~~~~~~~~~~~~~##########
####~~HUGE THANKS TO STE7AN AND ANTON FROM ANTONSMINDSTORMS.COM~~####
#####################################################################
#####################################################################


##########~~~~~~~~~~HARDWARE CONFIGURATION~~~~~~~~~~##########
ev3         = EV3Brick()
#   UART setup 
port        = Port.S1
baudrate    = 115200
ur          = UartRemote(Port.S1)
ur.uart     = UARTDevice(port, baudrate=baudrate, timeout=100)
#   Motors definition
roll_conv_inp       =   Motor(Port.A)                                                       #Conveyor with rolls    [Position 102]
roll_conv_mid       =   Motor(Port.B)                                                       #Conveyor with rolls    [Position 103]
roll_conv_outp      =   Motor(Port.C)                                                       #Conveyor with rolls    [Position 104]
scissorlift         =   Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)        #Scissorlift            [Position 106]
#   Sensor definition
touch_input_emerg   =   TouchSensor     (Port.S2)                                           #Red button to trigger an emergency stop
touch_input_reset   =   TouchSensor     (Port.S3)                                           #Blue button to reset the emergency state
sens_output_floor   =   UltrasonicSensor(Port.S4)                                           #Ultrasonic distance sensor for manual requests


##########~~~~~~~~~~HOMING POSITION ANGLES WHEN SENSOR/ENDPOINT ACTIVATED~~~~~~~~~~##########
scissorlift_homing  =   -900                                                                #Motor angle that will be used when resetting the homing position
scissorlift_top_pos =   2950                                                                #Motor angle at which the pickup/dropoff position is for the robot arm


##########~~~~~~~~~~GEARING~~~~~~~~~~##########
roll_dist_conv              =    610                                                        #Motor angle to bring a centered pallet from 1 roll conveyor to another
max_speed_roll_conv         =    200                                                        #Maximal speed (deg/sec) that a roll conveyor motor is allowed to rotate at
max_speed_scissorlift       =   1400                                                        #Maximal speed (deg/sec) that the scissorlift motor is allowed to rotate at
max_speed_roll_adj          =      1                                                        #Multiplier for the roll conveyor speed (0 < Value <= 1), adjusted by manual control on the touchscreen
max_speed_scissorlift_adj   =      1                                                        #Multiplier for the scissorlift speed   (0 < Value <= 1), adjusted by manual control on the touchscreen


##########~~~~~~~~~~MAXIMUM SPEED, MAXIMUM ACCELERATION, MAXIMUM POWER~~~~~~~~~~##########
roll_conv_inp.control.limits (1400,  300, 100)                                              #Motor settings [Max speed allowed (deg/sec) / Acceleration (deg/sec²) / Power (%)]
roll_conv_mid.control.limits (1400,  300, 100)                                              #Motor settings [Max speed allowed (deg/sec) / Acceleration (deg/sec²) / Power (%)]
roll_conv_outp.control.limits(1400,  300, 100)                                              #Motor settings [Max speed allowed (deg/sec) / Acceleration (deg/sec²) / Power (%)]
scissorlift.control.limits   (1400,  300, 100)                                              #Motor settings [Max speed allowed (deg/sec) / Acceleration (deg/sec²) / Power (%)]

#scissorlift.target_tolerances(30, 5)                                                       #Allowed deviation from the target before motion is considered complete. (degrees/second, degrees) [NOT USED!]


##########~~~~~~~~~~BLUETOOTH SETUP, SERVER SIDE~~~~~~~~~~##########
server_roll = BluetoothMailboxServer()                                                      #Defining the name for this master (Server) EV3 brick for sending/receiving bluetooth commands

conv_status_to_chain_mbox   =   TextMailbox     ('conveyor update to chain' , server_roll)  #Sending   the bluetooth commands to   the chain EV3 brick on this channel
chain_status_to_conv_mbox   =   TextMailbox     ('chain update to conveyor' , server_roll)  #Receiving the bluetooth commands from the chain EV3 brick on this channel

conv_status_to_crane_mbox   =   TextMailbox     ('conveyor update to crane' , server_roll)  #Sending   the bluetooth commands to   the crane EV3 brick on this channel
crane_status_to_conv_mbox   =   TextMailbox     ('crane update to conveyor' , server_roll)  #Receiving the bluetooth commands from the crane EV3 brick on this channel

conv_status_to_robot_mbox   =   TextMailbox     ('conveyor update to robot' , server_roll)  #Sending   the bluetooth commands to   the robot EV3 brick on this channel
robot_status_to_conv_mbox   =   TextMailbox     ('robot update to conveyor' , server_roll)  #Receiving the bluetooth commands from the robot EV3 brick on this channel


##########~~~~~~~~~~CREATING AND STARTING A TIMER~~~~~~~~~~##########
#timer_movement = StopWatch()                                                               #Creating a timer
#timer_movement.time()                                                                      #Reading  a timer's current value (ms)
#timer_movement.pause()                                                                     #Stopping a timer
#timer_movement.resume()                                                                    #Resuming a timer
#timer_movement.reset()                                                                     #Putting  a timer back at 0, if not stopped it will just keep running but start from 0 again.
timer_floors    = StopWatch()                                                               #TODO check if this timer is still used


##########~~~~~~~~~~BUILDING GLOBAL VARIABLES~~~~~~~~~~##########
rack_length             =   5                                                               #Amount of racks  on 1 side (length)
rack_floors             =   6                                                               #Amount of floors on 1 side (height)
total_storage_positions =   rack_length * rack_floors * 2                                   #Amount of total storage positions in the highbay calculated automatically
floor_storage_space     =   5                                                               #Amount of total storage positions on the floor near the robot arm

storage_locations       =   []                                                              #Var used to choose a random free storage position for next job
for x in range(total_storage_positions): storage_locations.append(x)                        #Filling the var with the position number for every location [0,1,2,...]

emergency_stop          =   True                                                            #Variable to see if the emergency stop has been pushed / reset
take_out_list           =   []                                                              #List containing all jobs to bring pallets out of the highbay
bring_here_list         =   []                                                              #List containing all jobs to bring pallets to a certain place in the highbay
floor_takeout_list      =   []                                                              #List containing all jobs to bring pallets back to the scissorlift by the 6DoF
comm_list_chain         =   []                                                              #List with all commands that still need to be send to the chain conveyor brick
comm_list_crane         =   []                                                              #List with all commands that still need to be send to the crane conveyor brick
comm_list_robot         =   []                                                              #List with all commands that still need to be send to the robot conveyor brick
comm_list_uart          =   []                                                              #List with all commands that still need to be send to the ESP32 by UART
inbound                 =   0                                                               #Amount of tasks taking in  by 6DoF
outbound                =   0                                                               #Amount of tasks taking out by 6DoF

hb_crane_input          =   "Off"                                                           #Status of the input  mode    [Off / Manual / Automatic] Taking a pallet from location 105 to 100
hb_crane_output         =   "Off"                                                           #Status of the output mode    [Off / Manual / Automatic] Taking a pallet from location 100 to 101
conveyors               =   "Off"                                                           #Status of the conveyors mode [Off / Automatic         ] Moving pallets between conveyors locations 101,102,103,104,105,106
robot_used              =   False                                                           #Status of the robot mode     [Off(False) / On(True)   ] Moving pallets between location 106 and 107
crane_status            =   "Homing"                                                        #Status of the stacker crane  [Homing / Ready / Picked up / Dropped off]
robot_status            =   "Homing"                                                        #Status of the robot arm      [Homing / Ready / Picked up / Performing task]
chain_status            =   "Homing"                                                        #Status of the chains brick   [Homing / Ready] 
mode_chosen             =   False                                                           #Used as wait variable whilst the amount of bluetooth connections is unknown [For mode selection]

man_adj_scissor_max     =   0                                                               #Adjusting the top_pos for the scissorlift (-10 <= value <= 10),  adjusted by manual control on the touchscreen
connections             =   3                                                               #Adjusting the amount of bluetooth connections (0 <= value <= 3), adjusted by manual control on the touchscreen

machines         = ["Crane", "Output chain conveyor", "Output corner transfer", "Middle roll conveyor", "Input corner transfer", "Input chain conveyor", "Scissor table", "Robot arm", "Pick&Place 1", "Pick&Place 2", "Pick&Place 3", "Pick&Place 4", "Pick&Place 5"]  #Storage locations names
positions        = [100,     101,                     102,                      103,                    104,                     105,                    106,             107,          110,           111,            112,            113,            114]             #Storage locations numbers
outside_dict     = {}                                                                       #Dictionary that will hold every location's information [Location # / Machine name / Box(True/False) / Box name / Request pending(True/False)]
for i in range(len(positions)): outside_dict.update({"location%s"%positions[i] : {"position" : positions[i], "machine" : machines[i], "box" : False, "name" : "No box present", "request" : False}})

outside_dict["location106"]["liftposition"] = "homing"                                      #Adding to the scissorlift dictionary the key for the current lifting position    
outside_dict["location106"]["liftheight"]   = scissorlift_top_pos                           #Adding to the scissorlift dictionary the key for the top position lifting height (Includes the manual adjustment)
for i in range(floor_storage_space): outside_dict["location10%s"%i]["dropofftime"] = 99999  #Adding to each floor location dictionary the time key when it was put down on the floor (seconds) TODO see if this is used


##########~~~~~~~~~~BRICK STARTUP SETTINGS~~~~~~~~~~##########
ev3.speaker.set_volume          (volume=90, which='_all_')                                  #Set the volume for all sounds (speaking and beeps etc)
ev3.speaker.set_speech_options  (language='en', voice='m7', speed=None, pitch=None)         #Select speaking language, and a voice (male/female)
small_font  = Font(size=6)                                                                  #Creating a font with  6 pixel height for text on screen    [Not used]
normal_font = Font(size=10)                                                                 #Creating a font with 10 pixel height for text on screen    [Not used]
big_font    = Font(size=16)                                                                 #Creating a font with 16 pixel height for text on screen
ev3.screen.set_font(big_font)                                                               #Choose a preset font for writing next texts
ev3.screen.clear()                                                                          #Make the screen empty (all pixels white)
#ev3.speaker.beep()                                                                         #Brick will make a beep sound 1 time                        [Not used, example]
#ev3.light.on(Color.GREEN)                                                                  #Turns the green lights on the brick on                     [Not used, example]
ev3.light.off()                                                                             #Turn the lights off on the brick
#ev3.screen.draw_text(4, 2, "", text_color=Color.BLACK, background_color=Color.WHITE)       #X/Y position for writing on the screen, textstring, text color, background color   [Example]


##########~~~~~~~~~~CREATING A FILE THAT IS SAVED OFFLINE~~~~~~~~~~##########
#os.remove("wms_hb_boxstatus.txt")                                                          #Removing the wms file for whatever reason you might need to delete it  [KEEP # for normal operation!]


##########~~~~~~~~~~RACK BOXES, SAVING IN AN OFFLINE FILE TO REMEMBER ON STARTUP LOCATIONS~~~~~~~~~~##########
create_file = open("wms_hb_boxstatus.txt", "a")                                             #Create new file on first start ever (file doesn't exist yet), if it does exist already just open it
create_file.write("")                                                                       #Add an empty string to have something in it
create_file.close()                                                                         #Close the .txt file.

with open("wms_hb_boxstatus.txt") as retrieve_wms:                                          #Reading from offline file, storing them in an online list
    wms_online_list = retrieve_wms.read().splitlines()                                      #Each line in the .txt file is added in the online list [line1, line2, line3,...]

for x in range(total_storage_positions - len(wms_online_list)):                             #Adding extra positions at first start ever or expanding racks
    wms_online_list.append("No box present")                                                #Setting the new extra locations as free rack positions

for x in range(len(wms_online_list) - total_storage_positions):                             #Deleting extra positions when downsizing the racks TODO is a for loop needed? Can't do just 1 line?
    del wms_online_list[total_storage_positions:]

def save_offline_wms():                                                                     #Defining the function that stores values from the online list to the offline .txt file
    with open("wms_hb_boxstatus.txt", "w") as backup_wms:                                   #Open the .txt file in 'Write' mode
        for current_state in wms_online_list:                                               #Each value (name of the location) is added on a new line in the .txt file
            backup_wms.write(current_state + "\n")                                          #When no box is present the name is "No box present", this means that line 1 is always location 0, even when empty


########## MANUAL OVERRIDING FOR WMS NAMES! THIS WILL DELETE THE CURRENT FILE AND CAN NOT BE UNDONE!! ##########
########## If you use this, put all pallets in order in the rack starting at down left in 1st rack, and go up, then go to the right side, then 2nd rack etc. ##########
def override_wms():                                                                         #Defining the override function
    override_list = ["Gear 12T LBG", "Tile 3L LBG", "Tile 4L Black", "Liftarm 3L Yellow", "Pin 2L Black", "Gear 20T Blue", "Axle 2L Red", "Liftarm 11L Yellow", "MOTUS HANDLING", \
        "Medium Actuator", "Small Chain Black", "Pin 3L Blue", "Gear Half 12T Tan", "Pin 3L Red", "Liftarm L 4x2 LBG", "Axle Hole Conn T1 LBG", "4Pin Conn LBG"]
    for x in range(len(wms_online_list)):                                                   #For each rack location perform an override
        if x < len(override_list): wms_online_list[x] = override_list[x]                    #If there is a name in the override list, use it
        else: wms_online_list[x] = "No box present"                                         #If there are more locations than items in the override list, add empty locations
    os.remove("wms_hb_outsidestatus.txt")                                                   #Delete the outside offline .txt file (Complete new start)
    print("The complete WMS file has been deleted and replaced by;", override_wms)          #Print this feedback line when debugging

#override_wms()                                                                             #Use hashtag # infront of next line of code if you DO NOT want to override! [KEEP # for normal operation!]
save_offline_wms()                                                                          #Save the offline rack WMS, if you enabled the override, this will make it permanent


##########~~~~~~~~~~WMS FOR ALL LOCATIONS OUTSIDE OF THE RACKS~~~~~~~~~~##########
outside_file = open("wms_hb_outsidestatus.txt", "a")                                        #Create new file on first start ever (file doesn't exist yet), if it does exist already just open it
outside_file.write("")                                                                      #Add an empty string to have something in it
outside_file.close()                                                                        #Close the .txt file.

with open("wms_hb_outsidestatus.txt") as retrieve_outside_wms:                              #Reading from offline file, storing them in an online list
    wms_outside_list = retrieve_outside_wms.read().splitlines()                             #Each line in the .txt file is added in the online list [line1, line2, line3,...]

for x in range(len(positions) - len(wms_outside_list)):                                     #Adding extra positions at first start ever or expanding conveyors/floor storage
    wms_outside_list.append("No box present")                                               #Setting the new extra locations as free machine parts/storage location

for x in range(len(wms_outside_list) - len(positions)):                                     #Deleting extra positions when downsizing the racks TODO is a for loop needed? Can't do just 1 line?
    del wms_outside_list[len(positions):]

for i in range(len(positions)):                                                             #At startup, load offline stored boxes that are outside of the racks
    if wms_outside_list[i] == "No box present": continue                                    #If there is no box at this location, do nothing
    else:                                                                                   #If there is a box, save the name in the corresponding dictionary for this location
        outside_dict["location%s"%positions[i]]["name"] = wms_outside_list[i]
        outside_dict["location%s"%positions[i]]["box"]  = True
        if i >= len(positions) - floor_storage_space:                                       #If the location is floor storage, set the dropoff time to 0 (seconds)
            outside_dict["location%s"%positions[i]]["dropofftime"]   = 0

outside_dict["location107"]["box"] = False                                                  #Reset the robot dictionary as there is no program to handle a box stuck on the arm
outside_dict["location107"]["name"] = "No box present"                                      #It will fall off when initiating (homing), you will need to manually add this box later

def save_outside_wms():
    global wms_outside_list

    for x in range(len(wms_outside_list)):                              
        wms_outside_list[x] = outside_dict["location%s"%positions[x]]["name"]               #Loading all current names into online list
    with open("wms_hb_outsidestatus.txt", "w") as backup_outside_wms:                       #Open the .txt file in 'Write' mode
        for current_outside_state in wms_outside_list:                                      #Each value (name of the location) is added on a new line in the .txt file
            backup_outside_wms.write(current_outside_state + "\n")                          #Save them to the offline file for everything outside the rack


##########~~~~~~~~~~DEFINE SUB-ROUTINES [FUNCTIONS]~~~~~~~~~~##########
##########~~~~~~~~~~UART RECEIVING COMMUNICATION COMMANDS~~~~~~~~~~##########
def mode_warehouse(zone, state):                                                            #The ESP32 will send with this command information about what modes needs to be used
    global hb_crane_input                                                                   #Using these global variables in this local function (if not defined to be global, it will make a local variable)
    global hb_crane_output
    global conveyors
    global robot_used
    global connections
    global mode_chosen

    if   zone == 0:                                                                         #ZONE  0 Is the operation mode for input boxes into the racks (location 105 to 100)
        if   state == 0: hb_crane_input = "Off"                                             #STATE 0 Keeps all boxes stationary on location 105 (input chain conveyor)
        elif state == 1: hb_crane_input = "Automatic"                                       #STATE 1 Chooses a random empty storage location if no preferred place was chosen manually
        elif state == 2: hb_crane_input = "Manual"                                          #STATE 2 Only brings to manually chosen empty storage locations
    elif zone == 1:                                                                         #ZONE  1 Is the operation mode for output boxes from the racks (location 100 to 101)
        if   state == 0: hb_crane_output = "Off"                                            #STATE 0 Keeps all boxes in the rack
        elif state == 1: hb_crane_output = "Automatic"                                      #STATE 1 Chooses a random stored box location if no manual requests are present (to the output chain conveyor)
        elif state == 2: hb_crane_output = "Manual"                                         #STATE 2 Only takes out the manually chosen boxes, or by sensor request a random one
    elif zone == 2:                                                                         #ZONE  2 Is the operation mode for all conveyors (locations 101,102,103,104,105,106)
        if   state == 0:                                                                    #STATE 0 Keeps all boxes stationary on the current location, no inter-conveyor transport
            conveyors = "Off"
            comm_list_chain.append("Conveyors off")                                         #The state gets added to the bluetooth communication waiting list for the chain conveyor brick
        elif state == 1:                                                                    #STATE 1 The transport between all the conveyors is done automatically
            conveyors = "Automatic"
            comm_list_chain.append("Conveyors automatic")                                   #The state gets added to the bluetooth communication waiting list for the chain conveyor brick
    elif zone == 3:                                                                         #ZONE  3 Is the operation mode for the 6 axis robotic arm/6 Degrees of Freedom [6DOF] (location 107)
        if   state == 0:                                                                    #STATE 0 Keeps all boxes around the 6DOF stationary, and no input to the floor
            robot_used = False
            comm_list_chain.append("Robot not used")                                        #The state gets added to the bluetooth communication waiting list for the chain conveyor brick
            comm_list_robot.append("Robot not used")                                        #The state gets added to the bluetooth communication waiting list for the robot conveyor brick
        elif state == 1:                                                                    #STATE 1 Every pallet coming out of the racks will be taken to the floor near the 6DOF, and returned on request
            robot_used = True
            comm_list_chain.append("Robot used")                                            #The state gets added to the bluetooth communication waiting list for the chain conveyor brick
            comm_list_robot.append("Robot used")                                            #The state gets added to the bluetooth communication waiting list for the robot conveyor brick
    elif zone == 4:                                                                         #ZONE  4 Is the bluetooth operation mode, selecting howmany devices will be connecting to this master
        if   state == 0:                                                                    #STATE 0 Sets the connecting devices to 0, for debugging some new parts of code with ESP/Master EV3
            connections = 0
            ev3.screen.draw_text(4,  20, "Debug mode")                                      #Write on the EV3 screen the current mode
        elif state == 1:                                                                    #STATE 1 Sets the connecting devices to 3 [Chain conveyors / Stacker crane / 6DOF]
            connections = 3
            ev3.screen.draw_text(4,  20, "Crane in use")                                    #Write on the EV3 screen the current mode for each machine part
            ev3.screen.draw_text(4,  40, "Chain conveyors used")
            ev3.screen.draw_text(4,  60, "Roll conveyors used")
            ev3.screen.draw_text(4,  80, "Robot can be used")
        elif state == 2:                                                                    #STATE 2 Sets the connecting devices to 2 [Chain conveyors / Stacker crane] Robot can not be used
            connections = 2
            ev3.screen.draw_text(4,  20, "Crane in use")                                    #Write on the EV3 screen the current mode for each machine part
            ev3.screen.draw_text(4,  40, "Chain conveyors used")
            ev3.screen.draw_text(4,  60, "Roll conveyors used")
            ev3.screen.draw_text(4,  80, "Robot can not be used")
        mode_chosen = True                                                                  #If the command for this ZONE 4 has come in, allow the program to start connecting to these devices
    #Debugging line to see current selected modes
    print(zone, state, "Crane input mode: ", hb_crane_input, ". Crane output mode: ", hb_crane_output, ". Conveyors mode: ", conveyors, "Robot used: ", robot_used, "Bluetooth connections: ", connections)


def update_request(loc, state):                                                             #The ESP32 will send with this command a request for a location (pickup/dropoff)
    global bring_here_list                                                                  #Using these global variables in this local function (if not defined to be global, it will make a local variable)
    global take_out_list
    global outside_dict
    global floor_takeout_list

    print("Request update with state %s"%state, " for location %s."%loc)                    #Print this feedback line when debugging
    if 0 <= loc < 100:                                                                      #Check if the request is for a location in the racks (locations 0,1,2,.. max 99)
        if wms_online_list[loc] == "No box present":                                        #Check if there is no box stored on this location
            if state == 0:                                                                  #If no box is stored and the state is 0, the dropoff requests needs to be deleted
                try: bring_here_list.remove(int(loc))                                       #Try to remove the location from the dropoff list
                except: return                                                              #If it fails (it wasn't in the list), end this function
            elif state == 1:                                                                #If no box is stored and the state is 1, a dropoff request needs to be added to this location
                bring_here_list.append(int(loc))                                            #Adding this location to the end of the list that has all locations to bring a box to next
        else: 
            if state == 0:                                                                  #If a box is stored and the state is 0, the pickup request needs to be deleted
                try: take_out_list.remove(int(loc))                                         #Try to remove the location from the pickup list
                except: return                                                              #If it fails (it wasn't in the list), end this function
            elif state == 1:                                                                #If a box is stored and the state is 1, a pickup request needs to be added for this location
                take_out_list.append(int(loc))                                              #Adding this location to the end of the list that has all locations where boxes need to be taken out from

    elif 110 <= loc < 120:                                                                  #Check if the request is for a storage location near the 6DOF (locations 110,111,112,113,114)
        if outside_dict["location%s"%loc]["box"] == True:                                   #Check if there is a box stored on this location
            if state == 0:                                                                  #If a box is on this location and the state is 0, the pickup request needs to be deleted
                try: floor_takeout_list.remove(int(loc))                                    #Try to remove the location from the pickup list
                except: return                                                              #If it fails (it wasn't in the list), end this function
            elif state == 1:                                                                #If a box is on this location and the state is 1, a pickup request needs to be added for this location
                floor_takeout_list.append(int(loc))                                         #Adding this location to the end of the list that has all locations where boxes need to be taken out from


def adjust_manual(task, val):                                                               #The ESP32 will send with this command manual controls for the 6DOF / scissorlift / conveyors / stacker crane
    global man_adj_scissor_max                                                              #Using these global variables in this local function (if not defined to be global, it will make a local variable)
    global max_speed_scissorlift_adj
    global max_speed_roll_adj
    global outside_dict
    global scissorlift_top_pos
    
    print(task,val )                                                                        #Print this feedback line when debugging
    if   task == 0:                                                                         #TASK  0 Are the commands for the 6DOF before homing has started
        if   val == 0:  conv_status_to_robot_mbox.send("Stop motors")                       #VAL   0 Is used to hold all motors in their current position
        elif val == 1:  conv_status_to_robot_mbox.send("Start homing")                      #VAL   1 Is used to start the homing of the 6DOF
        elif val == 2:  conv_status_to_robot_mbox.send("J1 CW")                             #VAL   2 Is used to run Joint 1         Clockwise
        elif val == 3:  conv_status_to_robot_mbox.send("J1 CCW")                            #VAL   3 Is used to run Joint 1 Counter-Clockwise
        elif val == 4:  conv_status_to_robot_mbox.send("J2 CW")                             #VAL   4 Is used to run Joint 2         Clockwise
        elif val == 5:  conv_status_to_robot_mbox.send("J2 CCW")                            #VAL   5 Is used to run Joint 2 Counter-Clockwise
        elif val == 6:  conv_status_to_robot_mbox.send("J3 CW")                             #VAL   6 Is used to run Joint 3         Clockwise
        elif val == 7:  conv_status_to_robot_mbox.send("J3 CCW")                            #VAL   7 Is used to run Joint 3 Counter-Clockwise
    elif 0 < task < 7:  comm_list_robot.append("Adjust J%s: %s"%(task, val))                #TASK 1-6 Are the commands to manually adjust each Joint of the 6DOF by 1degree (-10,10 limits)
    elif task ==  7:                                                                        #TASK  7 Command to mannually adjust the top position of the scissorlift by a few mm (-10,10 limits)
        man_adj_scissor_max = int(val)                                                      #TODO can't this value be added directly to the standard value and saved in the dictionary?
        outside_dict["location106"]["liftheight"]   = scissorlift_top_pos + (man_adj_scissor_max * 40)
    elif task ==  8: max_speed_scissorlift_adj = 0.01 * int(val)                            #TASK  8 Command for manually adjusting the scissorlift speed           (20% ... 100%)
    elif task ==  9: comm_list_crane.append("Height adjustment: %s"%val)                    #TASK  9 Command for manually adjusting the stacker crane basket height (-10,10 limits)
    elif task == 10: comm_list_crane.append("Speed adjustment: %s"%val)                     #TASK 10 Command for manually adjusting the stacker crane speed         (20% ... 100%)
    elif task == 11: comm_list_chain.append("Speed adjustment: %s"%val)                     #TASK 11 Command for manually adjusting the chain conveyors speed       (20% ... 100%)
    elif task == 12: max_speed_roll_adj = 0.01 * int(val)                                   #TASK 12 Command for manually adjusting the roll conveyors speed        (20% ... 100%)


def reset_error(error):                                                                     #The ESP32 will send with this command manual controlled error solving for the conveyors / stacker crane
    print("A request to reset an error at %s"%error)                                        #Print this feedback line when debugging
    if   error == 0:                                                                        #Error 0 Is used to retry checking if a box is on the input chain conveyor or not (if there should/should not be one)
        comm_list_chain.append("Reset")                                                     #A not used message is added to the bluetooth communication waiting list for the chain conveyor brick [To be able to call the real one multiple times if needed]
        comm_list_chain.append("Inp error reset")                                           #This real message is added to the bluetooth communication waiting list for the chain conveyor brick
    if   error == 1:                                                                        #Error 1 Is used to retry checking if a box is on the output chain conveyor or not (if there should/should not be one)
        comm_list_chain.append("Reset")
        comm_list_chain.append("Outp error reset")                                          #This real message is added to the bluetooth communication waiting list for the chain conveyor brick
    elif error == 2:                                                                        #Error 2 Is used to retry centering the telescopic fork of the stacker crane if it noticed a malfunction
        if crane_status == "Homing":                                                        #If the stacker crane is still homing and has a calibration error, send the next messages
            conv_status_to_crane_mbox.send("Try again")
            wait(400)
            conv_status_to_crane_mbox.send("Reset")
        else:                                                                               #If the stacker crane is finished homing and has a homing error, send the next messages
            comm_list_crane.append("Reset")
            comm_list_crane.append("Reset error fork homing")
    elif error == 3:                                                                        #Error 3 Is used to retry reaching its driving position if it noticed a malfunction
        comm_list_crane.append("Reset")
        comm_list_crane.append("Reset drive error")


def mod_wms(loc, state, name):                                                              #The ESP32 will send with this command manual added/deleted boxes to/from the WMS (Warehouse Management System)
    new_name = name.decode()                                                                #Decoding the string (name of the items in the pallet) that is send by UARTRemote
    print("Box status has changed to:", state, ". At location:", loc, "Name:", new_name)    #Print this feedback line when debugging
    if     0 <= loc < 100:                                                                  #Check if the WMS change is for a location in the racks (locations 0,1,2,.. max 99)
        if   state == 0:                                                                    #STATE 0 Removes the box from the rack WMS
            change_one_wms_position(loc, "No box present")                                  #Perform the function that changes 1 WMS position and saves it online+offline
            if int(loc) in take_out_list:                                                   #Check if for this location a request is open to take it out of the rack
                try: take_out_list.remove(int(loc))                                         #If it is, try to remove it from the list
                except: print("Did not find the box to remove")                             #Print this feedback line when debugging
        elif state == 1:                                                                    #STATE 1 Adds the box to the rack WMS
            change_one_wms_position(loc, new_name)                                          #Perform the function that changes 1 WMS position and saves it online+offline
            if int(loc) in bring_here_list:                                                 #Check if for this location a request is open to take a box to this location
                try: bring_here_list.remove(int(loc))                                       #If it is, try to remove it from the list
                except: print("Did not find the box to remove")                             #Print this feedback line when debugging
    elif 100 <= loc < 200:                                                                  #Check if the WMS change is for a conveyor or a storage location near the 6DOF (locations 100,101,102,103,104,105,106,107,110,111,112,113,114)
        if   state == 0:                                                                    #STATE 0 Removes the box from the outside WMS
            outside_dict["location%s"%loc]["name"] = "No box present"                       #Removing the current name from the dictionary
            outside_dict["location%s"%loc]["box"] = False                                   #Removing the box is present state from the dictionary
            if   loc == 101: comm_list_chain.append("Chain out empty")                      #If the location is a conveyor, a message is added to the bluetooth communication waiting list for the chain conveyor brick 
            elif loc == 102: comm_list_chain.append("Roll out empty")
            elif loc == 104: comm_list_chain.append("Roll in empty")
            elif loc == 105: comm_list_chain.append("Chain in empty")
            elif loc == 106: comm_list_chain.append("Scissor empty")
            if int(loc) in floor_takeout_list:                                              #Check if for this location a request is open to take a box from this location
                try: floor_takeout_list.remove(int(loc))                                    #If it is, try to remove it from the list
                except: print("Did not find the box to remove")                             #Print this feedback line when debugging
        elif state == 1:                                                                    #STATE 1 Adds the box to the outside WMS
            outside_dict["location%s"%loc]["name"] = new_name                               #Adding the new name to the dictionary
            outside_dict["location%s"%loc]["box"] = True                                    #Adding the box is present state to the dictionary
            if   loc == 101: comm_list_chain.append("Chain out full")                       #If the location is a conveyor, a message is added to the bluetooth communication waiting list for the chain conveyor brick 
            elif loc == 102: comm_list_chain.append("Roll out full")
            elif loc == 104: comm_list_chain.append("Roll in full")
            elif loc == 105: comm_list_chain.append("Chain in full")
            elif loc == 106:
                comm_list_chain.append("Input from scissor")
                comm_list_robot.append("Scissor full")                                      #If the location is the scissorlift, a message is added to the bluetooth communication waiting list for the robot brick 
        save_outside_wms()                                                                  #Saving the offline WMS for machine parts and floor storage


ur.add_command(mode_warehouse)                                                              #Adding all the previous defined receiving functions to the UartRemote commands list
ur.add_command(update_request)
ur.add_command(adjust_manual)
ur.add_command(reset_error)
ur.add_command(mod_wms)


##########~~~~~~~~~~UART SENDING COMMUNICATION COMMANDS~~~~~~~~~~##########
def update_WMS_ESP(loc, state, name):                                                       #The ESP32 will receive with this command information about 1 WMS position (name by string included)
    print("WMS update", loc, state, name)                                                   #Print this feedback line when debugging (location / box present or not / the name of the box)
    if state == 0:                                                                          #STATE 0 No box is present at this location
        while ur.call("update_storage", '2bs', loc, 0, "") == None: continue                #Send the message, whilst no answer is received, send the message again
    elif state == 1:                                                                        #STATE 1 A box is present at this location
        while ur.call("update_storage", '2b%ss'%len(name), loc, 1, name) == None: continue  #Send the message, whilst no answer is received, send the message again


##########~~~~~~~~~~UART AND BLUETOOTH SENDING COMMUNICATION COMMANDS~~~~~~~~~~##########
def communication_bt_uart():                                                                #The ESP32 will receive with this command all other information that is added to the list from a loop together with Bluetooth commands
    global comm_list_uart                                                                   #TODO is this global needed? BT lists are not global but work

    while True:                                                                             #Start a forever loop
        wait(100)
        if len(comm_list_uart) > 0:                                                         #Check if there is a command in the UART communication list
            comm_uart = comm_list_uart[0]                                                   #Reading the first command in the UART communication list [The result is a list]
            if comm_uart[0] == "update_mode":                                               #The first result is the command name, checking what command is requested, if "update mode" execute this statement
                print("update_mode", '1b%ss'%len(comm_uart[2]), comm_uart[1], comm_uart[2]) #Print this feedback line when debugging (command name / what is send, byte and-or string / the message consisting out of bytes-strings)
                while ur.call("update_mode", '1b%ss'%len(comm_uart[2]), comm_uart[1], comm_uart[2]) == None: continue   #Send the message, whilst no answer is received, send the message again
                del comm_list_uart[0]                                                       #If the message is succesfully send, delete it from the communication list
            elif comm_uart[0] == "update_storage":                                          #TODO check if this is still being used or join with the update_WMS_ESP
                print("update_storage", '2b%ss'%len(comm_uart[3]), comm_uart[1], comm_uart[2], comm_uart[3])
                while ur.call("update_storage", '2b%ss'%len(comm_uart[3]), comm_uart[1], comm_uart[2], comm_uart[3]) == None: continue
                del comm_list_uart[0]
            elif comm_uart[0] == "update_request":                                          #Sending the message that a location has changed its request status (if an automatic mode has chosen a random box transport)
                print("update_request", '2b', comm_uart[1], comm_uart[2])
                while ur.call("update_request", '2b', comm_uart[1], comm_uart[2]) == None: continue
                del comm_list_uart[0]
            elif comm_uart[0] == "transport_pallet":                                        #Sending a message that a pallet has been moved from one position to another, no name is given (faster process speed)
                print("transport_pallet", '2b', comm_uart[1], comm_uart[2])
                while ur.call("transport_pallet", '2b', comm_uart[1], comm_uart[2]) == None: continue
                del comm_list_uart[0]
            else: print("Command not found, can not execute")                               #Print this feedback line when debugging if the command name doesn't exist
            continue                                                                        #UART commands go first, so restart the loop from the beginning, trying to send another UART command TODO Maybe need to change this for faster Emergency stops?
        if len(comm_list_chain) > 0 and chain_status != "Homing":                           #Check if there is a command in the chain conveyor communication list
            print("outgoing message to chain brick: %s."%comm_list_chain[0])                #Print this feedback line when debugging
            conv_status_to_chain_mbox.send(comm_list_chain[0])                              #The message is send by bluetooth communication to the chain conveyor brick
            wait(50)
            conv_status_to_chain_mbox.send(comm_list_chain[0])                              #Sending each double as I had trouble receiving some commands TODO is this still needed?
            del comm_list_chain[0]                                                          #If the message is send, delete it from the communication list
            wait(300)                                                                       #If other lists have no messages, but this one would have 1 more, allow the receiver some time to read this message first
        if len(comm_list_crane) > 0 and crane_status != "Homing":                           #Check if there is a command in the stacker crane communication list
            print("outgoing message to crane brick: %s."%comm_list_crane[0])                #Print this feedback line when debugging
            conv_status_to_crane_mbox.send(comm_list_crane[0])                              #The message is send by bluetooth communication to the stacker crane brick
            wait(50)
            conv_status_to_crane_mbox.send(comm_list_crane[0])                              #Sending each double as I had trouble receiving some commands TODO is this still needed?
            del comm_list_crane[0]                                                          #If the message is send, delete it from the communication list
            wait(300)                                                                       #If other lists have no messages, but this one would have 1 more, allow the receiver some time to read this message first
        if len(comm_list_robot) > 0 and robot_status != "Homing":                           #Check if there is a command in the 6DOF communication list
            print("outgoing message to robot brick: %s."%comm_list_robot[0])                #Print this feedback line when debugging
            conv_status_to_robot_mbox.send(comm_list_robot[0])                              #The message is send by bluetooth communication to the 6DOF brick
            wait(50)
            conv_status_to_robot_mbox.send(comm_list_robot[0])                              #Sending each double as I had trouble receiving some commands TODO is this still needed?
            del comm_list_robot[0]                                                          #If the message is send, delete it from the communication list
            wait(300)                                                                       #If other lists have no messages, but this one would have 1 more, allow the receiver some time to read this message first


##########~~~~~~~~~~DEFINE SUB-ROUTINES [FUNCTIONS] FOR THE OPERATION OF THE WAREHOUSE~~~~~~~~~~##########
def check_emergency_stop(zone):                                                             #This function checks if there is a need to stop the current/next movement
    global emergency_stop                                                                   #Using these global variables in this local function (if not defined to be global, it will make a local variable)
    global conveyors
    global hb_crane_input
    global hb_crane_output

    if   zone == "General" and emergency_stop == True:                                      #If it's only the general emergency stop, check if it's True
        while emergency_stop == True: wait(250)                                             #If true, start this loop until the emergency stop has been reset
    elif zone == "Conveyors" and (emergency_stop == True or conveyors == "Off"):            #If the check request comes from a conveyor, also check if the conveyor mode is turned off
        while emergency_stop == True or conveyors == "Off": wait(250)                       #If true, start this loop until the emergency stop has been reset and conveyors are turned on
    elif zone == "Crane" and (emergency_stop == True or (hb_crane_input == "Off" and hb_crane_output == "Off")):    #If the check request comes from the stacker crane, check the modes
        while emergency_stop == True or (hb_crane_input == "Off" and hb_crane_output == "Off"): wait(250)           #Loop until reset/a stacker crane mode is turned on


def conveyor_transfer_auto():                                                               #A loop that checks if a roll conveyor or scissorlift can do a job
    global outside_dict                                                                     #Using these global variables in this local function (if not defined to be global, it will make a local variable)
    global outbound
    global inbound
    global floor_takeout_list
    global robot_used
    global robot_status

    while True:                                                                             #Start a forever loop
        if outside_dict["location102"]["box"] == True and outside_dict["location103"]["box"] == False:  #Check if there is a box on the output roll conveyor and none on the middle roll conveyor (locations 102,103)
            move_box_roll("output to mid")                                                  #Start the roll conveyor transfer function
        
        #When not using 6DoF (This can be turned on/off remotely)
        if outside_dict["location103"]["box"] == True and outside_dict["location104"]["box"] == False and outside_dict["location106"]["box"] == False and robot_used == False: #If the input roll conveyor is free, and the scissorlift also free, and a box on the middle roll conveyor
            move_box_roll("mid to input")                                                   #Start the roll conveyor transfer function
        if outside_dict["location106"]["box"] == True and outside_dict["location106"]["liftposition"] == "ready down" and robot_used == False:  #If there is a box on the scissorlift and the robot is not used (location 106)
            if outside_dict["location104"]["box"] == True:                                  #Check if there is a box on the output roll conveyor (location 104)
                scissorlift.run_target(max_speed_scissorlift * max_speed_scissorlift_adj, 300, then=Stop.COAST, wait=True)  #TODO I would like to have the scissorlift at this height when waiting for a command (potentially adding a box by forklift)
                while outside_dict["location104"]["box"] == True: wait(100)                 #If there is a box on the output roll conveyor, wait for it to be taken away (location 104)
                scissorlift.run_target(max_speed_scissorlift * max_speed_scissorlift_adj, 0, then=Stop.COAST, wait=True)    #Lower the scissorlift so the box touches the chains (location 106)
                comm_list_chain.append("Scissor is down")                                   #A message is added to the bluetooth communication waiting list for the chain conveyor brick that the scissor is down, box can be taken away (location 106)
        #TODO add a scissorlift raising slightly if the position is free and the robot is not used
        
        #When using 6DoF
        if robot_used == True and outside_dict["location103"]["box"] == True:               #If the robot is used and a box is on the middle roll conveyor (location 103)
            if outbound == 0 and full_floor_spaces("normal") < floor_storage_space and outside_dict["location104"]["box"] == False and len(floor_takeout_list) == 0:    #Check if no box is being taken out and if there is still enough room for 1 more going in
                inbound += 1                                                                #Set 1 extra box going to the robot floor storage
                move_box_roll("mid to input")                                               #Start the roll conveyor transfer function
        if robot_used == True and outside_dict["location106"]["box"] == True and outside_dict["location106"]["liftposition"] == "ready down" and inbound > 0 and outbound == 0: #If there is a box on the scissorlift and it is inbound (location 106)
            outside_dict["location106"]["liftposition"] = "moving up"                       #Change the scissorlift position from down to moving
            sub_scissor_robot_operation.start()                                             #Start the scissorlift-robot transfer function
        if robot_used == True and sens_output_floor.distance() < 40 and len(floor_takeout_list) == 0 and emergency_stop == False:
            for i in positions[-floor_storage_space:]:
                if outside_dict["location%s"%i]["box"] == True and not(i in floor_takeout_list):
                    ev3.speaker.beep()
                    floor_takeout_list.append(i)
                    break
        if inbound == 0 and outbound <= len(floor_takeout_list) and len(floor_takeout_list) > 0 and robot_used == True and robot_status == "Ready" and outside_dict["location107"]["box"] == False: #If there is no inbound, but takeout requests are open
            robot_status = "Performing task"                                                #Set the robot status to a busy state
            #print("outbound count:", outbound, ". Floor take_out_list: ", floor_takeout_list, ". Full floorspaces:" , full_floor_spaces("freespace"), ". Inbound count: ", inbound)    ##Print this feedback line when debugging [Not used]
            outbound += 1                                                                   #Set 1 extra box going out of the robot floor storage
            sub_scissor_robot_operation.start()                                             #Start the scissorlift-robot transfer function


def move_box_roll(pos):                                                                     #Transferring a pallet from one to another roller conveyor
    global outside_dict                                                                     #Using these global variables in this local function (if not defined to be global, it will make a local variable)
    global conveyors
    
    check_emergency_stop("Conveyors")                                                       #Check if the movement is allowed to start
    if pos == "output to mid":                                                              #Check what roll conveyor locations need to start transferring
        m1_angle = roll_conv_mid.angle()  + roll_dist_conv                                  #Get the current motor angle for the first  conveyor and add the movement, save it as a local variable
        m2_angle = roll_conv_outp.angle() + roll_dist_conv                                  #Get the current motor angle for the second conveyor and add the movement, save it as a local variable
        while roll_conv_outp.angle() < m2_angle - 10:                                       #Start a loop until the second conveyor is near 10degrees of finishing the rotation
            roll_conv_mid.run_target (max_speed_roll_conv * max_speed_roll_adj, m1_angle, then=Stop.COAST, wait=False)  #Send the first  motor run to the target command, don't wait for finishing
            roll_conv_outp.run_target(max_speed_roll_conv * max_speed_roll_adj, m2_angle, then=Stop.COAST, wait=False)  #Send the second motor run to the target command, don't wait for finishing
            while roll_conv_outp.angle() < m2_angle - 50:                                   #Start a loop until the second conveyor is near 50degrees of finishing the rotation
                if emergency_stop == True or conveyors == "Off":                            #If during this loop an emergency stop occurs or the conveyors are turned off
                    roll_conv_mid. stop()                                                   #Stop the first conveyor
                    roll_conv_outp.stop()                                                   #Stop the second conveyor
                    check_emergency_stop("Conveyors")                                       #Check if the movement is allowed to restart
                    break                                                                   #Close this loop, so motors will restart again
        while roll_conv_outp.control.done() == False: continue                              #Wait for the second motor to finish reaching the desired motor angle
        outside_dict["location103"]["name"] = outside_dict["location102"]["name"]           #Transfer the name from the previous location to the new one (location 102 to 103)
        outside_dict["location103"]["box"]  = True                                          #Set the box present at the new location (location 103)
        outside_dict["location102"]["name"] = "No box present"                              #Remove the name from the old location (location 102)
        outside_dict["location102"]["box"] = False                                          #Remove the box present at the old location (location 102)
        save_outside_wms()                                                                  #Saving the offline WMS for machine parts and floor storage
        comm_list_uart.append(["transport_pallet", 102, 103])                               #The message is added to the UART communication waiting list for the ESP32
        comm_list_chain.append("Roll out empty")                                            #The message is added to the bluetooth communication waiting list for the chain conveyor brick 
        
    elif pos == "mid to input":                                                             #Check what roll conveyor locations need to start transferring [Same as previous transfer, but different motors]
        m1_angle = roll_conv_inp.angle() + roll_dist_conv
        m2_angle = roll_conv_mid.angle() + roll_dist_conv
        while roll_conv_mid.angle() < m2_angle - 10:
            roll_conv_inp.run_target(max_speed_roll_conv * max_speed_roll_adj, m1_angle, then=Stop.COAST, wait=False)
            roll_conv_mid.run_target(max_speed_roll_conv * max_speed_roll_adj, m2_angle, then=Stop.COAST, wait=False)
            while roll_conv_mid.angle() < m2_angle - 50:
                if emergency_stop == True or conveyors == "Off":
                    roll_conv_inp.stop()
                    roll_conv_mid.stop()
                    check_emergency_stop("Conveyors")
                    break
        while roll_conv_mid.control.done() == False: continue
        outside_dict["location104"]["name"] = outside_dict["location103"]["name"]
        outside_dict["location104"]["box"]  = True
        outside_dict["location103"]["name"] = "No box present"
        outside_dict["location103"]["box"] = False
        save_outside_wms()
        comm_list_uart.append(["transport_pallet", 103, 104])
        comm_list_chain.append("Roll in full")


def crane_auto():                                                                           #A loop that checks if the stacker crane can do a job
    global bring_here_list                                                                  #Using these global variables in this local function (if not defined to be global, it will make a local variable)
    global take_out_list
    global outside_dict
    global crane_status

    temp_name_list = []                                                                     #Defining local variables
    task_storage = 0                                                                        #TODO can't this variable declaration be deleted?

    while True:                                                                             #Start a forever loop 
        check_emergency_stop("Crane")                                                       #Check if a movement is allowed to start
        if crane_status == "Ready" or crane_status == "Dropped off":                        #Check if stacker crane is ready to perform a task
            if outside_dict["location105"]["box"] == True and outside_dict["location100"]["box"] == False:  #Check if there is a box on the input chain conveyor (location 105) and the stacker crane is free (location 100)
                if bring_here_list == [] and hb_crane_input == "Automatic":                 #Check if the mode automatic input is selected and no more manually requests are open
                    while True:                                                             #Start a forever loop
                        chosen_random_nr = choice(storage_locations)                        #Choose a random number (only locations in the rack are in this list)
                        if wms_online_list[chosen_random_nr] == "No box present": break     #Check if the chosen location is free, if not reloop. If it is free, break out of the forever loop
                    comm_list_uart.append(["update_request", chosen_random_nr, 3])          #The message is added to the UART communication waiting list for the ESP32
                    crane_order("Store", 0, chosen_random_nr)                               #Start the crane function
                elif ( hb_crane_input == "Automatic" and bring_here_list != [] ) or ( hb_crane_input == "Manual" and bring_here_list != [] ):   #Check the input mode and if there is a manual request
                    task_storage = bring_here_list[0]                                       #Save the first location that is requested for input
                    comm_list_uart.append(["update_request", task_storage, 3])              #The message is added to the UART communication waiting list for the ESP32
                    crane_order("Store", 0, bring_here_list[0])                             #Start the crane function
                    if len(bring_here_list) > 0:                                            #TODO can't this be solved with a try/except
                        if bring_here_list[0] == task_storage: bring_here_list.pop(0)       #Delete the task that was performed
            if outside_dict["location101"]["box"] == False and outside_dict["location100"]["box"] == False: #Check if the output chain conveyor is empty (location 101) and the stacker crane is free (location 100)
                if (hb_crane_output == "Automatic" and take_out_list == [] ) or (hb_crane_output == "Manual" and take_out_list == [] and sens_output_floor.distance() < 60):    #Check if there is no manual takeout request
                    temp_name_list.clear()                                                  #Empty the local variable list
                    for x in wms_online_list:                                               #For every rack location perform the next check
                        if x != "No box present": temp_name_list.append(x)                  #If the location is not free (meaning there is a box), add it to the local variable list
                    if len(temp_name_list) > 0:                                             #If there is anything in the list
                        chosen_random_name = choice(temp_name_list)                         #Choose a random box from the list
                        chosen_random_box = wms_online_list.index(chosen_random_name)       #Find the location number for this name
                        comm_list_uart.append(["update_request", chosen_random_box, 3])     #The message is added to the UART communication waiting list for the ESP32
                        ev3.speaker.beep()                                                  #Make a beeping sound to show that the input has been accepted and order started
                        crane_order("Retrieve", wms_online_list.index(chosen_random_name), 0)   #Start the crane function
                elif (hb_crane_output == "Automatic" and take_out_list != [] ) or (hb_crane_output == "Manual" and take_out_list != [] ):   #Check if there is a manual takeout request
                    task_storage = take_out_list[0]                                         #Save the first location that is requested for output
                    comm_list_uart.append(["update_request", task_storage, 3])              #The message is added to the UART communication waiting list for the ESP32
                    crane_order("Retrieve", take_out_list[0], 0)                            #Start the crane function
                    if len(take_out_list) > 0:                                              #TODO can't this be solved with a try/except
                        if take_out_list[0] == task_storage: take_out_list.pop(0)           #Delete the task that was performed
            if outside_dict["location100"]["box"] == True:                                  #Check if there is a box stuck on the stacker crane (mostly startup after homing)
                if hb_crane_input != "Off" or hb_crane_output != "Off":                     #Check if the stacker crane is allowed to move
                    while True:                                                             #Start a forever loop
                        chosen_random_nr = choice(storage_locations)                        #Choose a random number (only locations in the rack are in this list)
                        if wms_online_list[chosen_random_nr] == "No box present": break     #Check if the chosen location is free, if not reloop. If it is free, break out of the forever loop
                    comm_list_uart.append(["update_request", chosen_random_nr, 3])          #The message is added to the UART communication waiting list for the ESP32
                    crane_order("Startup full", 0, chosen_random_nr)                        #Start the crane function


def crane_order(task, pos_pickup, pos_dropoff):                                             #Executing a stacker crane movement
    global outside_dict                                                                     #Using these global variables in this local function (if not defined to be global, it will make a local variable)
    global crane_status
    print("Crane task: ",task, ". Pickup at: ", pos_pickup, ". Dropoff at: ", pos_dropoff)  #Print this feedback line when debugging

    if task == "Startup full":                                                              #Check what the stacker crane needs to transfer, if a box is on the stacker crane after homing, store it
        comm_list_crane.append("Startup ,%s"%pos_dropoff)                                   #Tell the crane the job is storing a leftover box into the warehouse. Start adding the dropoff location to the bluetooth command list for the stacker crane
        while crane_status != "Dropped off": wait(50)                                       #Waiting for the crane to tell the box has been put in the warehouse, the crane is available for a new job now
        change_one_wms_position(pos_dropoff, outside_dict["location100"]["name"])           #Perform the function that changes 1 WMS position and saves it online+offline
        outside_dict["location100"]["name"] = "No box present"                              #Remove the name from the old location (location 100)
        outside_dict["location100"]["box"]  = False                                         #Remove the present status from the old location (location 100)
        save_outside_wms()                                                                  #Saving the offline WMS for machine parts and floor storage
        comm_list_uart.append(["transport_pallet", 100, pos_dropoff])                       #The message is added to the UART communication waiting list for the ESP32


    elif task == "Store":                                                                   #Check what the stacker crane needs to transfer, the box on location 105 needs to be stored in the racks
        comm_list_crane.append("Store at ,%s"%pos_dropoff)                                  #Tell the crane the job is storing a box into the warehouse. Start adding the dropoff location to the bluetooth command list for the stacker crane
        while crane_status != "Picked up": wait(50)                                         #Waiting for the crane to tell the box has been taken away from the input chain conveyor
        outside_dict["location100"]["box"]  = True                                          #Set the box present at the new location (location 100)
        outside_dict["location100"]["name"] = outside_dict["location105"]["name"]           #Transfer the name from the previous location to the new one (location 105 to 100)
        outside_dict["location105"]["name"] = "No box present"                              #Remove the name from the old location (location 105)
        outside_dict["location105"]["box"]  = False                                         #Remove the present status from the old location (location 105)
        save_outside_wms()                                                                  #Saving the offline WMS for machine parts and floor storage
        comm_list_uart.append(["transport_pallet", 105, 100])                               #The message is added to the UART communication waiting list for the ESP32
        comm_list_chain.append("Reset")                                                     #A not used message is added to the bluetooth communication waiting list for the chain conveyor brick [To be able to call the real one multiple times if needed]
        comm_list_chain.append("Chain in empty")                                            #Send to chain EV3 that the input chain conveyor is empty
        while crane_status != "Dropped off": wait(50)                                       #Waiting for the crane to tell the box has been put in the warehouse, the crane is available for a new job now
        change_one_wms_position(pos_dropoff, outside_dict["location100"]["name"])           #Perform the function that changes 1 WMS position and saves it online+offline
        outside_dict["location100"]["name"] = "No box present"                              #Remove the name from the old location (location 100)
        outside_dict["location100"]["box"]  = False                                         #Remove the present status from the old location (location 100)
        save_outside_wms()                                                                  #Saving the offline WMS for machine parts and floor storage
        comm_list_uart.append(["transport_pallet", 100, pos_dropoff])                       #The message is added to the UART communication waiting list for the ESP32

    elif task == "Retrieve":                                                                #Check what the stacker crane needs to transfer, a box in the rack needs to be taken out to location 101
        comm_list_crane.append("Retrieve at ,%s"%pos_pickup)                                #Tell the crane the job is taking a box from the warehouse. Start adding the pickup location to the bluetooth command list for the stacker crane
        while crane_status != "Picked up": wait(50)                                         #Waiting for the crane to tell the box has been taken away from the input chain conveyor
        outside_dict["location100"]["box"]  = True                                          #Set the box present at the new location (location 100)
        outside_dict["location100"]["name"] = wms_online_list[pos_pickup]                   #Transfer the name from the previous location to the new one (location 0-59 to 100)
        change_one_wms_position(pos_pickup, "No box present")                               #Perform the function that changes 1 WMS position and saves it online+offline
        save_outside_wms()                                                                  #Saving the offline WMS for machine parts and floor storage
        comm_list_uart.append(["transport_pallet", pos_pickup, 100])                        #The message is added to the UART communication waiting list for the ESP32
        while crane_status != "Dropped off": wait(50)                                       #Waiting for the crane to tell the box has been put in the warehouse, the crane is available for a new job now
        outside_dict["location101"]["box"]  = True                                          #Set the box present at the new location (location 101)
        outside_dict["location101"]["name"] = outside_dict["location100"]["name"]           #Transfer the name from the previous location to the new one (location 100 to 101)
        outside_dict["location100"]["name"] = "No box present"                              #Remove the name from the old location (location 100)
        outside_dict["location100"]["box"]  = False                                         #Remove the present status from the old location (location 100)
        save_outside_wms()                                                                  #Saving the offline WMS for machine parts and floor storage
        comm_list_uart.append(["transport_pallet", 100, 101])                               #The message is added to the UART communication waiting list for the ESP32
        comm_list_chain.append("Chain out full")                                            #Send to chain EV3 that the output chain conveyor is full
        comm_list_chain.append("Emptying mailbox")                                          #Clearing the mailbox for potential same command send later

    elif task == "Move":                                                                    #Check what the stacker crane needs to transfer, a box needs to be stored on another place in the racks TODO[Not used yet]
        comm_list_crane.append("Move between ,%s,%s"%(pos_pickup,pos_dropoff))              #Tell the crane the job is moving a box in the warehouse. Start adding the pickup and dropoff locations to the bluetooth command list for the stacker crane
        while crane_status != "Picked up": wait(50)                                         #Waiting for the crane to tell the box has been taken away from the input chain conveyor
        outside_dict["location100"]["box"]  = True                                          #Set the box present at the new location (location 100)
        outside_dict["location100"]["name"] = wms_online_list[pos_pickup]                   #Transfer the name from the previous location to the new one (location 0-59 to 100)
        change_one_wms_position(pos_pickup, "No box present")                               #Perform the function that changes 1 WMS position and saves it online+offline
        save_outside_wms()                                                                  #Saving the offline WMS for machine parts and floor storage
        comm_list_uart.append(["transport_pallet", pos_pickup, 100])                        #The message is added to the UART communication waiting list for the ESP32
        while crane_status != "Dropped off": wait(50)                                       #Waiting for the crane to tell the box has been put in the warehouse, the crane is available for a new job now
        change_one_wms_position(pos_dropoff, outside_dict["location100"]["name"])           #Perform the function that changes 1 WMS position and saves it online+offline
        outside_dict["location100"]["name"] = "No box present"                              #Remove the name from the old location (location 100)
        outside_dict["location100"]["box"]  = False                                         #Remove the present status from the old location (location 100)
        save_outside_wms()                                                                  #Saving the offline WMS for machine parts and floor storage
        comm_list_uart.append(["transport_pallet", 100, pos_dropoff])                       #The message is added to the UART communication waiting list for the ESP32

    elif task == "Manual":                                                                  #Check what the stacker crane needs to transfer, this one is for manually driving to a location, without using the telescopic fork TODO[Not used yet]
        comm_list_crane.append("Drive to ,%s"%pos_dropoff)                                  #Tell the crane the job is moving to a location in the warehouse
    

def full_floor_spaces(mode):                                                                #This function checks if there is free dropoff place and returns the location, or returns the amount of occupied places
    counter = 0                                                                             #Defining local variables
    empty_places = []

    for i in range(floor_storage_space):                                                    #Check every floor storage location (locations 110,111,112,113,114)
        if outside_dict["location%s"%positions[-floor_storage_space+i]]["box"] == True:     #If there is a box on this location
            counter +=1                                                                     #Add 1 to the local counter
        elif mode == "freespace": empty_places.append(positions[-floor_storage_space+i])    #If there is no box and the function mode is searching for a free spot, add this location to the local list
    if mode == "freespace": return choice(empty_places)                                     #If the mode is searching for a free location, choose a random one from all the free locations and return the value (locations 110,111,112,113,114)
    counter += inbound                                                                      #If the mode was normal, add to the occupied locations the amount of boxes still being inbound
    return counter                                                                          #Return the amount of occupied floor storage places


def scissor_robot_operation():                                                              #Executing a scissorlift to 6DOF or reverse movement
    global inbound                                                                          #Using these global variables in this local function (if not defined to be global, it will make a local variable)
    global outbound
    global floor_takeout_list
    global conveyors
      
    if inbound > 0:                                                                         #Check if the task is taking a box from the scissorlift to the floor
        comm_list_chain.append("Scissor is up")                                             #A message is added to the bluetooth communication waiting list for the chain conveyor brick that the scissor is no more down
        check_emergency_stop("Conveyors")                                                   #Check if a movement is allowed to start
        while scissorlift.angle() < outside_dict["location106"]["liftheight"] - 50:         #Start a loop until the scissorlift is near 50degrees of finishing the rotation
            scissorlift.run_target(max_speed_scissorlift * max_speed_scissorlift_adj, outside_dict["location106"]["liftheight"], then=Stop.COAST, wait=False)  #Send the scissorlift motor run to the target command, don't wait for finishing
            while scissorlift.angle() < outside_dict["location106"]["liftheight"] - 50:     #Start a loop until the scissorlift is near 50degrees of finishing the rotation
                if emergency_stop == True or conveyors == "Off":                            #If during this loop an emergency stop occurs or the conveyors are turned off
                    scissorlift.stop()                                                      #Stop the scissorlift
                    check_emergency_stop("Conveyors")                                       #Check if a movement is allowed to start
                    break                                                                   #Close this loop, so the motor will restart again
        comm_list_robot.append("Scissor is up")                                             #A message is added to the bluetooth communication waiting list for the robot brick that the scissor is up and ready for pickup (location 106)
        while robot_status != "Ready": wait(100)                                            #Wait for the robot to be finished with the previous task (location 107)
        outside_dict["location107"]["liftposition"] = "performing task"                     #Set the robot dictionary to no more ready, but performing a task now
        wait(100)                                                                           #TODO check if this wait is needed?
        dropoff_loc = full_floor_spaces("freespace")                                        #Request this function to return a free storage place location (locations 110,111,112,113,114)
        command_robot = "Scissor standard to zone {}"                                       #Make a local variable with a string to format TODO delete this variable, this was before I learned about %s formatting
        print(command_robot.format(dropoff_loc))                                            #Print this feedback line when debugging
        comm_list_robot.append(command_robot.format(dropoff_loc))                           #A message is added to the bluetooth communication waiting list for the robot brick where to dropoff the box (location 110,111,112,113,114)
        while robot_status != "Picked up": wait(50)                                         #Wait for the robot to be finished with the pickup (location 106 to 107)
        outside_dict["location107"]["name"] = outside_dict["location106"]["name"]           #Transfer the name from the previous location to the new one (location 106 to 107)
        outside_dict["location107"]["box"]  = True                                          #Set the box present at the new location (location 107)
        outside_dict["location106"]["name"] = "No box present"                              #Remove the name from the old location (location 106)
        outside_dict["location106"]["box"]  = False                                         #Remove the present status from the old location (location 106)
        save_outside_wms()                                                                  #Saving the offline WMS for machine parts and floor storage
        comm_list_uart.append(["transport_pallet", 106, 107])                               #The message is added to the UART communication waiting list for the ESP32
        check_emergency_stop("Conveyors")                                                   #Check if a movement is allowed to start
        while scissorlift.angle() > 50:                                                     #Start a loop until the scissorlift is near 50degrees of finishing the rotation
            scissorlift.run_target(max_speed_scissorlift * max_speed_scissorlift_adj, 0, then=Stop.COAST, wait=False)   #Send the scissorlift motor run to the target command, don't wait for finishing
            while scissorlift.angle() > 50:                                                 #Start a loop until the scissorlift is near 50degrees of finishing the rotation
                if emergency_stop == True or conveyors == "Off":                            #If during this loop an emergency stop occurs or the conveyors are turned off
                    scissorlift.stop()                                                      #Stop the scissorlift
                    check_emergency_stop("Conveyors")                                       #Check if a movement is allowed to start
                    break                                                                   #Close this loop, so the motor will restart again
        outside_dict["location106"]["liftposition"] = "ready down"                          #Change the scissorlift position from up to down
        comm_list_robot.append("Scissor is down")                                           #A message is added to the bluetooth communication waiting list for the robot brick that the scissor is down (location 106)
        comm_list_chain.append("Scissor is down")                                           #A message is added to the bluetooth communication waiting list for the chain conveyor brick that the scissor is down (location 106)
        comm_list_chain.append("Scissor empty")                                             #A message is added to the bluetooth communication waiting list for the chain conveyor brick that the scissor ready for new input (location 106)
        while robot_status != "Ready": wait(50)                                             #Wait for the robot to be finished with the previous task (location 107)
        outside_dict["location%s"%dropoff_loc]["dropofftime"]   = math.floor(timer_floors.time() / 1000)    #Adding to the floor location dictionary the time when it was put down on the floor (seconds)
        outside_dict["location%s"%dropoff_loc]["name"]          = outside_dict["location107"]["name"]       #Transfer the name from the previous location to the new one (location 107 to 110,111,112,113,114)
        outside_dict["location107"]["name"]                     = "No box present"          #Remove the name from the old location (location ...)
        outside_dict["location%s"%dropoff_loc]["box"]           = True                      #Set the box present at the new location (location 110,111,112,113,114)
        outside_dict["location107"]["box"]                      = False                     #Remove the present status from the old location (location 107)
        save_outside_wms()                                                                  #Saving the offline WMS for machine parts and floor storage
        comm_list_uart.append(["transport_pallet", 107, dropoff_loc])                       #The message is added to the UART communication waiting list for the ESP32
        inbound -= 1                                                                        #Set 1 less box going to the robot floor storage
    elif outbound > 0:                                                                      #Check if the task is taking a box from the floor to the scissorlift
        command_robot = "Zone {} to scissor standard"                                       #Make a local variable with a string to format TODO delete this variable, this was before I learned about %s formatting
        print(command_robot.format(floor_takeout_list[0]))                                  #Print this feedback line when debugging
        if outbound > 1: wait(1500)                                                         #Waiting for some reason TODO This was added, removed and had to readd, don't know why 500ms was not enough
        comm_list_robot.append(command_robot.format(floor_takeout_list[0]))                 #A message is added to the bluetooth communication waiting list for the robot brick where to pickup the box (location 110,111,112,113,114)
        while outside_dict["location106"]["box"] == True: wait(100)                         #Wait for the scissorlift to be empty (location 106)
        outside_dict["location106"]["liftposition"] = "moving up"                           #Change the scissorlift position from down to moving
        comm_list_chain.append("Scissor is up")                                             #A message is added to the bluetooth communication waiting list for the chain conveyor brick that the scissor is up (location 106)
        comm_list_chain.append("Prepare input 6dof")                                        #A message is added to the bluetooth communication waiting list for the chain conveyor brick that the scissor preparing for a new box to be taken to the rack (location 106)
        check_emergency_stop("Conveyors")                                                   #Check if a movement is allowed to start
        while scissorlift.angle() < outside_dict["location106"]["liftheight"] - 250 - 50:   #Start a loop until the scissorlift is near 50degrees of finishing the rotation (dropoff is 250 lower than pickup)
            scissorlift.run_target(max_speed_scissorlift * max_speed_scissorlift_adj, outside_dict["location106"]["liftheight"] - 250, then=Stop.COAST, wait=False) #Send the scissorlift motor run to the target command, don't wait for finishing (dropoff is 250 lower than pickup)
            while scissorlift.angle() < outside_dict["location106"]["liftheight"] - 250 - 50:   #Start a loop until the scissorlift is near 50degrees of finishing the rotation (dropoff is 250 lower than pickup)
                if emergency_stop == True or conveyors == "Off":                            #If during this loop an emergency stop occurs or the conveyors are turned off
                    scissorlift.stop()                                                      #Stop the scissorlift
                    check_emergency_stop("Conveyors")                                       #Check if a movement is allowed to start
                    break                                                                   #Close this loop, so the motor will restart again
        comm_list_robot.append("Scissor is up")                                             #A message is added to the bluetooth communication waiting list for the robot brick that the scissor is up and ready for dropoff (location 106)
        while robot_status != "Picked up": wait(50)                                         #Wait for the robot to be finished with the previous task (location 107)
        outside_dict["location%s"%floor_takeout_list[0]]["dropofftime"] = 99999             #Reset to the floor location dictionary the time when it was put down on the floor (seconds 99999)
        outside_dict["location%s"%floor_takeout_list[0]]["box"]         = False             #Remove the present status from the old location (location 110,111,112,113,114)
        outside_dict["location107"]["name"]                             = outside_dict["location%s"%floor_takeout_list[0]]["name"]  #Transfer the name from the previous location to the new one (location 110,111,112,113,114 to 107)
        outside_dict["location107"]["box"]                              = True              #Set the box present at the new location (location 107)
        outside_dict["location%s"%floor_takeout_list[0]]["name"]        = "No box present"  #Remove the name from the old location (floor location 110,111,112,113,114)
        save_outside_wms()                                                                  #Saving the offline WMS for machine parts and floor storage
        comm_list_uart.append(["transport_pallet", floor_takeout_list[0], 107])             #The message is added to the UART communication waiting list for the ESP32
        floor_takeout_list.pop(0)                                                           #Delete the task that was performed
        while robot_status != "Ready": wait(50)                                             #Wait for the robot to be finished with the previous task (location 107)
        outside_dict["location106"]["name"]                             = outside_dict["location107"]["name"]   #Transfer the name from the previous location to the new one (location 107 to 106)
        outside_dict["location107"]["name"]                             = "No box present"  #Remove the name from the old location (location 107)
        outside_dict["location107"]["box"]                              = False             #Remove the present status from the old location (location 107)
        outside_dict["location106"]["box"]                              = True              #Set the box present at the new location (location 106)
        save_outside_wms()                                                                  #Saving the offline WMS for machine parts and floor storage
        comm_list_uart.append(["transport_pallet", 107, 106])                               #The message is added to the UART communication waiting list for the ESP32
        check_emergency_stop("Conveyors")                                                   #Check if a movement is allowed to start
        comm_list_robot.append("Scissor is down")                                           #A message is added to the bluetooth communication waiting list for the robot brick that the scissor is down (location 106)
        comm_list_chain.append("Input from scissor")                                        #A message is added to the bluetooth communication waiting list for the chain conveyor brick that the scissor preparing for a new box to be taken to the rack (location 106)
        while scissorlift.angle() > 50:                                                     #Start a loop until the scissorlift is near 50degrees of finishing the rotation
            scissorlift.run_target(max_speed_scissorlift * max_speed_scissorlift_adj, 0, then=Stop.COAST, wait=False)   #Send the scissorlift motor run to the target command, don't wait for finishing
            while scissorlift.angle() > 50:                                                 #Start a loop until the scissorlift is near 50degrees of finishing the rotation
                if emergency_stop == True or conveyors == "Off":                            #If during this loop an emergency stop occurs or the conveyors are turned off
                    scissorlift.stop()                                                      #Stop the scissorlift
                    check_emergency_stop("Conveyors")                                       #Check if a movement is allowed to start
                    break                                                                   #Close this loop, so the motor will restart again
        outside_dict["location106"]["liftposition"] = "ready down"                          #Change the scissorlift position from up to down
        comm_list_chain.append("Scissor is down")                                           #A message is added to the bluetooth communication waiting list for the chain conveyor brick that the scissor is down (location 106)
        wait(50)                                                                            #TODO is this wait still needed?
        while outside_dict["location106"]["box"] == True: wait(100)                         #Wait for the scissorlift to be empty (location 106)
        if outbound > 0: outbound -= 1                                                      #Set 1 less box coming from the robot floor storage


def scissor_lift_operation():                                                               #TODO This function is used only once at startup, is it needed?
    global outside_dict                                                                     #Using these global variables in this local function (if not defined to be global, it will make a local variable)
    global conveyors

    if robot_used == True or (robot_used == False and outside_dict["location104"]["box"] == True):  #If the robot is used or if there is a box on the input roll conveyor
        comm_list_chain.append("Scissor is up")                                             #A message is added to the bluetooth communication waiting list for the chain conveyor brick that the scissor is up (location 106)
        check_emergency_stop("Conveyors")                                                   #Check if a movement is allowed to start
        while scissorlift.angle() < outside_dict["location106"]["liftheight"] - 50:         #Start a loop until the scissorlift is near 50degrees of finishing the rotation
            scissorlift.run_target(max_speed_scissorlift * max_speed_scissorlift_adj, outside_dict["location106"]["liftheight"], then=Stop.COAST, wait=False)   #Send the scissorlift motor run to the target command, don't wait for finishing
            while scissorlift.angle() < outside_dict["location106"]["liftheight"] - 50:     #Start a loop until the scissorlift is near 50degrees of finishing the rotation
                    if emergency_stop == True or conveyors == "Off":                        #If during this loop an emergency stop occurs or the conveyors are turned off
                        scissorlift.stop()                                                  #Stop the scissorlift
                        check_emergency_stop("Conveyors")                                   #Check if a movement is allowed to start
                        break                                                               #Close this loop, so the motor will restart again
        outside_dict["location106"]["liftposition"] = "ready up"                            #Change the scissorlift position from down to up

    if robot_used == False:                                                                 #If the robot is being used
        while outside_dict["location104"]["box"] == True: wait(200)                         #Wait while there is a box on the input roll conveyor
        check_emergency_stop("Conveyors")                                                   #Check if a movement is allowed to start
        while scissorlift.angle() > 0:                                                      #Start a loop until the scissorlift is down
            scissorlift.run_target(max_speed_scissorlift * max_speed_scissorlift_adj, 0, then=Stop.COAST, wait=False)   #Send the scissorlift motor run to the target command, don't wait for finishing
            while scissorlift.angle() > 0:                                                  #Start a loop until the scissorlift is down
                if emergency_stop == True or conveyors == "Off":                            #If during this loop an emergency stop occurs or the conveyors are turned off
                    scissorlift.stop()                                                      #Stop the scissorlift
                    check_emergency_stop("Conveyors")                                       #Check if a movement is allowed to start
                    break                                                                   #Close this loop, so the motor will restart again           
        outside_dict["location106"]["liftposition"] = "ready down"                          #Change the scissorlift position from up to down
        comm_list_chain.append("Scissor is down")                                           #A message is added to the bluetooth communication waiting list for the chain conveyor brick that the scissor is down (location 106)


def bluetooth_receiver():                                                                   #A loop that checks if there are new bluetooth commands incoming
    global outside_dict                                                                     #Using these global variables in this local function (if not defined to be global, it will make a local variable)
    global crane_status
    global robot_status
    global chain_status

    last_chain_msg = ""                                                                     #Defining local variables to remember the last received message
    last_crane_msg = ""
    last_robot_msg = ""

    while True:                                                                             #Start a forever loop
        new_msg = chain_status_to_conv_mbox.read()                                          #Read the last message received by bluetooth from the chain conveyor brick
        if new_msg != last_chain_msg:                                                       #Compare the last message with the last already received message
            last_chain_msg = new_msg                                                        #Store the new message as the received message
            print("incoming message from chain brick: %s."%new_msg)                         #Print this feedback line when debugging
            if   last_chain_msg                             == "Chain out empty":           #Compare the received message
                if outside_dict["location101"]["box"]       == True:                        #If there is a box at this location
                    outside_dict["location102"]["name"]     =  outside_dict["location101"]["name"]  #Transfer the name from the previous location to the new one (location 101 to 102)
                    outside_dict["location101"]["name"]     = "No box present"              #Remove the name from the old location (location 101)
                    outside_dict["location101"]["box"]      = False                         #Remove the present status from the old location (location 101)
            elif last_chain_msg                             == "Roll out full":             #Compare the received message
                if outside_dict["location102"]["box"]       == False:                       #If there is no box at this location
                    outside_dict["location102"]["box"]      = True                          #Set the box present at the new location (location 102)
                    comm_list_uart.append(["transport_pallet", 101, 102])                   #The message is added to the UART communication waiting list for the ESP32
            elif last_chain_msg                             == "Chain in full from corner": #Compare the received message
                if outside_dict["location105"]["box"]       == False:                       #If there is no box at this location
                    outside_dict["location105"]["name"]     = outside_dict["location104"]["name"]   #Transfer the name from the previous location to the new one (location 104 to 105)
                    outside_dict["location105"]["box"]      = True                          #Set the box present at the new location (location 105)
                    outside_dict["location104"]["name"]     = "No box present"              #Remove the name from the old location (location 104)
                    comm_list_uart.append(["transport_pallet", 104, 105])                   #The message is added to the UART communication waiting list for the ESP32
            elif last_chain_msg                             == "Roll in empty":             #Compare the received message
                if outside_dict["location104"]["box"]       == True:                        #If there is a box at this location
                    outside_dict["location104"]["box"]      = False                         #Remove the present status from the old location (location 104)
            elif last_chain_msg                             == "Chain in full from scissor":    #Compare the received message
                if outside_dict["location105"]["box"]       == False:                       #If there is no box at this location
                    outside_dict["location104"]["box"]      = True                          #Set the box present at the new location (location 104) #To ensure waiting for the corner transfer to be up to continue
                    outside_dict["location105"]["name"]     = outside_dict["location106"]["name"]   #Transfer the name from the previous location to the new one (location 106 to 105)
                    outside_dict["location105"]["box"]      = True                          #Set the box present at the new location (location 105)
                    outside_dict["location106"]["name"]     = "No box present"              #Remove the name from the old location (location 106)
                    outside_dict["location106"]["box"]      = False                         #Remove the present status from the old location (location 106)
                    comm_list_uart.append(["transport_pallet", 106, 105])                   #The message is added to the UART communication waiting list for the ESP32
            elif last_chain_msg                             == "Scissor full":              #Compare the received message
                if outside_dict["location106"]["box"]       == False:                       #If there is no box at this location
                    outside_dict["location106"]["name"]     = outside_dict["location104"]["name"]   #Transfer the name from the previous location to the new one (location 104 to 106)
                    outside_dict["location106"]["box"]      = True                          #Set the box present at the new location (location 106)
                    outside_dict["location104"]["name"]     = "No box present"              #Remove the name from the old location (location 104)
                    comm_list_uart.append(["transport_pallet", 104, 106])                   #The message is added to the UART communication waiting list for the ESP32
            elif last_chain_msg                             == "Homing finished":           #Compare the received message
                    comm_list_uart.append(["update_mode", 0, "Homing chain conveyors not finished"])    #The message is added to the UART communication waiting list for the ESP32
                    chain_status = "Ready"                                                  #Change the state for the chain conveyor brick to ready, ready to operate tasks
            elif last_chain_msg                             == "Homing unfinished":         #Compare the received message
                    comm_list_uart.append(["update_mode", 1, "Homing chain conveyors not finished"])    #The message is added to the UART communication waiting list for the ESP32
                    chain_status = "Homing"                                                 #Change the state for the chain conveyor brick to homing, unable to operate tasks
            elif last_chain_msg                             == "Outp not free error":       #Compare the received message
                    comm_list_uart.append(["update_mode", 1, "Output chain not empty"])     #The message is added to the UART communication waiting list for the ESP32
            elif last_chain_msg                             == "Outp not full error":       #Compare the received message
                    comm_list_uart.append(["update_mode", 1, "Output chain not full"])      #The message is added to the UART communication waiting list for the ESP32
            elif last_chain_msg                             == "Inp not free error":        #Compare the received message
                    comm_list_uart.append(["update_mode", 1, "Input chain not empty"])      #The message is added to the UART communication waiting list for the ESP32
            elif last_chain_msg                             == "Inp not full error":        #Compare the received message
                    comm_list_uart.append(["update_mode", 1, "Input chain not full"])       #The message is added to the UART communication waiting list for the ESP32
            elif last_chain_msg                             == "Input received":            #Compare the received message
                    comm_list_crane.append("Input received")                                #Tell the crane the job is finished, box was well received. Message is added to the bluetooth command list for the stacker crane
            
            save_outside_wms()                                                              #Saving the offline WMS for machine parts and floor storage
        wait(20)                                                                            #Breathing time for the EV3

        new_msg = robot_status_to_conv_mbox.read()                                          #Read the last message received by bluetooth from the robot brick
        if new_msg != last_robot_msg:                                                       #Compare the last message with the last already received message
            last_robot_msg = new_msg                                                        #Store the new message as the received message
            print("incoming message from robot brick: %s."%last_robot_msg)                  #Print this feedback line when debugging
            if   last_robot_msg                                == "Ready":                  #Compare the received message
                robot_status = "Ready"                                                      #Change the state for the robot brick to ready, ready to operate tasks
            elif last_robot_msg                                == "Picked up":              #Compare the received message
                robot_status = "Picked up"                                                  #Change the state for the robot brick that it picked up the box for the current task
            elif last_robot_msg                                == "Homing finished":        #Compare the received message
                comm_list_uart.append(["update_mode", 0, "Homing robot not finished"])      #The message is added to the UART communication waiting list for the ESP32
                robot_status = "Ready"                                                      #Change the state for the robot brick to ready to operate tasks
            elif last_robot_msg                                == "Homing unfinished":      #Compare the received message
                comm_list_uart.append(["update_mode", 1, "Homing robot not finished"])      #The message is added to the UART communication waiting list for the ESP32
                robot_status = "Homing"                                                     #Change the state for the robot brick to homing, unable to operate tasks
        wait(20)                                                                            #Breathing time for the EV3

        new_msg = crane_status_to_conv_mbox.read()                                          #Read the last message received by bluetooth from the stacker crane brick
        if new_msg != last_crane_msg:                                                       #Compare the last message with the last already received message
            last_crane_msg = new_msg                                                        #Store the new message as the received message
            print("incoming message from crane brick: %s."%last_crane_msg)                  #Print this feedback line when debugging
            if   last_crane_msg                                == "Ready":                  #Compare the received message
                crane_status = "Ready"                                                      #Change the state for the stacker crane brick to ready, ready to operate tasks
            elif last_crane_msg                                == "Picked up":              #Compare the received message
                crane_status = "Picked up"                                                  #Change the state for the stacker crane brick that it picked up the box for the current task
            elif last_crane_msg                                == "Dropped off":            #Compare the received message
                crane_status = "Dropped off"                                                #Change the state for the stacker crane brick that it dropped off the box for the current task
            elif last_crane_msg                                == "Homing finished":        #Compare the received message
                comm_list_uart.append(["update_mode", 0, "Homing stacker crane not finished"])  #The message is added to the UART communication waiting list for the ESP32
                crane_status = "Ready"                                                      #Change the state for the stacker crane brick to ready, ready to operate tasks
            elif last_crane_msg                                == "Homing unfinished":      #Compare the received message
                comm_list_uart.append(["update_mode", 1, "Homing stacker crane not finished"])  #The message is added to the UART communication waiting list for the ESP32
                crane_status = "Homing"                                                     #Change the state for the stacker crane brick to homing, unable to operate tasks
            elif last_crane_msg                                == "Homing fork error":      #Compare the received message
                comm_list_uart.append(["update_mode", 1, "Homing stacker crane fork error"])    #The message is added to the UART communication waiting list for the ESP32
            elif last_crane_msg                                == "Crane sensor remains pushed error":  #Compare the received message
                comm_list_uart.append(["update_mode", 1, "Homing stacker crane lift or drive error"])   #The message is added to the UART communication waiting list for the ESP32
            elif last_crane_msg                                == "Crane positioning error":    #Compare the received message
                comm_list_uart.append(["update_mode", 1, "Positioning stacker crane error"])    #The message is added to the UART communication waiting list for the ESP32
        wait(20)                                                                            #Breathing time for the EV3
     

def change_one_wms_position(location, new_name):                                            #Perform the function that changes 1 WMS position and saves it online+offline
    global wms_online_list                                                                  #Using this global variable in this local function (if not defined to be global, it will make a local variable)

    wms_online_list[location] = new_name                                                    #Transfer the name for the new location to the wms online list (location 0,1,2,...,99 / namestring)
    save_offline_wms()                                                                      #Save the updated online list to the offline .txt file, ready to be used if the brick is shut down now


def wait_for_release_buttons():                                                             #A loop that checks if all buttons on the EV3 brick are released
    while ev3.buttons.pressed() != []: continue                                             #If not released, keep waiting


def alarm_lights():                                                                         #A loop that checks if the emergency state is still on and flashes the EV3 brick lights or turns them green
    global emergency_stop                                                                   #Using this global variable in this local function (if not defined to be global, it will make a local variable)

    while emergency_stop == True:                                                           #If during this loop an emergency stop remains
        ev3.light.off()                                                                     #Turn the lights off on the brick
        wait(250)                                                                           #Wait 250ms
        ev3.light.on(Color.RED)                                                             #Turn the lights red (flashing at 2Hz)
        wait(250)                                                                           #Wait 250ms
    ev3.light.on(Color.GREEN)                                                               #If the emergency state is finished, turn the green EV3 lights on


def startupsequence():                                                                      #This function is called at startup of the EV3 brick
    global connections                                                                      #Using these global variables in this local function (if not defined to be global, it will make a local variable)
    global mode_chosen

    for i in range(total_storage_positions):                                                #Check every rack storage location (locations 0,1,2,...,99)
        if wms_online_list[i] == "No box present": continue                                 #If there is no box at this storage location, continue the for loop
        else: update_WMS_ESP(i, 1, wms_online_list[i])                                      #If there is a box, send the location and name of the pallet by a function and UartRemote
        wait(100)                                                                           #Wait 100ms TODO check if this wait can be deleted
    for i in positions:                                                                     #Check every conveyor and floor storage location (locations 100,101,102,103,104,105,106,107,110,111,112,113,114)
        if outside_dict["location%s"%i]["box"] == True:                                     #If there is a box at this location
            update_WMS_ESP(i, 1, outside_dict["location%s"%i]["name"])                      #Send the location and name of the pallet by a function and UartRemote
        wait(100)                                                                           #Wait 100ms TODO check if this wait can be deleted
    comm_list_uart.append(["update_mode", 0, "Show connection screen"])                     #The message is added to the UART communication waiting list for the ESP32

    ev3.screen.draw_text(4,  2, "Select operating mode", text_color=Color.BLACK, background_color=Color.WHITE)  #Write a line of text on the EV3 screen
    while mode_chosen == False: ur.process_uart()                                           #Process incoming commands by UART until the mode has been chosen on the screen
    ev3.screen.draw_text(4,  2, "Connecting to %s devices.                                "%connections, text_color=Color.BLACK, background_color=Color.WHITE)  #Write a line of text on the EV3 screen, how many devices will be connecting by bluetooth
    comm_list_uart.append(["update_mode", 0, "Mode not selected"])                          #The message is added to the UART communication waiting list for the ESP32

    ##########ALWAYS START THIS SERVER-BRICK FIRST. THEN START THE SLAVE-BRICKS, OR THEY WILL TIMEOUT IF THEY CAN NOT CONNECT TO THIS BRICK BY BLUETOOTH (THIS PROGRAM NEEDS TO BE RUNNING##########
    server_roll.wait_for_connection(connections)                                            #Waiting for the amount of bluetooth devices connected
    ev3.screen.draw_text(4,  2, "Connected to %s devices.                                "%connections, text_color=Color.BLACK, background_color=Color.WHITE)   #Write a line of text on the EV3 screen

    comm_list_uart.append(["update_mode", 0, "Communication not online"])                   #The message is added to the UART communication waiting list for the ESP32
    wait(1000)                                                                              #Wait 1second to give the ESP32 program time to update all its functions
    comm_list_uart.append(["update_mode", 0, "WMS Data not received"])                      #The message is added to the UART communication waiting list for the ESP32

    if robot_used == True:                                                                  #Depending on the current states send messages, this state is if the 6DOF is being used
        comm_list_chain.append("Robot used")                                                #A message is added to the bluetooth communication waiting list for the chain conveyor brick that the robot mode is used (location 107)
        comm_list_robot.append("Robot used")                                                #A message is added to the bluetooth communication waiting list for the robot brick that the robot mode is used (location 107)
    if outside_dict["location107"]["box"] == True: comm_list_robot.append("Robot full")     #A message is added to the bluetooth communication waiting list for the robot brick that the robot holds a box (location 107) [Not used, it can't hold a box whilst homing]
    if outside_dict["location106"]["box"] == True:                                          #Depending on the current states send messages, this state is for if there is a box on the scissorlift (location 106)
        comm_list_chain.append("Input from scissor")                                        #A message is added to the bluetooth communication waiting list for the chain conveyor brick that the scissor is down and has a box to be transported (location 106)
        comm_list_robot.append("Scissor full")                                              #A message is added to the bluetooth communication waiting list for the robot brick that the scissor is full (location 106)
    if outside_dict["location105"]["box"] == True: comm_list_chain.append("Chain in full")  #A message is added to the bluetooth communication waiting list for the chain conveyor brick that the input  chain conveyor has a box (location 105)
    if outside_dict["location104"]["box"] == True: comm_list_chain.append("Roll in full")   #A message is added to the bluetooth communication waiting list for the chain conveyor brick that the input  roll  conveyor has a box (location 104)
    if outside_dict["location102"]["box"] == True: comm_list_chain.append("Roll out full")  #A message is added to the bluetooth communication waiting list for the chain conveyor brick that the output roll  conveyor has a box (location 102)
    if outside_dict["location101"]["box"] == True: comm_list_chain.append("Chain out full") #A message is added to the bluetooth communication waiting list for the chain conveyor brick that the output chain conveyor has a box (location 101)
    if outside_dict["location100"]["box"] == True: comm_list_crane.append("Crane full")     #A message is added to the bluetooth communication waiting list for the crane brick that the crane has a box on the telescopic fork (location 106)

    sub_alarm_lights.start()                                                                #Start a sub-thread for flashing a red light if the emergency state is true
    

def homing_scissorlift():                                                                   #This function is called at startup of the EV3 brick after bluetooth connections are working, and it
    global scissorlift_homing                                                               #Using these global variables in this local function (if not defined to be global, it will make a local variable)
    global outside_dict

    scissorlift.run_until_stalled(-500, then=Stop.HOLD, duty_limit=40)                      #Start turning the scissorlift motor at maximal -500degrees/second (downwards), maximal 40% torque used. Wait until the motor is stalled and hold position (Homing)
    scissorlift.reset_angle(scissorlift_homing)                                             #Reset the scissorlift motor angle to a preset value, lower than 0, so in normal operation the motor will never reach this endstop
    scissorlift.run_target(1400, 0, then=Stop.COAST, wait=True)                             #Send the scissorlift motor run to the target command (position 0degrees), wait for finishing
    outside_dict["location106"]["liftposition"] = "ready down"                              #Change the scissorlift position to down in the dictionary
    comm_list_chain.append("Scissor is down")                                               #A message is added to the bluetooth communication waiting list for the chain conveyor brick that the scissorlift is in the down position (location 106)
    comm_list_robot.append("Scissor is down")                                               #A message is added to the bluetooth communication waiting list for the robot brick that the scissorlift is in the down position (location 106)
    comm_list_uart.append(["update_mode", 0, "Homing scissorlift not finished"])            #The message is added to the UART communication waiting list for the ESP32


##########~~~~~~~~~~CREATING MULTITHREADS~~~~~~~~~~##########
sub_conveyor_auto           =   Thread(target=conveyor_transfer_auto)                       #This creates a thread, made from a previously defined function. No arguments can be given
sub_crane_auto              =   Thread(target=crane_auto)
sub_bluetooth_receiver      =   Thread(target=bluetooth_receiver)
sub_scissor_lift_operation  =   Thread(target=scissor_lift_operation)
sub_scissor_robot_operation =   Thread(target=scissor_robot_operation)
sub_communication_bt_uart   =   Thread(target=communication_bt_uart)
sub_alarm_lights            =   Thread(target=alarm_lights)
sub_homing_scissorlift      =   Thread(target=homing_scissorlift)


##########~~~~~~~~~~PROGRAM STARTING, STARTUP ALL BLUETOOTH RX AND TX, AND UART TX~~~~~~~~~~##########
sub_bluetooth_receiver.start()                                                              #This starts the loop thread that receives all the bluetooth communication from the slave EV3 bricks. Non-blocking
sub_communication_bt_uart.start()                                                           #This starts the loop thread that sends all the communication to the slave EV3 bricks by bluetooth and the ESP32 by UART. Non-blocking


##########~~~~~~~~~~WAIT UNTIL (0, 2 or 3) BLUETOOTH DEVICES ARE CONNECTED~~~~~~~~~~##########
startupsequence()                                                                           #This calls the function for startup. Blocking


##########~~~~~~~~~~EV3 HAS INCREMENTAL ENCODERS IN IT'S MOTORS, SO IT'S NEEDED TO CALIBRATE EVERY STARTUP~~~~~~~~~~##########
sub_homing_scissorlift.start()                                                              #This starts the thread that performs a homing for the EV3 motors. Non-blocking


##########~~~~~~~~~~PROGRAM RUNNING THE LOGISTICS CENTER~~~~~~~~~~##########
sub_scissor_lift_operation.start()                                                          #This starts the thread that checks if a box needs extra care taken away from the scissorlift at startup. Non-blocking TODO check if still needed
sub_conveyor_auto.start()                                                                   #This starts the loop thread that controls the 3 roller conveyors and the scissor table + makes tasks for the 6DoF. Non-blocking
sub_crane_auto.start()                                                                      #This starts the loop thread that controls the high bay crane, it sends tasks for pickup and dropoff. Non-blocking


while True:                                                                                 #Main program, it gets UART commands from the ESP32 and handles the emergency button/reset
    wait(50)                                                                                #Breathing time for the EV3
    ur.process_uart()                                                                       #Check if there is an UART incoming message. Non-blocking
    if touch_input_emerg.pressed() == True:                                                 #Check if the emergency button is pressed in at this moment
        if emergency_stop == False:                                                         #Check if the emergency state is not already set
            emergency_stop = True                                                           #Set the emergency state
            comm_list_uart.append(["update_mode", 1, "Emergency Stop pushed"])              #The message is added to the UART communication waiting list for the ESP32
            sub_alarm_lights.start()                                                        #This starts the loop thread that controls the red flashing EV3 lights. Non-blocking
            comm_list_chain.append("Emergency stop pushed")                                 #A message is added to the bluetooth communication waiting list for the chain conveyor brick that the emergency state is set
            comm_list_crane.append("Emergency stop pushed")                                 #A message is added to the bluetooth communication waiting list for the stacker crane brick that the emergency state is set
            comm_list_robot.append("Emergency stop pushed")                                 #A message is added to the bluetooth communication waiting list for the robot brick that the emergency state is set
    if touch_input_reset.pressed() == True:                                                 #Check if the reset emergency button is pressed in at this moment
        if connections == 3 and not(chain_status == "Homing" or crane_status == "Homing" or robot_status == "Homing") or connections == 2 and not(chain_status == "Homing" or crane_status == "Homing") or connections == 0:    #Check if all connected devices have finished homing
            if emergency_stop == True and not touch_input_emerg.pressed() == True:          #If the emergency stop state is True and not currently pressed, then send reset to all controllers
                comm_list_uart.append(["update_mode", 0, "Emergency Stop pushed"])          #The message is added to the UART communication waiting list for the ESP32
                emergency_stop = False                                                      #Reset the emergency state
                comm_list_chain.append("Emergency stop reset")                              #A message is added to the bluetooth communication waiting list for the chain conveyor brick that the emergency state is reset
                comm_list_crane.append("Emergency stop reset")                              #A message is added to the bluetooth communication waiting list for the stacker crane brick that the emergency state is reset
                comm_list_robot.append("Emergency stop reset")                              #A message is added to the bluetooth communication waiting list for the robot brick that the emergency state is reset
    
    ##########USED FOR DEBUGGING OR YOU CAN ADD MANUAL COMMANDS TRIGGERED BY EV3 BUTTONS##########
    if   ev3.buttons.pressed() == [Button.UP]:                                              #Check if the 'Up' button is pressed on the EV3 brick       [Used for debugging, add commands here to trigger on demand]
        wait_for_release_buttons()                                                          #Wait for all buttons to be released

    elif ev3.buttons.pressed() == [Button.DOWN]:                                            #Check if the 'Down' button is pressed on the EV3 brick     [Used for debugging, add commands here to trigger on demand]
        wait_for_release_buttons()                                                          #Wait for all buttons to be released
        comm_list_robot.append("Scissor standard to zone 111")                              #Debugging the robot, send it a command to pickup a box from the scissorlift and dropoff on the second floor spot (location 111)

    elif ev3.buttons.pressed() == [Button.LEFT]:                                            #Check if the 'Left' button is pressed on the EV3 brick     [Used for debugging, add commands here to trigger on demand]
        wait_for_release_buttons()                                                          #Wait for all buttons to be released

    elif ev3.buttons.pressed() == [Button.RIGHT]:                                           #Check if the 'Right' button is pressed on the EV3 brick    [Used for debugging, add commands here to trigger on demand]
        wait_for_release_buttons()                                                          #Wait for all buttons to be released
        comm_list_uart.append(["update_mode", 0, "Communication not online"])               #The messages are added to the UART communication waiting list for the ESP32
        comm_list_uart.append(["update_mode", 0, "Homing robot not finished"])
        comm_list_uart.append(["update_mode", 0, "Homing chain conveyors not finished"])
        comm_list_uart.append(["update_mode", 0, "Homing stacker crane not finished"])

    elif ev3.buttons.pressed() == [Button.CENTER]:                                          #Check if the 'Center' button is pressed on the EV3 brick   [Used for debugging, add commands here to trigger on demand]
        wait_for_release_buttons()                                                          #Wait for all buttons to be released


##########~~~~~~~~~~EXTRA'S NOT USED IN NORMAL PROGRAM, FOR SAVING PURPOSES~~~~~~~~~~##########                                                                        
ev3.speaker.say("Thank you for watching, subscribe for more Technic Machines")              #The EV3 speaker will read out this textstring
