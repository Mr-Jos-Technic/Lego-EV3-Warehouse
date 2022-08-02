# import libraries
from machine import I2S, Pin
import espidf as esp
import lvgl as lv
from ili9XXX import ili9341,LANDSCAPE
from xpt2046 import xpt2046

from machine import UART
from machine import Pin
from uartremote import *
import time
import esp32
import math
from neopixel import NeoPixel
from imagetools import get_png_info, open_png
import usys as sys

print("Temperature of the ESP32 chip ", (esp32.raw_temperature() - 32) * 5 / 9)

np_status = NeoPixel(Pin(22),6) #Toren lamp, 6 LEDs
np_wms = NeoPixel(Pin(21),108)  #Magazijn, 108 LEDs

baudrate=115200
rx_pin=18
tx_pin=19
port=1
uart=UART(port,baudrate=baudrate,rx=rx_pin,tx=tx_pin,timeout=1)
ur = UartRemote()

#init display
disp = ili9341( miso=12, mosi=13, clk=14, cs=15, dc=23, rst=25, backlight=-1,power=-1,width=320, height=240, rot=LANDSCAPE)
# use same SPI as display, init touch
touch = xpt2046(spihost=esp.HSPI_HOST,cs=26,transpose=False,cal_x0=3865, cal_y1=329, cal_x1=399, cal_y0=3870)
# enable backlight
l=Pin(2,Pin.OUT)
l.value(1) #1


#Warehouse rack configuration
rows = 5
levels = 6
total_wh_positions = rows*levels*2
rack_nr =[]
for i in range(total_wh_positions): rack_nr.append(str(i))
#Pallet locations configuration
pallets_dict       = {}
for i in range(total_wh_positions): pallets_dict.update({"location%s"%i : {"position": i, "box": False, "name": "No box present", "request": False}})
conveyor_dict      = {}
conveyors_nr       = ["100", "101", "102", "103", "104", "105", "106", "107"]
for i in conveyors_nr: conveyor_dict.update({"location%s"%i : {"position": i, "box": False, "name": "No box present"}})
robot_dict         = {}
robot_floor        = ["110", "111", "112", "113", "114"]
for i in robot_floor: robot_dict.update({"location%s"%i : {"position": i, "box": False, "name": "No box present", "request": False}})
#Variables configuration
hb_crane_input     = "Off"
hb_crane_output    = "Off"
conveyors          = "Off"
output             = "Return directly"   #Return directly, scissor lift
robot_used         = False
emergency_stop     = True
machine_errors     = ["Emergency Stop pushed", "Mode not selected", "Communication not online"]
last_shown_errors  = []
mode_request       = 0                   #Time when last mode change has been requested
robot_homing_start = "Not started"
uart_started       = False
selection          = "Debug testing master"

last_event_modes       = None
last_event_modes_time  = 0
last_event_errors      = None
last_event_errors_time = 0

cont_btns = ["Crane input automatic", "Crane input manual",
                 "Crane output automatic", "Crane output manual",
                 "Robot used", "Conveyors on"]

red_wh            = (5, 0, 0)    #10,0,0
green_wh          = (0, 5, 0)    #0,10,0
request_wh        = (5, 5, 5)  #10,10,10

error_info = {"Emergency Stop pushed" :                    {"info" : "The emergency button has been pushed, press the blue reset button to reset this emergency state", "actions" : None},
              "Communication not online" :                 {"info" : "The communication with the master EV3 is not working, turn on the EV3 program", "actions" : None},
              "WMS Data not received" :                    {"info" : "The WMS (Warehouse Management System) data is not received yet to show pallet positions", "actions" : None},
              "Homing stacker crane not finished" :        {"info" : "The stacker crane has motors with incremental encoders, they need to find their home positions first", "actions" : None},
              "Homing robot not finished" :                {"info" : "The robot has motors with incremental encoders, they need to find their home positions first", "actions" : None},
              "Homing chain conveyors not finished" :      {"info" : "The chain conveyors have motors with incremental encoders, they need to find their home positions first", "actions" : None},
              "Homing scissorlift not finished" :          {"info" : "The scissorlift has a motor with incremental encoder, it needs to find his home position first", "actions" : None},
              "Homing stacker crane fork error" :          {"info" : "The fork has not centered correctly, move it near the center position and clear the error", "actions" : ["Try again", "Close"]},
              "Homing stacker crane lift or drive error" : {"info" : "The homing touch sensor remains pushed, check the basket lifting string or rail end buffer", "actions" : ["Try again", "Close"]},
              "Output chain not empty" :                   {"info" : "The output chain conveyor should have been empty, but it is not", "actions" : ["Try again", "Close"]},
              "Output chain not full" :                    {"info" : "The output chain conveyor should have been full, but it is not", "actions" : ["Try again", "Close"]},
              "Input chain not empty" :                    {"info" : "The input chain conveyor should have been empty, but it is not", "actions" : ["Try again", "Close"]},
              "Input chain not full" :                     {"info" : "The input chain conveyor should have been full, but it is not", "actions" : ["Try again", "Close"]},
              "Positioning stacker crane error" :          {"info" : "The stacker crane did not reach its driving position correctly", "actions" : ["Try again", "Close"]},
              "Mode not selected" :                        {"info" : "At the homepage select which machine parts will be used", "actions" : None},
              "Extra error" : {"info" : "", "actions" : None}}

lv.img.cache_set_size(2) #Select the amount of images that will remain in cache, if lower than the amount used it will remove the shortest to load, and reload it when needed
#Creating a PNG decoder
decoder = lv.img.decoder_create()
decoder.info_cb = get_png_info
decoder.open_cb = open_png
#Open a PNG file to use it as background later
##############################OPENING PNG WITH TRIMETRIC VIEW FROM THE WAREHOUSE##############################
try:
    with open('../../../robot_320.png','rb') as f:
        png_data = f.read()
except:
    print("Not found the image")
    
robot_320_img = lv.img_dsc_t({
    'data_size': len(png_data),
    'data': png_data 
})
##############################OPENING PNG WITH TOPVIEW FROM THE WAREHOUSE##############################
try:
    with open('../../../Top_view_400_231.png','rb') as f:
        png_data = f.read()
except:
    print("Not found the image")
    
top_view_img = lv.img_dsc_t({
    'data_size': len(png_data),
    'data': png_data 
})


##############################CALCULATE THE POSITION FOR EACH LED CORRESPONDING TO THE WAREHOUSE LOCATION (LED 0-108 VS LOCATION 0-59)##############################
wh_positions_led  = [] #Define where every LED is on the strip for each storage position
def calculate_wh_positions_on_led_strip(levels, rows): #Lamps run in a loop from bottom to top, and back down. Left lamp = front rack, middle lamp = backside rack
    counter = 0                #Counting 1 row (front and back rack = 1)
    for x in range(100):       #2 Racks per number
        for y in range(2):     #Lamp left of pilar at 0 (one full row), right side at 1 (One full row, no left at first pilar)
            for z in range(2): #Front row at 0, back row at 1
                for i in range(levels):
                    if x == 0 and y == 0: continue #Skipping first row as there are no lights on the first pillar.
                    wh_positions_led.append((x * levels * 2 * 3) - 3 + (3 * y) + (- i*3 + y*2*i*3) + (1 * z))
            counter += 1
            if counter > rows: return
calculate_wh_positions_on_led_strip(levels, rows)


def light_tower_check(): #Function to check which light should be turned on completely, and which slightly on the light tower (so each color remains visible)
    if hb_crane_input == "Off" and hb_crane_output == "Off" and conveyors == "Off":
        np_status[4]=(40, 0, 0)
        np_status[2]=( 0, 2, 0)
    if mode_request > time.ticks_ms() - 2000 and math.fmod(time.ticks_ms(), 1000) > 500 :
        np_status[5]=(25,25,25)
    else:
        np_status[5]=( 4, 4, 4)
    if hb_crane_input == "Automatic" or hb_crane_input == "Manual" or hb_crane_output == "Automatic" or hb_crane_output == "Manual" or conveyors == "Automatic" or conveyors == "Manual":
        np_status[4]=( 2, 0, 0)
        np_status[2]=( 0,40, 0)
    if "Emergency Stop pushed" in machine_errors:
        np_status[4]=(40, 0, 0)
        np_status[2]=( 0, 2, 0)
        if math.fmod(time.ticks_ms(), 2000) > 1000: np_status[1]=( 0, 0,40)
        else:                                       np_status[1]=( 0, 0, 2)
    else:                                           np_status[1]=( 0, 0, 2)
    if len(machine_errors) > 0 and "Emergency Stop pushed" not in machine_errors or len(machine_errors) > 1:
        np_status[3]=(35,20, 0)
    else:
        np_status[3]=( 2, 2, 0)
    np_status.write()

#np_status[5]=(4,4,4)  #white  user interference ongoing
#np_status[4]=(40,0,0) #red    not running
#np_status[3]=(2,2,0)  #orange error with machine needs resolving
#np_status[2]=(0,2,0)  #green  machine (partially) in auto/manual modes
#np_status[1]=(0,0,2)  #blue   reset of emergency system required
#np_status[0]= Not used


def wh_location_states_check(): #Coloring the LED's in the warehouse to correspond to their box state/request
    for i in pallets_dict:
        if pallets_dict[i]["box"] == True: np_wms[wh_positions_led[pallets_dict[i]["position"]]] = red_wh
        else:                              np_wms[wh_positions_led[pallets_dict[i]["position"]]] = green_wh
        if pallets_dict[i]["request"] == True:
            if math.fmod(time.ticks_ms(), 2000) > 1000: np_wms[wh_positions_led[pallets_dict[i]["position"]]] = request_wh
    np_wms.write()

##############################UART COMMUNICATION DEFINITIONS##############################
#Changing the WMS data if a pallet location update is received by UART, this includes the name in a encoded string
def update_storage(loc, state, name):
    global pallets_dict
    global conveyor_dict
    global robot_dict
        
    #print(loc, state, name)
    if 0 <= loc < 100:
        if state == 0:     #Box has been removed from the warehouse racks
            pallets_dict["location%s"%loc]["box"] = False
            pallets_dict["location%s"%loc]["name"] = "No box present"
            pallets_dict["location%s"%loc]["request"] = False
            for i in range(total_wh_positions):
                child_pos = list_request.get_child(i).get_text().split(", ")
                if int(child_pos[0]) == loc:
                    list_request.get_child(i).delete()
                    break
            cont_storeto.get_child(59-loc).clear_state(lv.STATE.DISABLED | lv.STATE.CHECKED)
            list_remove_wms.get_child(loc).clear_state(lv.STATE.CHECKED)
            list_remove_wms.get_child(loc).add_flag(lv.obj.FLAG.HIDDEN)
        elif state == 1:   #Box has been put in the warehouse racks
            if type(name) == bytes: pallets_dict["location%s"%loc]["name"] = name.decode()
            else: pallets_dict["location%s"%loc]["name"] = name
            print("Will remove the request state next, then turn LED red")
            pallets_dict["location%s"%loc]["request"] = False
            if pallets_dict["location%s"%loc]["box"] == False:
                checkbox_request_wh = lv.checkbox(list_request)
                checkbox_request_wh.set_text(str(loc) + ", " + pallets_dict["location%s"%loc]["name"])
                checkbox_request_wh.add_event_cb(update_request_list, lv.EVENT.CLICKED, None)
            pallets_dict["location%s"%loc]["box"] = True
            cont_storeto.get_child(59-loc).add_state(lv.STATE.CHECKED | lv.STATE.DISABLED)
            list_remove_wms.get_child(loc).clear_state(lv.STATE.CHECKED)
            list_remove_wms.get_child(loc).set_text(str(pallets_dict["location%s"%loc]["position"]) + ", " + pallets_dict["location%s"%loc]["name"])
            list_remove_wms.get_child(loc).clear_flag(lv.obj.FLAG.HIDDEN)
    elif 100 <= loc < 110:
        if state == 0:
            conveyor_dict["location%s"%loc]["box"] = False
            conveyor_dict["location%s"%loc]["name"] = "No box present"
            cont_overview_conv.get_child(conveyors_nr.index(str(loc)) +1).clear_state(lv.STATE.CHECKED)
            list_remove_wms.get_child(loc-100+total_wh_positions).clear_state(lv.STATE.CHECKED)
            list_remove_wms.get_child(loc-100+total_wh_positions).add_flag(lv.obj.FLAG.HIDDEN)
        elif state == 1:
            if type(name) == bytes: conveyor_dict["location%s"%loc]["name"] = name.decode()
            else: conveyor_dict["location%s"%loc]["name"] = name
            conveyor_dict["location%s"%loc]["box"] = True
            cont_overview_conv.get_child(conveyors_nr.index(str(loc))+1).add_state(lv.STATE.CHECKED)
            list_remove_wms.get_child(loc-100+total_wh_positions).clear_state(lv.STATE.CHECKED)
            list_remove_wms.get_child(loc-100+total_wh_positions).set_text(str(conveyor_dict["location%s"%loc]["position"]) + ", " + conveyor_dict["location%s"%loc]["name"])
            list_remove_wms.get_child(loc-100+total_wh_positions).clear_flag(lv.obj.FLAG.HIDDEN)
    elif 110 <= loc < 120:
        if state == 0:
            robot_dict["location%s"%loc]["box"] = False
            robot_dict["location%s"%loc]["name"] = "No box present"
            robot_dict["location%s"%loc]["request"] = False
            cont_overview_conv.get_child(robot_floor.index(str(loc))+1+len(conveyors_nr)).clear_state(lv.STATE.CHECKED)
            for i in range(len(robot_floor)):
                child_pos = list_robotsto.get_child(i).get_text().split(", ")
                if int(child_pos[0]) == loc:
                    list_robotsto.get_child(i).delete()
                    break
            list_remove_wms.get_child(loc-110+total_wh_positions+len(conveyors_nr)).clear_state(lv.STATE.CHECKED)
            list_remove_wms.get_child(loc-110+total_wh_positions+len(conveyors_nr)).add_flag(lv.obj.FLAG.HIDDEN)
        elif state == 1:
            if type(name) == bytes: robot_dict["location%s"%loc]["name"] = name.decode()
            else: robot_dict["location%s"%loc]["name"] = name
            robot_dict["location%s"%loc]["request"] = False
            if robot_dict["location%s"%loc]["box"] == False:
                checkbox_robot_sto = lv.checkbox(list_robotsto)
                checkbox_robot_sto.set_text(str(loc) + ", " + robot_dict["location%s"%loc]["name"])
                checkbox_robot_sto.add_event_cb(update_request_list, lv.EVENT.CLICKED, None)
            robot_dict["location%s"%loc]["box"] = True
            cont_overview_conv.get_child(robot_floor.index(str(loc))+1+len(conveyors_nr)).add_state(lv.STATE.CHECKED)
            list_remove_wms.get_child(loc-110+total_wh_positions+len(conveyors_nr)).clear_state(lv.STATE.CHECKED)
            list_remove_wms.get_child(loc-110+total_wh_positions+len(conveyors_nr)).set_text(str(robot_dict["location%s"%loc]["position"]) + ", " + robot_dict["location%s"%loc]["name"])
            list_remove_wms.get_child(loc-110+total_wh_positions+len(conveyors_nr)).clear_flag(lv.obj.FLAG.HIDDEN)
            

#When a pallet has been moved from one location to another, the master EV3 will just send start and end location, no names (faster communication without a string)
def transport_pallet(loc_start, loc_end):
    print("Transport requested from %s to %s."%(loc_start, loc_end))
    name = ""
    if 0 <= loc_start < 100:
        if pallets_dict["location%s"%loc_start]["box"] == True:
            name = pallets_dict["location%s"%loc_start]["name"]
    elif 100 <= loc_start < 110:
        if conveyor_dict["location%s"%loc_start]["box"] == True:
            name = conveyor_dict["location%s"%loc_start]["name"]
    elif 110 <= loc_start < 120:
        if robot_dict["location%s"%loc_start]["box"] == True:
            name = robot_dict["location%s"%loc_start]["name"]
    if name == "": return("Invalid move, no box present")
    try:
        update_storage(loc_end, 1, name)
        update_storage(loc_start, 0, "")
    except: return
        #machine_errors.append("Invalid move requested")


#Changing the WMS data if a request is received by UART, this adds or removes a request state for a box. To either pick it up, or drop it off at a location
def update_request(loc, state):
    print("Request to lock the location %s"%loc, "in the state %s"%state)
    if 0 <= loc < 100:
        if pallets_dict["location%s"%loc]["box"] == True:
            for i in range(total_wh_positions):
                child_pos = list_request.get_child(i).get_text().split(", ")
                if int(child_pos[0]) == loc:
                    if state == 0:
                        pallets_dict["location%s"%loc]["request"] = False
                        list_request.get_child(i).clear_state(lv.STATE.CHECKED | lv.STATE.DISABLED)
                    elif state == 1:
                        pallets_dict["location%s"%loc]["request"] = True
                        list_request.get_child(i).clear_state(lv.STATE.DISABLED)
                        list_request.get_child(i).add_state(lv.STATE.CHECKED)
                    elif state == 2:
                        pallets_dict["location%s"%loc]["request"] = False
                        list_request.get_child(i).clear_state(lv.STATE.CHECKED)
                        list_request.get_child(i).add_state(lv.STATE.DISABLED)
                    elif state == 3:
                        pallets_dict["location%s"%loc]["request"] = True
                        list_request.get_child(i).add_state(lv.STATE.CHECKED | lv.STATE.DISABLED)
                    break
        else:
            if state == 0:
                pallets_dict["location%s"%loc]["request"] = False
                cont_storeto.get_child(59-loc).clear_state(lv.STATE.CHECKED | lv.STATE.DISABLED)
            elif state == 1:
                pallets_dict["location%s"%loc]["request"] = True
                cont_storeto.get_child(59-loc).add_state(lv.STATE.CHECKED)
                cont_storeto.get_child(59-loc).clear_state(lv.STATE.DISABLED)
            elif state == 2:
                pallets_dict["location%s"%loc]["request"] = False
                cont_storeto.get_child(59-loc).add_state(lv.STATE.DISABLED)
                cont_storeto.get_child(59-loc).add_state(lv.STATE.CHECKED)
            elif state == 3:
                pallets_dict["location%s"%loc]["request"] = True
                cont_storeto.get_child(59-loc).add_state(lv.STATE.CHECKED | lv.STATE.DISABLED)
    elif 110 <= loc < 120:
        if robot_dict["location%s"%loc]["box"] == True:
            for i in range(len(robot_floor)):
                child_pos = list_robotsto.get_child(i).get_text().split(", ")
                if int(child_pos[0]) == loc:
                    if state == 0:
                        robot_dict["location%s"%loc]["request"] = False
                        list_robotsto.get_child(i).clear_state(lv.STATE.CHECKED | lv.STATE.DISABLED)
                    elif state == 1:
                        robot_dict["location%s"%loc]["request"] = True
                        list_robotsto.get_child(i).add_state(lv.STATE.CHECKED)
                        list_robotsto.get_child(i).clear_state(lv.STATE.DISABLED)
                    elif state == 2:
                        robot_dict["location%s"%loc]["request"] = False
                        list_robotsto.get_child(i).add_state(lv.STATE.DISABLED)
                        list_robotsto.get_child(i).clear_state(lv.STATE.CHECKED)
                    elif state == 3:
                        robot_dict["location%s"%loc]["request"] = True
                        list_robotsto.get_child(i).add_state(lv.STATE.CHECKED | lv.STATE.DISABLED)
                    break


#Adding/removing a machine error by UART
def update_mode(state, problem):
    global machine_errors
    global list_errors
    global selection
    global uart_started
    
    issue = problem.decode()
    print(issue, ": New state =", state)
    if state == 0:   #Removing the error
        if issue in machine_errors:
            machine_errors.remove(issue)
            if issue == "Homing robot not finished":
                dropdown_manual.close()
                dropdown_manual.set_selected(dropdown_manual.get_option_cnt() - 1)
            elif issue == "Homing scissorlift not finished":
                dropdown_manual.add_option("\n".join(["Adjustment scissorlift"]), 1)
                dropdown_manual.close()
            elif issue == "Homing stacker crane not finished":
                dropdown_manual.add_option("\n".join(["Adjustment stacker crane"]), 1)
                dropdown_manual.close()
            elif issue == "Homing chain conveyors not finished":
                dropdown_manual.add_option("\n".join(["Adjustment conveyors"]), 1)
                dropdown_manual.close()
            elif issue == "Communication not online":
                machine_errors.extend(["WMS Data not received", "Homing scissorlift not finished"])
                list_errors.add_btn(lv.SYMBOL.USB, "WMS Data not received")
                list_errors.get_child(list_errors.get_child_cnt()-1).add_event_cb(event_errors, lv.EVENT.CLICKED, None)
                list_errors.add_btn(lv.SYMBOL.LOOP, "Homing scissorlift not finished")
                list_errors.get_child(list_errors.get_child_cnt()-1).add_event_cb(event_errors, lv.EVENT.CLICKED, None)
                if "Full warehouse with robot" in selection or "Full warehouse without robot" in selection:
                    machine_errors.extend(["Homing stacker crane not finished", "Homing chain conveyors not finished"])
                    list_errors.add_btn(lv.SYMBOL.LOOP, "Homing stacker crane not finished")
                    list_errors.get_child(list_errors.get_child_cnt()-1).add_event_cb(event_errors, lv.EVENT.CLICKED, None)
                    list_errors.add_btn(lv.SYMBOL.LOOP, "Homing chain conveyors not finished")
                    list_errors.get_child(list_errors.get_child_cnt()-1).add_event_cb(event_errors, lv.EVENT.CLICKED, None)
                if "Full warehouse with robot" in selection:
                    machine_errors.extend(["Homing robot not finished"])
                    list_errors.add_btn(lv.SYMBOL.LOOP, "Homing robot not finished")
                    list_errors.get_child(list_errors.get_child_cnt()-1).add_event_cb(event_errors, lv.EVENT.CLICKED, None)
                cont_storeto.clear_flag(lv.obj.FLAG.HIDDEN)
                list_request.clear_flag(lv.obj.FLAG.HIDDEN)
                list_robotsto.clear_flag(lv.obj.FLAG.HIDDEN)
        elif issue == "Show connection screen":
            if uart_started == False:
                uart_started = True
                cont_mode_selector.clear_flag(lv.obj.FLAG.HIDDEN)
    elif state == 1: #Adding the error
        if issue not in machine_errors:
            machine_errors.append(issue)
            try:
                for i in range(list_errors.get_child_cnt()):
                    if issue == list_errors.get_child(i).get_child(1):
                        return
            except: return
            if issue == "Emergency Stop pushed":
                list_errors.add_btn(lv.SYMBOL.WARNING, issue)
                list_errors.get_child(list_errors.get_child_cnt()-1).add_event_cb(event_errors, lv.EVENT.CLICKED, None)
                return
            elif issue == "Communication not online":
                list_errors.add_btn(lv.SYMBOL.WIFI, issue)
            elif issue == "WMS DATA not received":
                list_errors.add_btn(lv.SYMBOL.USB, issue)
            elif issue == "Homing stacker crane fork error":
                list_errors.add_btn(lv.SYMBOL.WARNING, issue)
            else:
                list_errors.add_btn(lv.SYMBOL.WARNING, issue)
            list_errors.get_child(list_errors.get_child_cnt()-1).add_event_cb(event_errors, lv.EVENT.CLICKED, None)
            

#Adding the extra commands to the uartremote
ur.add_command(update_storage)
ur.add_command(update_request)
ur.add_command(transport_pallet)
ur.add_command(update_mode)


##############################THESE DEFINITIONS ARE TO DO SOMETHING AFTER AN INTERACTION WITH AN OBJECT ON THE SCREEN (CLICK/DRAG/...)##############################
### Screen action definitions ###
def update_request_list(e): #If a 'Request' checkbox has been clicked, check the new state, save the request state and send the new state by UART to the master EV3
    target = e.get_target()
    target_info = target.get_text().split(", ")
    print("Retrieve from %s"%target_info)
    if 0 <= int(target_info[0]) < 100:
        if target.get_state() & lv.STATE.CHECKED:
            pallets_dict["location%s"%target_info[0]]["request"] = True
            ur.send_command("update_request", '2b', int(target_info[0]), 1)
        else:
            pallets_dict["location%s"%target_info[0]]["request"] = False
            ur.send_command("update_request", '2b', int(target_info[0]), 0)
        #state = target.get_state()
    elif 110 <= int(target_info[0]) < 120:
        if target.get_state() & lv.STATE.CHECKED:
            robot_dict["location%s"%target_info[0]]["request"] = True
            ur.send_command("update_request", '2b', int(target_info[0]), 1)
        else:
            robot_dict["location%s"%target_info[0]]["request"] = False
            ur.send_command("update_request", '2b', int(target_info[0]), 0)


def update_storeto_list(e): #If a 'Store' checkbox has been clicked, check the new state, save the request state and send the new state by UART to the master EV3
    target = e.get_target()
    target_info = int(target.get_text())
    print("Store to %s"%target_info)
    if target.get_state() & lv.STATE.CHECKED:
        pallets_dict["location%s"%target_info]["request"] = True
        ur.send_command("update_request", '2b', int(target_info), 1)
    else:
        pallets_dict["location%s"%target_info]["request"] = False
        ur.send_command("update_request", '2b', int(target_info), 0)

def menu_select(e): #If an item in the dropdown menu has been selected, perform the corresponding tasks by reading the label of the selected item
    dropdown = e.get_target()
    option = " "*64
    dropdown.get_selected_str(option, len(option))
    if "Overview conveyors" in option:
        ta.add_flag(lv.obj.FLAG.HIDDEN)
        cont_add_wms.add_flag(lv.obj.FLAG.HIDDEN)
        btn_remove_wms.add_flag(lv.obj.FLAG.HIDDEN)
        list_remove_wms.add_flag(lv.obj.FLAG.HIDDEN)
        cont_overview_conv.clear_flag(lv.obj.FLAG.HIDDEN)
        dropdown.get_parent().get_child(1).set_text(option)
    elif "Add manual to WMS" in option:
        cont_overview_conv.add_flag(lv.obj.FLAG.HIDDEN)
        btn_remove_wms.add_flag(lv.obj.FLAG.HIDDEN)
        list_remove_wms.add_flag(lv.obj.FLAG.HIDDEN)
        ta.clear_flag(lv.obj.FLAG.HIDDEN)
        cont_add_wms.clear_flag(lv.obj.FLAG.HIDDEN)
        dropdown.get_parent().get_child(1).set_text(option)
    elif "Remove manual from WMS" in option:
        cont_overview_conv.add_flag(lv.obj.FLAG.HIDDEN)
        ta.add_flag(lv.obj.FLAG.HIDDEN)
        cont_add_wms.add_flag(lv.obj.FLAG.HIDDEN)
        btn_remove_wms.clear_flag(lv.obj.FLAG.HIDDEN)
        list_remove_wms.clear_flag(lv.obj.FLAG.HIDDEN)
        dropdown.get_parent().get_child(1).set_text("Removing from WMS")
    else:
        cont_overview_conv.add_flag(lv.obj.FLAG.HIDDEN)
        ta.add_flag(lv.obj.FLAG.HIDDEN)
        cont_add_wms.add_flag(lv.obj.FLAG.HIDDEN)
        btn_remove_wms.add_flag(lv.obj.FLAG.HIDDEN)
        list_remove_wms.add_flag(lv.obj.FLAG.HIDDEN)
        dropdown.get_parent().get_child(1).set_text("Nothing selected")


def refresh_errors(e): #If the refresh error button has been clicked, check what errors are still actual, remove the already resolved errors from the history list
    target = e.get_target()
    update_list = target.get_parent().get_child(1) #List object
    counter = 0         #Adjust the count when an item is deleted
    found_errors = []   #Already shown errors
    for i in range(20):
        try:
            try:
                if update_list.get_child(i-counter).get_child(0).get_text() in machine_errors:
                    found_errors.append(update_list.get_child(i-counter).get_child(0).get_text())
                else:
                    update_list.get_child(i-counter).delete()
                    counter += 1
            except:
                if update_list.get_child(i-counter).get_child(1).get_text() in machine_errors:
                    found_errors.append(update_list.get_child(i-counter).get_child(1).get_text())
                else:
                    update_list.get_child(i-counter).delete()
                    counter += 1
        except:
            continue
    counter = len(found_errors)
    for i in machine_errors:   #Adding errors not shown yet
        if i not in found_errors:
            update_list.add_btn(None, i)
            update_list.get_child(counter).add_event_cb(event_errors, lv.EVENT.CLICKED, None)
            counter += 1
            
            
def event_request(e): #TODO No idea what this one does, will look later again, hard to find anything with THONNY IDE
    global last_event_errors
    global last_event_errors_time
    
    target = e.get_target()
    if type(target) != type(lv.obj()):
        last_event_errors = None
        last_event_errors_time = 0
        #"Touched scrollable area"
        return
    label = target.get_child(0).get_text()
    print(label)


def event_errors(e): #If in the error history list an item has been clicked, check if it has been clicked before in the last 500ms (double clicked), if so show more information in a messagebox
    global last_event_errors
    global last_event_errors_time
    
    target = e.get_target()
    if type(target) != type(lv.obj()):
        last_event_errors = None
        last_event_errors_time = 0
        #"Touched scrollable area")
        return
    try: label = target.get_child(0).get_text()
    except: label = target.get_child(1).get_text()
    if label != last_event_errors or time.ticks_ms() - 500 > last_event_errors_time:
        last_event_errors = label
        last_event_errors_time = time.ticks_ms()
    else:
        print("Double clicked %s"%label)
        last_event_errors = None
        last_event_errors_time = 0
        if label in error_info:
            mbox1 = lv.msgbox(target.get_parent().get_parent(), label, error_info[label]["info"], error_info[label]["actions"], True)
            mbox1.add_event_cb(btns_mbox_error, lv.EVENT.VALUE_CHANGED, None) #####Added for extra buttons
            mbox1.align(lv.ALIGN.TOP_LEFT,5,50)
        else:
            mbox1 = lv.msgbox(target.get_parent().get_parent(), label, "No information found for this error", None, True)
            mbox1.align(lv.ALIGN.TOP_LEFT,5,50)
        

def btns_mbox_error(e): #If an opened messagebox from an error has buttons, check what action needs to be done when a button has been clicked. This is to resolve sensor/motor errors on the EV3's
    global list_errors
    
    mbox = e.get_current_target()
    label = mbox.get_active_btn_text()
    error = mbox.get_child(0).get_text()
    if label == "Try again":
        if error == "Input chain not empty" or error == "Input chain not full":
            ur.send_command("reset_error", '1b', 0)
        elif error == "Output chain not empty" or error == "Output chain not full":
            ur.send_command("reset_error", '1b', 1)
        elif error == "Homing stacker crane fork error" or error == "Homing stacker crane lift or drive error":
            ur.send_command("reset_error", '1b', 2)
        elif error == "Positioning stacker crane error":
            ur.send_command("reset_error", '1b', 3)
        
        machine_errors.remove(error)
        counter = 0
        for i in range(20):
            try:
                if list_errors.get_child(i-counter).get_child(1).get_text() == error:
                    list_errors.get_child(i-counter).delete()
                    counter += 1
            except:
                continue
        mbox.delete()
    elif label == "Close":
        mbox.delete()
        
        
def event_modes(e): #This function is to select the Off/Automatic/Manual modes to operate each machine part by double clicking the buttons on the modes page
    global hb_crane_input
    global hb_crane_output
    global conveyors
    global output
    global robot_used
    global mode_request
    global last_event_modes
    global last_event_modes_time
    global selection
    global machine_errors
    
    target = e.get_target()
    if type(target) != type(lv.btn()): #Check if the click was on a button or the free space around it
        last_event_modes = None
        last_event_modes_time = 0
        #print("Touched scrollable area")
        return
    label = target.get_child(0).get_text()
    if label != last_event_modes or time.ticks_ms() - 500 > last_event_modes_time or "Communication not online" in machine_errors:
        last_event_modes = label
        last_event_modes_time = time.ticks_ms()
        #print(label)
    else:
        print("Double clicked %s"%label)
        last_event_modes = None
        last_event_modes_time = 0
        
        if "Homing stacker crane not finished" in machine_errors or "Homing robot not finished" in machine_errors or "Homing chain conveyors not finished" in machine_errors or "Homing scissorlift not finished" in machine_errors or "Communication not online" in machine_errors:
            print("Before mbox")
            mbox1 = lv.msgbox(target.get_parent().get_parent(), label, "The homing is not finished, please wait with selecting modes", None, True)
            mbox1.align(lv.ALIGN.TOP_LEFT,5,50)
            print("after mbox")
        elif label == cont_btns[0]:
            if hb_crane_input == "Off":
                mode_request = time.ticks_ms()
                ur.send_command("mode_warehouse", '2b', 0,1)                    
                target.set_style_bg_color(lv.palette_main(lv.PALETTE.GREEN), 0)
                hb_crane_input = "Automatic"
                target.get_parent().get_child(1).add_flag(lv.obj.FLAG.HIDDEN) 
            elif hb_crane_input == "Automatic":
                mode_request = time.ticks_ms()
                ur.send_command("mode_warehouse", '2b', 0,0) 
                target.set_style_bg_color(lv.palette_main(lv.PALETTE.RED), 0)
                hb_crane_input = "Off"
                target.get_parent().get_child(1).clear_flag(lv.obj.FLAG.HIDDEN)
         
        elif label == cont_btns[1]:
            if hb_crane_input == "Off":
                mode_request = time.ticks_ms()
                ur.send_command("mode_warehouse", '2b', 0,2)
                target.set_style_bg_color(lv.palette_main(lv.PALETTE.GREEN), 0)
                hb_crane_input = "Manual"
                target.get_parent().get_child(0).add_flag(lv.obj.FLAG.HIDDEN)
            elif hb_crane_input == "Manual":
                mode_request = time.ticks_ms()
                ur.send_command("mode_warehouse", '2b', 1,0) 
                target.set_style_bg_color(lv.palette_main(lv.PALETTE.RED), 0)
                hb_crane_input = "Off"
                target.get_parent().get_child(0).clear_flag(lv.obj.FLAG.HIDDEN)

        elif label == cont_btns[2]:
            if hb_crane_output == "Off":
                mode_request = time.ticks_ms()
                ur.send_command("mode_warehouse", '2b', 1,1) 
                target.set_style_bg_color(lv.palette_main(lv.PALETTE.GREEN), 0)
                hb_crane_output = "Automatic"
                target.get_parent().get_child(3).add_flag(lv.obj.FLAG.HIDDEN)
            elif hb_crane_output == "Automatic":
                mode_request = time.ticks_ms()
                ur.send_command("mode_warehouse", '2b', 1,0) 
                target.set_style_bg_color(lv.palette_main(lv.PALETTE.RED), 0)
                hb_crane_output = "Off"
                target.get_parent().get_child(3).clear_flag(lv.obj.FLAG.HIDDEN)
                                
        elif label == cont_btns[3]:
            if hb_crane_output == "Off":
                mode_request = time.ticks_ms()
                ur.send_command("mode_warehouse", '2b', 1,2) 
                target.set_style_bg_color(lv.palette_main(lv.PALETTE.GREEN), 0)
                hb_crane_output = "Manual"
                target.get_parent().get_child(2).add_flag(lv.obj.FLAG.HIDDEN)
            elif hb_crane_output == "Manual":
                mode_request = time.ticks_ms()
                ur.send_command("mode_warehouse", '2b', 1,0) 
                target.set_style_bg_color(lv.palette_main(lv.PALETTE.RED), 0)
                hb_crane_output = "Off"
                target.get_parent().get_child(2).clear_flag(lv.obj.FLAG.HIDDEN)
                
        elif label == cont_btns[4] and "Full warehouse with robot" in selection:
            if robot_used == False:
                mode_request = time.ticks_ms()
                ur.send_command("mode_warehouse", '2b', 3,1) 
                target.set_style_bg_color(lv.palette_main(lv.PALETTE.GREEN), 0)
                robot_used = True
            elif robot_used == True:
                mode_request = time.ticks_ms()
                ur.send_command("mode_warehouse", '2b', 3,0) 
                target.set_style_bg_color(lv.palette_main(lv.PALETTE.RED), 0)
                robot_used = False
                
        elif label == cont_btns[5]:
            if conveyors == "Off":
                mode_request = time.ticks_ms()
                ur.send_command("mode_warehouse", '2b', 2,1)
                target.set_style_bg_color(lv.palette_main(lv.PALETTE.GREEN), 0)
                conveyors = "Automatic"
            elif conveyors == "Automatic":
                mode_request = time.ticks_ms()
                ur.send_command("mode_warehouse", '2b', 2,0)
                target.set_style_bg_color(lv.palette_main(lv.PALETTE.RED), 0)
                conveyors = "Off"


def set_mode_clicked(e): #This function is a scrollbar to select in what mode the EV3 master needs to start its program (to let it know the amount of bluetooth connections awaiting)
    global selection
    
    target = e.get_target()
    label = target.get_child(0).get_text()
    option = " "*30
    roller_mode.get_selected_str(option, len(option))
    selection = option.strip()
    if   label == "Set warehouse mode":
        print(selection)
        if   "Debug testing master" in selection:
            ur.send_command("mode_warehouse", '2b', 4, 0)
        elif "Full warehouse with robot" in selection:
            ur.send_command("mode_warehouse", '2b', 4, 1) 
        elif "Full warehouse without robot" in selection:
            ur.send_command("mode_warehouse", '2b', 4, 2)
        cont_mode_selector.add_flag(lv.obj.FLAG.HIDDEN)


def homing_robot_clicked(e): #Function to start the homing of the large 6axis-robot, or perform some basic movements with the first 3 axis, to set it in a safe position before homing starts
    global robot_homing_start
    
    target = e.get_target()
    label = target.get_child(0).get_text()
    if   label == "Start homing robot":
        robot_homing_start = "Full"
        ur.send_command("adjust_manual", '2b', 0, 1)
        cont_manual_homing.add_flag(lv.obj.FLAG.HIDDEN)
        manual_label.set_text("Manual controls")
        dropdown_manual.set_selected(dropdown_manual.get_option_cnt() - 1)
        
    elif label == "J1: CW":       ur.send_command("adjust_manual", '2b', 0, 2)
    elif label == "J1: CCW":      ur.send_command("adjust_manual", '2b', 0, 3) 
    elif label == "J2: CW":       ur.send_command("adjust_manual", '2b', 0, 4)
    elif label == "J2: CCW":      ur.send_command("adjust_manual", '2b', 0, 5) 
    elif label == "J3: CW":       ur.send_command("adjust_manual", '2b', 0, 6)
    elif label == "J3: CCW":      ur.send_command("adjust_manual", '2b', 0, 7) 


def homing_robot_released(e): #Function to stop the motors from running when a button has been pressed for manual control before homing the robot
    ur.send_command("adjust_manual", '2b', 0, 0)
    

def adjust_robot_angle(e): #Function that changes the text DURING sliding the slidebars to adjust the motor angles
    target = e.get_target()
    label = target.get_child(0).get_text()
    target.get_child(0).set_text("Joint %s adjust: %s"%(label[6], target.get_value()))


def send_robot_angle(e): #Function that sends the adjusted motor angle AFTER releasing the slider
    target = e.get_target()
    label = target.get_child(0).get_text()
    print(label, target.get_value())
    ur.send_command("adjust_manual", '2b', int(label[6]), int(target.get_value()))


def adjust_scissor_angle(e): #Function that changes the text DURING sliding the slidebars to adjust the motor angle/speed
    target = e.get_target()
    label = target.get_child(0).get_text()
    if "height" in label:
        target.get_child(0).set_text("Scissor max height\n     adjustment: %s"%target.get_value())
    else:
        target.get_child(0).set_text("Speed max: %s"%target.get_value())
        

def send_scissor_angle(e): #Function that sends the adjusted motor angle/speed AFTER releasing the slider
    target = e.get_target()
    label = target.get_child(0).get_text()
    print(label, target.get_value())
    if "height" in label:
        ur.send_command("adjust_manual", '2b', 7, int(target.get_value()))
    else:
        ur.send_command("adjust_manual", '2b', 8, int(target.get_value()))
    
    
def adjust_crane_angle(e): #Function that changes the text DURING sliding the slidebars to adjust the motor angle/speed
    target = e.get_target()
    label = target.get_child(0).get_text()
    if "height" in label:
        target.get_child(0).set_text("Crane height\nadjustment: %s"%target.get_value())
    else:
        target.get_child(0).set_text("Speed max: %s"%slider_crane_speed.get_value())
    

def send_crane_angle(e): #Function that sends the adjusted motor angle/speed AFTER releasing the slider
    target = e.get_target()
    label = target.get_child(0).get_text()
    print(label, target.get_value())
    if "height" in label:
        ur.send_command("adjust_manual", '2b', 9, int(target.get_value()))
    else:
        ur.send_command("adjust_manual", '2b', 10, int(target.get_value()))


def adjust_conv(e): #Function that changes the text DURING sliding the slidebars to adjust the motor speed
    target = e.get_target()
    label = target.get_child(0).get_text()
    if "Chain" in label:
        target.get_child(0).set_text("Chain speed\nadjustment: %s"%slider_conv_chain.get_value())
    else:
        target.get_child(0).set_text("Rolls speed\nadjustment: %s"%slider_conv_roll.get_value())
    

def send_adjust_conv(e): #Function that sends the adjusted motor speed AFTER releasing the slider
    target = e.get_target()
    label = target.get_child(0).get_text()
    print(label, target.get_value())
    if "Chain" in label:
        ur.send_command("adjust_manual", '2b', 11, int(target.get_value()))
    else:
        ur.send_command("adjust_manual", '2b', 12, int(target.get_value()))    


def manual_menu(e): #If an item in the manual adjustment dropdown menu has been selected, perform the corresponding tasks by reading the label of the selected item
    global robot_homing_start
    global selection

    dropdown = e.get_target()
    option = " "*64
    dropdown.get_selected_str(option, len(option))
    if "Close all" in option:
        cont_manual_homing.add_flag(lv.obj.FLAG.HIDDEN)
        cont_adjust_robot.add_flag(lv.obj.FLAG.HIDDEN)
        cont_adjust_scissor.add_flag(lv.obj.FLAG.HIDDEN)
        cont_adjust_crane.add_flag(lv.obj.FLAG.HIDDEN)
        cont_adjust_conv.add_flag(lv.obj.FLAG.HIDDEN)
        manual_label.set_text("Manual control page")
    if "Robot control" in option and robot_homing_start != "Full" and "Full warehouse with robot" in selection:
        cont_adjust_scissor.add_flag(lv.obj.FLAG.HIDDEN)
        cont_adjust_crane.add_flag(lv.obj.FLAG.HIDDEN)
        cont_adjust_conv.add_flag(lv.obj.FLAG.HIDDEN)
        cont_manual_homing.clear_flag(lv.obj.FLAG.HIDDEN)
        manual_label.set_text("Safely homing robot")
    elif "Robot control" in option and "Homing robot not finished" not in machine_errors and "Full warehouse with robot" in selection:
        cont_adjust_scissor.add_flag(lv.obj.FLAG.HIDDEN)
        cont_adjust_crane.add_flag(lv.obj.FLAG.HIDDEN)
        cont_adjust_conv.add_flag(lv.obj.FLAG.HIDDEN)
        cont_adjust_robot.clear_flag(lv.obj.FLAG.HIDDEN)
        manual_label.set_text("Fine adjustment robot")
    elif "Robot control" in option and "Homing robot not finished" in machine_errors and "Full warehouse with robot" in selection:
        manual_label.set_text("No controls, homing")
    elif "Robot control" in option and not "Full warehouse with robot" in selection:
        manual_label.set_text("Robot not used mode")
    elif "Adjustment scissorlift" in option and "Homing scissorlift not finished" not in machine_errors:
        cont_manual_homing.add_flag(lv.obj.FLAG.HIDDEN)
        cont_adjust_robot.add_flag(lv.obj.FLAG.HIDDEN)
        cont_adjust_crane.add_flag(lv.obj.FLAG.HIDDEN)
        cont_adjust_conv.add_flag(lv.obj.FLAG.HIDDEN)
        cont_adjust_scissor.clear_flag(lv.obj.FLAG.HIDDEN)
        manual_label.set_text(option)
    elif "Adjustment stacker crane" in option and "Homing stacker crane not finished" not in machine_errors:
        cont_manual_homing.add_flag(lv.obj.FLAG.HIDDEN)
        cont_adjust_robot.add_flag(lv.obj.FLAG.HIDDEN)
        cont_adjust_scissor.add_flag(lv.obj.FLAG.HIDDEN)
        cont_adjust_conv.add_flag(lv.obj.FLAG.HIDDEN)
        cont_adjust_crane.clear_flag(lv.obj.FLAG.HIDDEN)
        manual_label.set_text(option)
    elif "Adjustment conveyors" in option and "Homing chain conveyors not finished" not in machine_errors:
        cont_manual_homing.add_flag(lv.obj.FLAG.HIDDEN)
        cont_adjust_robot.add_flag(lv.obj.FLAG.HIDDEN)
        cont_adjust_scissor.add_flag(lv.obj.FLAG.HIDDEN)
        cont_adjust_crane.add_flag(lv.obj.FLAG.HIDDEN)
        cont_adjust_conv.clear_flag(lv.obj.FLAG.HIDDEN)
        manual_label.set_text(option)
    else:
        cont_manual_homing.add_flag(lv.obj.FLAG.HIDDEN)
        cont_adjust_robot.add_flag(lv.obj.FLAG.HIDDEN)
        cont_adjust_scissor.add_flag(lv.obj.FLAG.HIDDEN)
        cont_adjust_crane.add_flag(lv.obj.FLAG.HIDDEN)
        cont_adjust_conv.add_flag(lv.obj.FLAG.HIDDEN)        


def ta_event_cb(e,kb): #Function to write text in a textarea with the premade keyboard (it will automatically show/go away with this)
    code = e.get_code()
    ta = e.get_target()
    if code == lv.EVENT.FOCUSED:
        kb.set_textarea(ta)
        kb.clear_flag(lv.obj.FLAG.HIDDEN)

    if code == lv.EVENT.DEFOCUSED:
        kb.set_textarea(None)
        kb.add_flag(lv.obj.FLAG.HIDDEN)
        

def add_to_wms(e): #Function to add a box to the WMS, by looking at the slected button and text filled in the textarea
    target = e.get_target()
    zone = target.get_child(0).get_text()
    name = ta.get_text()
    if len(name) > 0: #The name should be longer than 0 characters
        option = " "*5
        if   zone == "Add to the rack WMS": roller_rack.get_selected_str(option, len(option))
        elif zone == "Add to the conv WMS": roller_conv.get_selected_str(option, len(option))
        elif zone == "Add to the floor WMS": roller_floor.get_selected_str(option, len(option))
        pos_str = option.strip() #Get the new location number (0-114) in string form with an extra character hiding somewhere
        pos_int = int(pos_str[:len(pos_str)-1]) #Save only the numbers as an integer
        print("This is the new name for the added box:", name)
        update_storage(pos_int, 1, ta.get_text())
        ur.send_command("mod_wms", '2b%ss'%len(name), pos_int, 1, name) #Send the new location as state 1 (box present) and the name of the box


def remove_from_wms(e): #Function to remove a box from the WMS, first give a warning here by opening a messagebox
    target = e.get_target()
    if "Communication not online" not in machine_errors:
        mbox_delete = lv.msgbox(target.get_parent(), "Deleting from WMS", "Are you sure you want to delete the selection from the Warehouse Management System? This is permanent and can not be undone!", ["Yes, delete!", "No!!"], True)
        mbox_delete.add_event_cb(btns_mbox_delete, lv.EVENT.VALUE_CHANGED, None)
        mbox_delete.align(lv.ALIGN.TOP_LEFT,5,50)
    else:
        mbox_delete = lv.msgbox(target.get_parent(), "Deleting from WMS", "This can not be done before the communication is working", None, True)
        mbox_delete.align(lv.ALIGN.TOP_LEFT,5,50)

def btns_mbox_delete(e): #Function for the buttons in the delete WMS messagebox
    mbox = e.get_current_target()
    label = mbox.get_active_btn_text()
    if label == "Yes, delete!":
        for i in range(128):
            try:
                if list_remove_wms.get_child(i).get_state() & lv.STATE.CHECKED:
                    if i < total_wh_positions: #Position is in the rack
                        pallets_dict["location%s"%i]["box"] = False
                        pallets_dict["location%s"%i]["name"] = "No box present"
                        pallets_dict["location%s"%i]["request"] = False
                        ur.send_command("mod_wms", '2bs', int(i), 0, "")
                        update_storage(i, 0, "")
                    elif i < total_wh_positions + len(conveyors_nr): #on a conveyor
                        conveyor_dict["location%s"%(i-total_wh_positions+100)]["box"] = False
                        conveyor_dict["location%s"%(i-total_wh_positions+100)]["name"] = "No box present"
                        ur.send_command("mod_wms", '2bs', int(i-total_wh_positions+100), 0, "")
                        update_storage(i-total_wh_positions+100, 0, "")
                    elif i < total_wh_positions + len(conveyors_nr) + len(robot_floor):
                        robot_dict["location%s"%(i-total_wh_positions-len(conveyors_nr)+110)]["box"] = False
                        robot_dict["location%s"%(i-total_wh_positions-len(conveyors_nr)+110)]["name"] = "No box present"
                        robot_dict["location%s"%(i-total_wh_positions-len(conveyors_nr)+110)]["request"] = False
                        ur.send_command("mod_wms", '2bs', int(i-total_wh_positions-len(conveyors_nr)+110), 0, "")
                        update_storage(i-total_wh_positions-len(conveyors_nr)+110, 0, "")
                    list_remove_wms.get_child(i).clear_state(lv.STATE.CHECKED)
                    list_remove_wms.get_child(i).add_flag(lv.obj.FLAG.HIDDEN)
            except:
                continue
    mbox.delete()


##############################HERE ARE ALL OBJECTS FOR THE TOUCHSCREEN DEFINED##############################
#Creating styles for buttons, not buttons themselve
style_btn = lv.style_t()
style_btn.init()
style_btn.set_radius(10)
style_btn.set_bg_opa(lv.OPA.COVER)
style_btn.set_bg_color(lv.palette_lighten(lv.PALETTE.GREY, 3))
style_btn.set_bg_grad_color(lv.palette_main(lv.PALETTE.GREY))
style_btn.set_bg_grad_dir(lv.GRAD_DIR.VER)
style_btn.set_border_color(lv.color_white())
style_btn.set_border_opa(lv.OPA._70)
style_btn.set_border_width(2)
style_btn.set_text_color(lv.color_white())

style_btn_blu = lv.style_t()
style_btn_blu.init()
style_btn_blu.set_bg_color(lv.palette_lighten(lv.PALETTE.BLUE, 2))
style_btn_blu.set_bg_grad_color(lv.palette_main(lv.PALETTE.BLUE))

style_btn_gry = lv.style_t()
style_btn_gry.init()
style_btn_gry.set_bg_color(lv.palette_lighten(lv.PALETTE.GREY, 2))
style_btn_gry.set_bg_grad_color(lv.palette_main(lv.PALETTE.GREY))

style_transp = lv.style_t()
style_transp.init()
style_transp.set_text_color(lv.color_black())
style_transp.set_bg_color(lv.palette_lighten(lv.PALETTE.GREY, 2))
style_transp.set_pad_all(6)
style_transp.set_bg_opa(lv.OPA._20)
 
style_semi_transp = lv.style_t()
style_semi_transp.init()
style_semi_transp.set_text_color(lv.color_black())
style_semi_transp.set_bg_opa(lv.OPA._40)

style_indic = lv.style_t()
style_indic.init()
style_indic.set_bg_opa(lv.OPA.COVER)
style_indic.set_bg_color(lv.palette_lighten(lv.PALETTE.BLUE, 2))
style_indic.set_radius(5)

style_red_txt = lv.style_t()
style_red_txt.init()
style_red_txt.set_text_color(lv.palette_main(lv.PALETTE.RED))


##############################CREATING THE MAIN PART OF THE GUI WITH THE NEXT 1 LINE! EVERY PAGE WILL BE A TILE HEREAFTER AND CAN BE SCROLLED TO##############################
page_tiles = lv.tileview(lv.scr_act())
##############################HOMING AND ADJUSTMENT PAGE##############################
#Creating the first tab (tile). Giving it the X and Y position in the tile array, at startup it will show the 0,0 tile. It can not go negative
tile_homing = page_tiles.add_tile(0, 0, lv.DIR.RIGHT)
#Adding a background image
img_robot = lv.img(tile_homing)
img_robot.set_src(robot_320_img)
img_robot.align(lv.ALIGN.CENTER, 0, 11)
img_robot.set_size(320, 217)
#Adding a label for extra information
manual_label = lv.label(tile_homing)
manual_label.align(lv.ALIGN.TOP_LEFT, 135, 25)
manual_label.set_width(lv.pct(55))
manual_label.set_text("")


###Manual aid for setting the 6 axis robot in a safe position and start the homing if OK
cont_manual_homing = lv.obj(tile_homing)
cont_manual_homing.set_size(180, 197)
cont_manual_homing.add_style(style_transp, 0)
cont_manual_homing.align(lv.ALIGN.BOTTOM_RIGHT, 0, 0)
cont_manual_homing.add_flag(lv.obj.FLAG.HIDDEN)
 
btn_homing_move_j1_cw = lv.btn(cont_manual_homing)
label = lv.label(btn_homing_move_j1_cw)
label.set_text("J1: CW")
btn_homing_move_j1_cw.set_size(lv.pct(47), lv.SIZE.CONTENT)
btn_homing_move_j1_cw.add_style(style_transp, 0)
btn_homing_move_j1_cw.align(lv.ALIGN.BOTTOM_RIGHT, 0, 0)
btn_homing_move_j1_cw.add_event_cb(homing_robot_clicked, lv.EVENT.LONG_PRESSED, None)
btn_homing_move_j1_cw.add_event_cb(homing_robot_released, lv.EVENT.RELEASED, None)

btn_homing_move_j1_ccw = lv.btn(cont_manual_homing)
label = lv.label(btn_homing_move_j1_ccw)
label.set_text("J1: CCW")
btn_homing_move_j1_ccw.set_size(lv.pct(51), lv.SIZE.CONTENT)
btn_homing_move_j1_ccw.add_style(style_transp, 0)
btn_homing_move_j1_ccw.align(lv.ALIGN.BOTTOM_LEFT, 0, 0)
btn_homing_move_j1_ccw.add_event_cb(homing_robot_clicked, lv.EVENT.LONG_PRESSED, None)
btn_homing_move_j1_ccw.add_event_cb(homing_robot_released, lv.EVENT.RELEASED, None)

btn_homing_move_j2_cw = lv.btn(cont_manual_homing)
label = lv.label(btn_homing_move_j2_cw)
label.set_text("J2: CW")
btn_homing_move_j2_cw.set_size(lv.pct(47), lv.SIZE.CONTENT)
btn_homing_move_j2_cw.add_style(style_transp, 0)
btn_homing_move_j2_cw.align(lv.ALIGN.BOTTOM_RIGHT, 0, -40)
btn_homing_move_j2_cw.add_event_cb(homing_robot_clicked, lv.EVENT.LONG_PRESSED, None)
btn_homing_move_j2_cw.add_event_cb(homing_robot_released, lv.EVENT.RELEASED, None)

btn_homing_move_j2_ccw = lv.btn(cont_manual_homing)
label = lv.label(btn_homing_move_j2_ccw)
label.set_text("J2: CCW")
btn_homing_move_j2_ccw.set_size(lv.pct(51), lv.SIZE.CONTENT)
btn_homing_move_j2_ccw.add_style(style_transp, 0)
btn_homing_move_j2_ccw.align(lv.ALIGN.BOTTOM_LEFT, 0, -40)
btn_homing_move_j2_ccw.add_event_cb(homing_robot_clicked, lv.EVENT.LONG_PRESSED, None)
btn_homing_move_j2_ccw.add_event_cb(homing_robot_released, lv.EVENT.RELEASED, None)

btn_homing_move_j3_cw = lv.btn(cont_manual_homing)
label = lv.label(btn_homing_move_j3_cw)
label.set_text("J3: CW")
btn_homing_move_j3_cw.set_size(lv.pct(47), lv.SIZE.CONTENT)
btn_homing_move_j3_cw.add_style(style_transp, 0)
btn_homing_move_j3_cw.align(lv.ALIGN.TOP_RIGHT, 0, 40)
btn_homing_move_j3_cw.add_event_cb(homing_robot_clicked, lv.EVENT.LONG_PRESSED, None)
btn_homing_move_j3_cw.add_event_cb(homing_robot_released, lv.EVENT.RELEASED, None)
 
btn_homing_move_j3_ccw = lv.btn(cont_manual_homing)
label = lv.label(btn_homing_move_j3_ccw)
label.set_text("J3: CCW")
btn_homing_move_j3_ccw.set_size(lv.pct(51), lv.SIZE.CONTENT)
btn_homing_move_j3_ccw.add_style(style_transp, 0)
btn_homing_move_j3_ccw.align(lv.ALIGN.TOP_LEFT, 0, 40)
btn_homing_move_j3_ccw.add_event_cb(homing_robot_clicked, lv.EVENT.LONG_PRESSED, None)
btn_homing_move_j3_ccw.add_event_cb(homing_robot_released, lv.EVENT.RELEASED, None)

btn_homing_robot_start = lv.btn(cont_manual_homing)
label = lv.label(btn_homing_robot_start)
label.center()
label.set_text("Start homing robot")
btn_homing_robot_start.set_size(lv.pct(100), lv.SIZE.CONTENT)
btn_homing_robot_start.add_style(style_semi_transp, 0)
btn_homing_robot_start.align(lv.ALIGN.TOP_LEFT, 0, 0)
btn_homing_robot_start.add_event_cb(homing_robot_clicked, lv.EVENT.SHORT_CLICKED, None)


###Manual adjustment container for each of the 6 axis from the robot arm
cont_adjust_robot = lv.obj(tile_homing)
cont_adjust_robot.set_size(180, 197)
cont_adjust_robot.add_style(style_transp, 0)
cont_adjust_robot.align(lv.ALIGN.BOTTOM_RIGHT, 0, 0)
cont_adjust_robot.add_flag(lv.obj.FLAG.HIDDEN)

for i in range(1, 7):
    slider = lv.slider(cont_adjust_robot)
    slider.remove_style_all()
    slider.set_size(lv.pct(100), 20)
    slider.set_mode(lv.slider.MODE.SYMMETRICAL)
    slider.align(lv.ALIGN.BOTTOM_MID, 0, 32 -i*32)
    slider.set_range(-10,10)
    slider.add_style(style_transp, 0)
    slider.add_style(style_indic, lv.PART.INDICATOR)
    slider.add_event_cb(adjust_robot_angle, lv.EVENT.VALUE_CHANGED, None)
    slider.add_event_cb(send_robot_angle, lv.EVENT.RELEASED, None)
    label = lv.label(slider)
    label.set_text("Joint %s adjust: %s"%(i, slider.get_value()))
    label.align(lv.ALIGN.CENTER, 0, 0)


###Creating a dropdown menu with currently available options, later more will be added
dropdown_manual = lv.dropdown(tile_homing)
dropdown_manual.add_style(style_semi_transp, 0)
dropdown_manual.align(lv.ALIGN.TOP_LEFT, 0, 25) #dropdown.align_to(cont_modes, lv.ALIGN.TOP_RIGHT,150,-15)
dropdown_manual.set_options("\n".join(["Robot control", "Close all"])) #, "Adjustment robot", "Adjustment scissor"
dropdown_manual.set_text("Menu")
dropdown_manual.set_selected_highlight(True)
dropdown_manual.add_event_cb(manual_menu, lv.EVENT.VALUE_CHANGED, None)
dropdown_manual.add_flag(lv.obj.FLAG.HIDDEN)


###Manual speed and height adjustment container for the scissor lift
cont_adjust_scissor = lv.obj(tile_homing)
cont_adjust_scissor.set_size(160, 120)
cont_adjust_scissor.add_style(style_transp, 0)
cont_adjust_scissor.align(lv.ALIGN.BOTTOM_LEFT, 0, -20)
cont_adjust_scissor.add_flag(lv.obj.FLAG.HIDDEN)

slider_scissor = lv.slider(cont_adjust_scissor)
slider_scissor.remove_style_all()
slider_scissor.set_size(lv.pct(100), 40)
slider_scissor.set_mode(lv.slider.MODE.SYMMETRICAL)
slider_scissor.align(lv.ALIGN.BOTTOM_MID, 0, 0)
slider_scissor.set_range(-10,10)
slider_scissor.add_style(style_transp, 0)
slider_scissor.add_style(style_indic, lv.PART.INDICATOR)
slider_scissor.add_event_cb(adjust_scissor_angle, lv.EVENT.VALUE_CHANGED, None)
slider_scissor.add_event_cb(send_scissor_angle, lv.EVENT.RELEASED, None)
label = lv.label(slider_scissor)
label.set_text("Scissor max height\n     adjustment: %s"%slider_scissor.get_value())
label.align(lv.ALIGN.CENTER, 0, 0)

slider_scissor_speed = lv.slider(cont_adjust_scissor)
slider_scissor_speed.remove_style_all()
slider_scissor_speed.set_size(lv.pct(100), 40)
slider_scissor_speed.set_range(20,100)
slider_scissor_speed.set_value(100, lv.ANIM.OFF)
slider_scissor_speed.align(lv.ALIGN.TOP_LEFT, 0, 0)
slider_scissor_speed.add_style(style_transp, 0)
slider_scissor_speed.add_style(style_indic, lv.PART.INDICATOR)
slider_scissor_speed.add_event_cb(adjust_scissor_angle, lv.EVENT.VALUE_CHANGED, None)
slider_scissor_speed.add_event_cb(send_scissor_angle, lv.EVENT.RELEASED, None)
label = lv.label(slider_scissor_speed)
label.set_text("Speed max: %s"%slider_scissor_speed.get_value())
label.align(lv.ALIGN.CENTER, 0, 0)


###Manual speed and height adjustment container for the stacker crane
cont_adjust_crane = lv.obj(tile_homing)
cont_adjust_crane.set_size(160, 120)
cont_adjust_crane.add_style(style_transp, 0)
cont_adjust_crane.align(lv.ALIGN.BOTTOM_LEFT, 0, -20)
cont_adjust_crane.add_flag(lv.obj.FLAG.HIDDEN)

slider_crane = lv.slider(cont_adjust_crane)
slider_crane.remove_style_all()
slider_crane.set_size(lv.pct(100), 40)
slider_crane.set_mode(lv.slider.MODE.SYMMETRICAL)
slider_crane.align(lv.ALIGN.BOTTOM_MID, 0, 0)
slider_crane.set_range(-10,10)
slider_crane.add_style(style_transp, 0)
slider_crane.add_style(style_indic, lv.PART.INDICATOR)
slider_crane.add_event_cb(adjust_crane_angle, lv.EVENT.VALUE_CHANGED, None)
slider_crane.add_event_cb(send_crane_angle, lv.EVENT.RELEASED, None)
label = lv.label(slider_crane)
label.set_text("Crane height\nadjustment: %s"%slider_crane.get_value())
label.align(lv.ALIGN.CENTER, 0, 0)

slider_crane_speed = lv.slider(cont_adjust_crane)
slider_crane_speed.remove_style_all()
slider_crane_speed.set_size(lv.pct(100), 40)
slider_crane_speed.set_range(20,100)
slider_crane_speed.set_value(100, lv.ANIM.OFF)
slider_crane_speed.align(lv.ALIGN.TOP_LEFT, 0, 0)
slider_crane_speed.add_style(style_transp, 0)
slider_crane_speed.add_style(style_indic, lv.PART.INDICATOR)
slider_crane_speed.add_event_cb(adjust_crane_angle, lv.EVENT.VALUE_CHANGED, None)
slider_crane_speed.add_event_cb(send_crane_angle, lv.EVENT.RELEASED, None)
label = lv.label(slider_crane_speed)
label.set_text("Speed max: %s"%slider_crane_speed.get_value())
label.align(lv.ALIGN.CENTER, 0, 0)


###Manual speed adjustment container for the chain & roll conveyors
cont_adjust_conv = lv.obj(tile_homing)
cont_adjust_conv.set_size(160, 120)
cont_adjust_conv.add_style(style_transp, 0)
cont_adjust_conv.align(lv.ALIGN.BOTTOM_LEFT, 0, -20)
cont_adjust_conv.add_flag(lv.obj.FLAG.HIDDEN)

slider_conv_chain = lv.slider(cont_adjust_conv)
slider_conv_chain.remove_style_all()
slider_conv_chain.set_size(lv.pct(100), 40)
slider_conv_chain.set_range(20,100)
slider_conv_chain.set_value(100, lv.ANIM.OFF)
slider_conv_chain.align(lv.ALIGN.BOTTOM_MID, 0, 0)
slider_conv_chain.add_style(style_transp, 0)
slider_conv_chain.add_style(style_indic, lv.PART.INDICATOR)
slider_conv_chain.add_event_cb(adjust_conv, lv.EVENT.VALUE_CHANGED, None)
slider_conv_chain.add_event_cb(send_adjust_conv, lv.EVENT.RELEASED, None)
label = lv.label(slider_conv_chain)
label.set_text("Chain speed\nadjustment: %s"%slider_conv_chain.get_value())
label.align(lv.ALIGN.CENTER, 0, 0)

slider_conv_roll = lv.slider(cont_adjust_conv)
slider_conv_roll.remove_style_all()
slider_conv_roll.set_size(lv.pct(100), 40)
slider_conv_roll.set_range(20,100)
slider_conv_roll.set_value(100, lv.ANIM.OFF)
slider_conv_roll.align(lv.ALIGN.TOP_LEFT, 0, 0)
slider_conv_roll.add_style(style_transp, 0)
slider_conv_roll.add_style(style_indic, lv.PART.INDICATOR)
slider_conv_roll.add_event_cb(adjust_conv, lv.EVENT.VALUE_CHANGED, None)
slider_conv_roll.add_event_cb(send_adjust_conv, lv.EVENT.RELEASED, None)
label = lv.label(slider_conv_roll)
label.set_text("Rolls speed\nadjustment: %s"%slider_conv_roll.get_value())
label.align(lv.ALIGN.CENTER, 0, 0)


###At the startup of the EV3 master brick, it needs to know the amount of connections, it can be selected here
cont_mode_selector = lv.obj(tile_homing)
cont_mode_selector.set_size(lv.pct(100), 197)
cont_mode_selector.add_style(style_transp, 0)
cont_mode_selector.align(lv.ALIGN.BOTTOM_RIGHT, 0, 0)
cont_mode_selector.add_flag(lv.obj.FLAG.HIDDEN)

btn_mode_selector = lv.btn(cont_mode_selector)
label = lv.label(btn_mode_selector)
label.center()
label.set_text("Set warehouse mode")
btn_mode_selector.set_size(lv.pct(100), lv.SIZE.CONTENT)
btn_mode_selector.add_style(style_semi_transp, 0)
btn_mode_selector.align(lv.ALIGN.TOP_LEFT, 0, 0)
btn_mode_selector.add_event_cb(set_mode_clicked, lv.EVENT.SHORT_CLICKED, None)

roller_mode = lv.roller(cont_mode_selector)
roller_mode.set_options("\n".join([
    "Full warehouse with robot",
    "Full warehouse without robot",
    "Debug testing master"]),lv.roller.MODE.INFINITE)
roller_mode.set_visible_row_count(3)
roller_mode.align_to(btn_mode_selector, lv.ALIGN.TOP_LEFT, 0, 40)

##############################ERROR PAGE (HISTORY)##############################

tile_errors = page_tiles.add_tile(1, 0, lv.DIR.LEFT | lv.DIR.BOTTOM | lv.DIR.RIGHT)
btn_refresh_errors = lv.btn(tile_errors)
label = lv.label(btn_refresh_errors)
label.set_text("Refresh errors")
btn_refresh_errors.set_size(lv.SIZE.CONTENT, lv.SIZE.CONTENT)
btn_refresh_errors.align(lv.ALIGN.TOP_LEFT, 0, 22)
btn_refresh_errors.add_event_cb(refresh_errors, lv.EVENT.CLICKED, None)

list_errors = lv.list(tile_errors)
list_errors.set_size(lv.pct(90), lv.pct(70))
list_errors.align(lv.ALIGN.TOP_LEFT, 0, 60)
for i in machine_errors:
    if i == "Emergency Stop pushed": list_errors.add_btn(lv.SYMBOL.WARNING, i)
    elif i == "Communication not online": list_errors.add_btn(lv.SYMBOL.WIFI, i)
    elif i == "Mode not selected": list_errors.add_btn(lv.SYMBOL.SETTINGS, i)
    else: list_errors.add_btn(lv.SYMBOL.WARNING, i)
    list_errors.get_child(list_errors.get_child_cnt()-1).add_event_cb(event_errors, lv.EVENT.CLICKED, None)


##############################MODE SELECTION PAGE##############################

tile_modes = page_tiles.add_tile(2, 0, lv.DIR.BOTTOM | lv.DIR.LEFT)
#Creating a scrolling row
cont_modes = lv.obj(tile_modes) #cont_modes = lv.obj(lv.scr_act())
cont_modes.set_size(220, 210) #cont_modes.set_size(220, 215)
cont_modes.align(lv.ALIGN.TOP_LEFT, 0, 25) #cont_modes.align(lv.ALIGN.TOP_LEFT, 0, 5)
cont_modes.set_scroll_snap_y(lv.SCROLL_SNAP.CENTER)
cont_modes.add_flag(lv.obj.FLAG.SCROLL_ONE)
cont_modes.set_flex_flow(lv.FLEX_FLOW.COLUMN)

for i in range(len(cont_btns)):
    btn_cont_modes = lv.btn(cont_modes)
    btn_cont_modes.set_size(lv.pct(95), lv.SIZE.CONTENT)
    btn_cont_modes.add_flag(lv.obj.FLAG.EVENT_BUBBLE) #Linking to parent
    #btn_cont_modes.add_style(style_btn_blu, 1)  #Is this needed?
    btn_cont_modes.add_style(style_btn_gry, lv.STATE.PRESSED)
    label = lv.label(btn_cont_modes)
    label.set_text(cont_btns[i])
    label.center()
    if i < 2 or i > 4:
        btn_cont_modes.clear_flag(lv.obj.FLAG.SNAPPABLE)
    btn_cont_modes.set_style_bg_color(lv.palette_main(lv.PALETTE.RED), 0)
cont_modes.add_event_cb(event_modes, lv.EVENT.CLICKED, None)


##############################CONVEYOR OVERVIEW##############################

tile_conveyors = page_tiles.add_tile(1, 1, lv.DIR.TOP | lv.DIR.RIGHT)
#Creating the dropdown menu
dropdown = lv.dropdown(tile_conveyors)
dropdown.align(lv.ALIGN.TOP_LEFT, 0, 25) #dropdown.align_to(cont_modes, lv.ALIGN.TOP_RIGHT,150,-15)
dropdown.set_options("\n".join(["Overview conveyors", "Add manual to WMS", "Remove manual from WMS", "Close all"]))
dropdown.set_text("Menu")
dropdown.set_selected_highlight(True)
dropdown.add_event_cb(menu_select, lv.EVENT.VALUE_CHANGED, None)

conveyors_label = lv.label(tile_conveyors)
conveyors_label.align(lv.ALIGN.TOP_LEFT, 135, 35)
conveyors_label.set_width(lv.pct(55))
conveyors_label.set_text("Overview conveyors")

cont_overview_conv = lv.obj(tile_conveyors)
img_top = lv.img(cont_overview_conv)
img_top.set_src(top_view_img)
img_top.align(lv.ALIGN.TOP_LEFT, -15, -25)
img_top.set_size(400, 231)

cont_overview_conv.set_size(280, 179) #Max 95% in X   (lv.pct(85), lv.pct(70))
cont_overview_conv.align(lv.ALIGN.TOP_LEFT, 0, 60)
pos_counter = 0
pos_val_conv = [[51,112],[60,74],[129,74],[129,112],[129,159],[60,159],[181,159],[225,130]]
for i in conveyors_nr:
    checkbox_overview_conv = lv.checkbox(cont_overview_conv)
    checkbox_overview_conv.align(lv.ALIGN.TOP_LEFT, pos_val_conv[pos_counter][0], pos_val_conv[pos_counter][1])
    checkbox_overview_conv.set_text(str(i))
    checkbox_overview_conv.add_style(style_red_txt, 0)
    if conveyor_dict["location%s"%i]["box"] == True:
        checkbox_overview_conv.add_state(lv.STATE.CHECKED | lv.STATE.DISABLED)
    else:
        checkbox_overview_conv.add_state(lv.STATE.DISABLED)
    pos_counter += 1
pos_counter = 0
pos_val_stor = [[300, 170],[352, 170],[404,170],[404,90],[181,20]]
for i in robot_floor:
    checkbox_overview_conv = lv.checkbox(cont_overview_conv)
    checkbox_overview_conv.align(lv.ALIGN.TOP_LEFT, pos_val_stor[pos_counter][0], pos_val_stor[pos_counter][1])
    checkbox_overview_conv.set_text(str(i))
    checkbox_overview_conv.add_style(style_red_txt, 0)
    if robot_dict["location%s"%i]["box"] == True:
        checkbox_overview_conv.add_state(lv.STATE.CHECKED | lv.STATE.DISABLED)
    else:
        checkbox_overview_conv.add_state(lv.STATE.DISABLED)
    pos_counter += 1


cont_add_wms = lv.obj(tile_conveyors)
cont_add_wms.set_size(282, 132) #Max 95% in X   (lv.pct(85), lv.pct(70))
cont_add_wms.align(lv.ALIGN.TOP_LEFT, 0, 97)
cont_add_wms.add_flag(lv.obj.FLAG.HIDDEN)

kb = lv.keyboard(lv.scr_act()) #Defining the keyboard
kb.add_flag(lv.obj.FLAG.HIDDEN)

ta = lv.textarea(tile_conveyors)
ta.set_size(282, 42)
ta.set_one_line(True)
ta.set_accepted_chars("0123456789 -abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ")
ta.set_max_length(29)
ta.align(lv.ALIGN.TOP_LEFT, 0, 60)
ta.add_event_cb(lambda e: ta_event_cb(e,kb), lv.EVENT.ALL, None)
ta.set_placeholder_text("Name for new box to WMS")
ta.add_flag(lv.obj.FLAG.HIDDEN)

btn_add_wms_rack = lv.btn(cont_add_wms)
label = lv.label(btn_add_wms_rack)
label.set_text("Add to the rack WMS")
btn_add_wms_rack.set_size(lv.SIZE.CONTENT, lv.SIZE.CONTENT)
btn_add_wms_rack.align(lv.ALIGN.TOP_LEFT, 33, 34)
btn_add_wms_rack.add_event_cb(add_to_wms, lv.EVENT.CLICKED, None)

btn_add_wms_conv = lv.btn(cont_add_wms)
label = lv.label(btn_add_wms_conv)
label.set_text("Add to the conv WMS")
btn_add_wms_conv.set_size(lv.SIZE.CONTENT, lv.SIZE.CONTENT)
btn_add_wms_conv.align(lv.ALIGN.TOP_LEFT, 33, 94)
btn_add_wms_conv.add_event_cb(add_to_wms, lv.EVENT.CLICKED, None)

btn_add_wms_floor = lv.btn(cont_add_wms)
label = lv.label(btn_add_wms_floor)
label.set_text("Add to the floor WMS")
btn_add_wms_floor.set_size(lv.SIZE.CONTENT, lv.SIZE.CONTENT)
btn_add_wms_floor.align(lv.ALIGN.TOP_LEFT, 33, 154)
btn_add_wms_floor.add_event_cb(add_to_wms, lv.EVENT.CLICKED, None)

roller_rack = lv.roller(cont_add_wms)
roller_rack.set_options("\n".join(rack_nr),lv.roller.MODE.NORMAL)
roller_rack.set_style_pad_all(3, 0)
roller_rack.set_visible_row_count(3)
roller_rack.align(lv.ALIGN.TOP_LEFT, 0, 0)
roller_rack.set_width(32)

roller_conv = lv.roller(cont_add_wms)
roller_conv.set_options("\n".join(conveyors_nr),lv.roller.MODE.NORMAL)
roller_conv.set_style_pad_all(3, 0)
roller_conv.set_visible_row_count(3)
roller_conv.align(lv.ALIGN.TOP_RIGHT, 0, 60)

roller_floor = lv.roller(cont_add_wms)
roller_floor.set_options("\n".join(robot_floor),lv.roller.MODE.NORMAL)
roller_floor.set_style_pad_all(3, 0)
roller_floor.set_visible_row_count(3)
roller_floor.align(lv.ALIGN.TOP_LEFT, 0, 120)
roller_floor.set_width(32)


btn_remove_wms = lv.btn(tile_conveyors)
label = lv.label(btn_remove_wms)
label.set_text("Remove selected from WMS")
btn_remove_wms.set_size(282, lv.SIZE.CONTENT)
btn_remove_wms.align(lv.ALIGN.TOP_LEFT, 0, 60)
btn_remove_wms.add_event_cb(remove_from_wms, lv.EVENT.CLICKED, None)
btn_remove_wms.add_flag(lv.obj.FLAG.HIDDEN)  

list_remove_wms = lv.list(tile_conveyors)
list_remove_wms.set_size(282, 132)
list_remove_wms.align(lv.ALIGN.TOP_LEFT, 0, 97)   
for i in range(len(pallets_dict)):
    checkbox_remove_wms = lv.checkbox(list_remove_wms)
    checkbox_remove_wms.set_text(str(pallets_dict["location%s"%i]["position"]) + ", " + pallets_dict["location%s"%i]["name"])
    if pallets_dict["location%s"%i]["box"] == False:
        checkbox_remove_wms.add_flag(lv.obj.FLAG.HIDDEN)
for i in range(len(conveyor_dict)):
    checkbox_remove_wms = lv.checkbox(list_remove_wms)
    checkbox_remove_wms.set_text(str(conveyor_dict["location%s"%(i+100)]["position"]) + ", " + conveyor_dict["location%s"%(i+100)]["name"])
    if conveyor_dict["location%s"%(i+100)]["box"] == False:
        checkbox_remove_wms.add_flag(lv.obj.FLAG.HIDDEN)
for i in range(len(robot_dict)):
    checkbox_remove_wms = lv.checkbox(list_remove_wms)
    checkbox_remove_wms.set_text(str(robot_dict["location%s"%(i+110)]["position"]) + ", " + robot_dict["location%s"%(i+110)]["name"])
    if robot_dict["location%s"%(i+110)]["box"] == False:    
        checkbox_remove_wms.add_flag(lv.obj.FLAG.HIDDEN)
list_remove_wms.add_flag(lv.obj.FLAG.HIDDEN)


##############################WMS PAGE##############################

tile_WMS = page_tiles.add_tile(2, 1, lv.DIR.LEFT | lv.DIR.TOP)
tabview = lv.tabview(tile_WMS, lv.DIR.BOTTOM, 25)
tab_request = tabview.add_tab("Request box")
tab_storeto = tabview.add_tab("Store box to")
tab_robotsto = tabview.add_tab("Robot storage")
   
list_request = lv.list(tab_request)
list_request.set_size(lv.pct(90), lv.pct(85))
list_request.align(lv.ALIGN.TOP_LEFT, 0, 16)   
for i in pallets_dict:
    if pallets_dict[i]["box"] == True:
        checkbox_request_wh = lv.checkbox(list_request)
        checkbox_request_wh.set_text(str(pallets_dict[i]["position"]) + ", " + pallets_dict[i]["name"])
        checkbox_request_wh.add_event_cb(update_request_list, lv.EVENT.CLICKED, None)
list_request.add_flag(lv.obj.FLAG.HIDDEN)


col_dsc = [45, 45, 45, 45, 45, lv.GRID_TEMPLATE.LAST]
row_dsc = [19, 19, 19, 19, 19, 40, 19, 19, 19, 19, 19, 19, lv.GRID_TEMPLATE.LAST]
cont_storeto = lv.obj(tab_storeto)
cont_storeto.set_style_grid_column_dsc_array(col_dsc, 0)
cont_storeto.set_style_grid_row_dsc_array(row_dsc, 0)
cont_storeto.set_size(lv.pct(95), lv.pct(92))
cont_storeto.align(lv.ALIGN.TOP_LEFT, 0, 16)
cont_storeto.set_layout(lv.LAYOUT_GRID.value)
for i in range(total_wh_positions):
    col = i // (levels * 2)
    row = i % (levels * 2)
    checkbox_storeto_wh = lv.checkbox(cont_storeto)
    checkbox_storeto_wh.set_grid_cell(lv.GRID_ALIGN.STRETCH, col, 1, lv.GRID_ALIGN.STRETCH, row, 1)
    checkbox_storeto_wh.set_text(str(59-i))
    if pallets_dict["location%s"%i]["box"] == True:
        checkbox_storeto_wh.add_state(lv.STATE.CHECKED | lv.STATE.DISABLED)
    checkbox_storeto_wh.add_event_cb(update_storeto_list, lv.EVENT.CLICKED, None)
cont_storeto.add_flag(lv.obj.FLAG.HIDDEN)


list_robotsto = lv.list(tab_robotsto)
list_robotsto.set_size(lv.pct(90), lv.pct(85))
list_robotsto.align(lv.ALIGN.TOP_LEFT, 0, 16)   
for i in robot_dict:
    if robot_dict[i]["box"] == True:
        checkbox_robot_sto = lv.checkbox(list_robotsto)
        checkbox_robot_sto.set_text(str(robot_dict[i]["position"]) + ", " + robot_dict[i]["name"])
        checkbox_robot_sto.add_event_cb(update_request_list, lv.EVENT.CLICKED, None)
list_robotsto.add_flag(lv.obj.FLAG.HIDDEN)


##############################SCROLLING ERROR BAR AT THE TOP OF THE SCREEN ON EVERY PAGE##############################

error_box = lv.label(lv.scr_act())
error_box.align(lv.ALIGN.TOP_LEFT, 0, 5)
error_box.set_long_mode(lv.label.LONG.SCROLL_CIRCULAR)
error_box.set_width(lv.pct(95))
error_box.set_recolor(True)
error_box.set_text("Error messages shown here")

def update_error_label():
    global last_shown_errors
    global robot_homing_start
    
    error_string = ""
    error_clr = []
    
    if last_shown_errors != machine_errors:
        del last_shown_errors
        last_shown_errors = []
        for i in machine_errors:
            last_shown_errors.append(i)
        if machine_errors == []:
            error_box.set_text("#00FF00 No machine errors#")
            return
        elif "Emergency Stop pushed" in machine_errors:
            if len(machine_errors) > 1:
                error_string = "#ff0000 Emergency Stop pushed# - - - "
            else:
                error_string = "#ff0000 Emergency Stop pushed#"
                error_box.set_text(error_string)
                return
        for i in machine_errors:
            if i != "Emergency Stop pushed":
                error_clr.append("#FFA500 " + i + "# - - - " )
            error_box.set_text(error_string + "".join(error_clr))
        if "Communication not online" not in machine_errors and robot_homing_start == "Not started" and "Full warehouse with robot" in selection:
            robot_homing_start = "Partial"
            cont_manual_homing.clear_flag(lv.obj.FLAG.HIDDEN)
            dropdown_manual.clear_flag(lv.obj.FLAG.HIDDEN)
            manual_label.set_text("Safely homing robot")
        elif "Communication not online" not in machine_errors and not "Full warehouse with robot" in selection:
            dropdown_manual.clear_flag(lv.obj.FLAG.HIDDEN)


half_second = False    
while True:
    try: ur.process_uart()
    except: print("Some error receiving communication")
    ur.flush()
    if math.fmod(time.ticks_ms(), 1000) > 500:
        if half_second == False:
            half_second = True
            wh_location_states_check()
            light_tower_check()
            update_error_label()
            #print("reaction time:", math.fmod(time.ticks_ms(), 1000) - 500, "over half")
    elif half_second == True:
        half_second = False
        wh_location_states_check()
        light_tower_check()
        update_error_label()
        #print("reaction time:", math.fmod(time.ticks_ms(), 1000), "under half ---")


   
#while True:
#    for i in range(5):
#        light_tower_check()
#        for x in range(10):
#            try: ur.process_uart()
#            except: print("Some error receiving communication")
#            ur.flush()
#            #sleep_ms(7)
#    wh_location_states_check()
#    update_error_label()