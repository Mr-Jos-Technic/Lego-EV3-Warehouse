# Lego EV3 Warehouse
MicroPython programs for my EV3 / ESP powered LEGO warehouse

This is where I will publish all my free to use micropython programs involved in my large automated Lego warehouse.

Current setup uses;
- 5x EV3 controllers with PyBricks MicroPython and UARTremote by Anton's Mindstorms (Copyright (c) 2021 Anton's Mindstorms)
- 1x ESP32 with UARTremote by Anton's Mindstorms (Copyright (c) 2021 Anton's Mindstorms)
- 114 NeoPixels LED's (36x3 and 1x6) from Anton Mindstorm's Hacks (Copyright (c) 2021 Anton's Mindstorms)
- 1x 2.8" Touchscreen using LVGL library connected to the ESP32 with TFT breakout board and TFT shield (for 2.4") (Copyright (c) 2022 Ste7an)
- Custom made EV3 cables. https://www.youtube.com/watch?v=cbXq8OteGAs
  
 Current setup video:
 
 [![Lego EV3 Warehouse on YouTube](https://img.youtube.com/vi/eiBUa0sTVF0/0.jpg)](https://www.youtube.com/watch?v=eiBUa0sTVF0 "Lego EV3 Warehouse on YouTube")
 
 In total 6 MicroPython programs are running at the same time;
- (1) Warehouse_Roll_Conveyors_v1   (Master EV3, bluetooth connection to (2),(3) and (4). UART connection to (6) )
- (2) Warehouse_Chain_Conveyors_v1  (Slave EV3,  bluetooth connection to (1) )
- (3) Warehouse_Stacker_Crane_v1    (Slave EV3,  bluetooth connection to (1) )
- (4) Warehouse_Righttside_Robot_v1 (Slave EV3,  bluetooth connection to (1) and (5) )
- (5) Warehouse_Leftside_Robot_v1   (Master EV3, bluetooth connection to (4) )
- (6) Main                          (ESP32, UART connection to (1) )
  
Startup-order to run the full warehouse: (6) wait for startup -> (5) + (1) wait for mode selection -> (2) + (3) + (4)

All my programs/machines are just made for fun and demonstrating what plastic bricks are capable of doing.  
# Any possible similarities to real machines are purely coincidental
