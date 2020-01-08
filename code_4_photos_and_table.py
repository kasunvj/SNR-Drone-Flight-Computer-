import subprocess as sp
import logging 
sp.call('cls',shell=True)
logging.basicConfig(format='%(asctime)s - %(message)s',level=logging.INFO)
logging.info("Flight Checking Procedures")
#info, debug,warning, eror,critical
from picamera.array import PiRGBArray
from picamera import PiCamera

import dronekit_sitl
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative # Import DroneKit-Python
from pymavlink import mavutil
import time 
import os
import numpy as np


#SITL
#connection_string = '127.0.0.1:14550'
#sitl = dronekit_sitl.start_default()

#Actual Drone over micro USB Direrctly
#connection_string = '/dev/ttyACM0'
connection_string = '192.168.43.220:14550'

#Camea
camera = PiCamera()

alti = 15

#connection to FC
'''
try:
    logging.info("Connecting to vehicle on: %s" % (connection_string,))
    vehicle = connect(connection_string, wait_ready=True)
    logging.info("Connection to FC: Go ")
except:
    logging.info("Connection to FC: No go ")
'''

#Radio for emergency control

'''

try:
    # Access channels individually
    t1= time.time()
    while(time.time()-t1 < 10):
        print("Read channels individually:",round(10-(time.time()-t1)))
        print(" Ch1:    Roll :    %s" % vehicle.channels['1'])
        print(" Ch2:    Pitch:    %s" % vehicle.channels['2'])
        print(" Ch3:    Throttle: %s" % vehicle.channels['3'])
        print(" Ch4:    Yaw:      %s" % vehicle.channels['4'])
        print(" Ch5: %s" % vehicle.channels['5'])
        print(" Ch6: %s" % vehicle.channels['6'])
        print(" Ch7: %s" % vehicle.channels['7'])
        print(" Ch8: %s" % vehicle.channels['8'])
        print("Number of channels: %s" % len(vehicle.channels))
        time.sleep(1)
    if(raw_input("See a change in values? y/n\n") == 'y'):
        logging.info("Radio Go")
        
    else:
        logging.info("Radio No Go")

except:
    logging.info("Radio : No Go")
    


#Arming check

try: 
    logging.info("Pre Arm check")
    while not vehicle.is_armable:
        logging.info("check- vehicle booted,good GPS Fix,EKF pre-arm complte")
        time.sleep(1)

    logging.info("Arming Motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        logging.info("Arming")
        time.sleep(1)
    
    t1= time.time()
    while(time.time()-t1 <10):
        print("Armed ",round(10 - (time.time()-t1)))
        time.sleep(0.5)
    vehicle.armed = False


    if (raw_input("Can you see arming? y/n\n") == 'y'):
        logging.info("Arming Go\n")

    else:
        logging.info("Arming No Go\n")

except:
    logging.info("Arming : No Go")
'''
print("Guidance is internal--------------------------\n\n")
'''

logging.info("Arming Motors")
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

while not vehicle.armed:
    logging.info("Arming")
    time.sleep(1)

time.sleep(5)
logging.info("Ready to take off")
vehicle.simple_takeoff(alti)

while True:
    print("Altitude: ", vehicle.location.global_relative_frame.alt)
    if vehicle.location.global_relative_frame.alt >= alti*0.95:
        logging.info("Altitude Reached")
        break
    time.sleep(1)
#----------------Simple Mission---------------------------------------------------
time.sleep(2)
'''
'''
def send_ned_velociry(velocity_x, velocity_y, velocity_z, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink))

    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

send_ned_velociry(2,0,-5,5)

'''

logging.info("Ready to move to 1st waypoint")
vehicle.airspeed = 3;

logging.info("Going towards first point for 30 seconds ...")
point1 = LocationGlobalRelative(6.7980226, 79.8995304, alti)
loging.info("Global Location:", vehicle.location.global_relative_frame)
vehicle.simple_goto(point1)
time.sleep(30)

vehicle.gimbal.rotate(45,0,0)
loging.info("Gimble done")

print("Going towards Second point for 30 seconds ...")
point1 = LocationGlobalRelative(6.7982676, 79.8999327, alti)
vehicle.simple_goto(point1)
# time.sleep(30)

while True:
    #mode:0-return to home location
    #mode:1-searching through way points
    #mode:2-go to a human detected location
    #
    gps_point = get_way_point(mode)# send mode,returns (lat,lon,attitude)
    point = LocationGlobalRelative(gps_point[0],gps_point[1] , gps_point[2])
    

#----------------------------------------------------------
'''
logging.info("Mission Done")


logging.info("Landing")
vehicle.mode = VehicleMode("LAND")

while vehicle.armed:
    time.sleep(1)

'''

while True:
    print(" Gimbal status: %s" % vehicle.gimbal)
    time.sleep(1)

vehicle.close()
