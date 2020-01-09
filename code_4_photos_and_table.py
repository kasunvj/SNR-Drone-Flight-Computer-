import subprocess as sp
import logging 
sp.call('cls',shell=True)
logging.basicConfig(format='%(asctime)s - %(message)s',level=logging.INFO)
logging.info("Flight Checking Procedures")
#info, debug,warning, eror,critical

#from picamera.array import PiRGBArray
#from picamera import PiCamera

import dronekit_sitl
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative # Import DroneKit-Python
from pymavlink import mavutil
import time 
import os
import numpy as np
import csv
import math

#Connection-------------------------------------------
#SITL
connection_string = '127.0.0.1:14550'
sitl = dronekit_sitl.start_default()

#Actual Drone over micro USB Direrctly
#(connection_string = '/dev/ttyACM0')

#connection_string = '192.168.43.220:14550'

#Camera-----------------------------------------------------
#camera = PiCamera()


#Initialization Variables-----------------------------------
alti = 15
waypoints = np.zeros((10,5))
current_waypoint = 0
current_photo = 0
test_com = False
photo_time_interval = 3

#methods----------------------------------------------------
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
'''
def get_non_zero_rows(matrix):
    # return the number of active (non zero) rows in a 2D matrix
    non_zero_rows = 0
    for i in range(0,matrix.shape[0]):
        if matrix[i][0] != 0:
            non_zero_rows = non_zero_rows +1
    return non_zero_rows


def load_waypoints():
    # 1st waypoint is the Starting Waypoint
    with open('waypoints.txt','rt') as file:
        waypoint_data = csv.reader(file)
        line = 0;
        for row in waypoint_data:
            for column in range(0,5):
                waypoints[line][column]= row[column]
            line = line + 1
    logging.info("waypoints loaded")

def get_gimbal_angle():
    gimbal_pitch = 45
    gimbal_roll = 0
    gimbal = [gimbal_pitch,gimbal_roll]
    return gimbal

def position_euclidian_dist(lat1,lon1,lat2,lon2):
	return math.sqrt(math.pow(lat2-lat1,2) + math.pow(lon2-lon1,2))

    
def height_euclidian_dist(h1,h2):
	return h2-h1


def timestamp(waypoint,number):
	if waypoint<10:
		name_waypoint = "0"+str(waypoint)
	else:
		name_waypoint = str(waypoint)

	if number<10:
		name_number = "0"+str(number)
	else:
		name_number = str(number)

	name = name_waypoint+name_number+" "+time.ctime()

	return name



#connection to FC-------------------------------------------

try:
    logging.info("Connecting to vehicle on: %s" % (connection_string,))
    vehicle = connect(connection_string, wait_ready=True)
    logging.info("Connection to FC: Go ")
    test_com = True
except:
    logging.info("Connection to FC: No go ")
    test_com = False




#Radio for emergency control---------------------------------

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
    


#Arming check-----------------------------------------------

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
while not test_com:
	logging.critical("Communication error")
	time.sleep(10)

print("\nGuidance is internal--------------------------\n")


#take off------------------------------------------------
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

time.sleep(2)

load_waypoints()

home_tag = vehicle.location.global_relative_frame
logging.info("Home tag saved alt: %s",home_tag.alt)

while (current_waypoint < get_non_zero_rows(waypoints)):
    # preparing 
    logging.info("Directing to the waypoint: %s",waypoints[current_waypoint][0])
    lat = waypoints[current_waypoint][2]
    lon = waypoints[current_waypoint][3]
    alt = waypoints[current_waypoint][4]
    #vehicle.airspeed = 3;
    point = LocationGlobalRelative(lat,lon,alt)
    vehicle.gimbal.rotate(get_gimbal_angle()[0],get_gimbal_angle()[1],0)
    logging.info("Gimble Pitch: %s Roll: %s",get_gimbal_angle()[0],get_gimbal_angle()[1])

    #sending moving command
    vehicle.simple_goto(point)

    #getting details of vehical location
    vehicle_location_info = vehicle.location.global_relative_frame
    lat0 = vehicle_location_info.lat
    lon0 = vehicle_location_info.lon
    alt0 = vehicle_location_info.alt  
    position_remaining = position_euclidian_dist(lat,lon,lat0,lon0)*100000
    alt_remaining = height_euclidian_dist(alt,alt0)
    print(round(position_remaining,2))
    print(round(alt_remaining,2))

    #image tagging
    photo_number = 0;
    time1 = time.time()

    while ((position_remaining > 5) or (abs(alt_remaining)> 1)):
        
        vehicle_location_info = vehicle.location.global_relative_frame
        lat0 = vehicle_location_info.lat
        lon0 = vehicle_location_info.lon
        alt0 = vehicle_location_info.alt
        position_remaining = position_euclidian_dist(lat,lon,lat0,lon0)*100000
        alt_remaining = height_euclidian_dist(alt,alt0)

        print("GPS Euclidian Dist:",round(position_remaining,2),"Alt remaining(m):",round(alt_remaining,2)) 

        
        
        time2 = time.time()
        if (time2 - time1)> photo_time_interval :
        	#takeing image
        	#camera.capture()
        	photo_number = photo_number + 1 
        	print("D:/Projects/Research_Project/SNR-Drone-Flight-Computer-/",timestamp(current_waypoint,photo_number),".jpg")
        	time1 = time2

        time.sleep(0.5)


    logging.info("waypont reach: %s", current_waypoint+1)


    current_waypoint = current_waypoint +1 
    #current_waypoint is the waypoint drone directing to

    



'''

send_ned_velociry(2,0,-5,5)

'''
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
'''

# time.sleep(30)


    

#----------------------------------------------------------

logging.info("Mission Done")


logging.info("Landing")
vehicle.mode = VehicleMode("LAND")

while vehicle.armed:
    time.sleep(1)



vehicle.close()
