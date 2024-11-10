#Code to perform the Kamikaze Attack

from __future__ import print_function
import cv2
import json
import math
import time

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from calc_gps_3 import gps

# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect("127.0.0.1:14550", wait_ready=True)

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration, drone):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = drone.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        drone.send_mavlink(msg)
        time.sleep(1)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    
    
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    
    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
                             
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
    
    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)
       
#Altitude will be fixed at 15m
alt = 15
    
arm_and_takeoff(alt)

time.sleep(5)
backup = 0

def get_cordinates():
    global backup
    try:
        fileObject = open('/home/rithwick11111/Desktop/GlobalHawks/Simulations/Kamikaze/Round_3/coordinates.json', 'r+')
        jsonData = json.load(fileObject)
        fileObject.close()
        backup = jsonData
        return jsonData
    except:
        return backup


def get_current_location():
    vehicle.currentlocation = vehicle.location.global_frame
    x0 = vehicle.location.global_frame.lat
    y0 = vehicle.location.global_frame.lon
    return x0, y0


def get_targetpoint(waypointx, waypointy):
    px,py = get_cordinates()
    print('centroid of bbox: ',px,py)
    x,y = gps.compute_gps(gps, px, py, 3, 15, x0, y0, (640,480,0))
    print("Target coordinates are:", x, y)
    xt = 2*waypointx - x
    yt = 2*waypointy - y
    print("Supposed target coordinates are:", xt, yt)
    targetpoint = LocationGlobalRelative(xt, yt, -23)
    return targetpoint


x0,y0 = get_current_location()

waypoint1 = LocationGlobalRelative(-35.36278792, 149.16516927, alt)

print("Going to first waypoint")
vehicle.simple_goto(waypoint1, groundspeed=2)
vehicle.simple_goto(waypoint1, groundspeed=2)
time.sleep(30)
print("First waypoint reached!")
time.sleep(5)
print("No Object Detected")

wp2x = -35.36280674
wp2y = 149.16453309
wp3x = -35.36326910
wp3y = 149.16450343
hx = -35.36326215
hy = 149.16523585

waypoint2 = LocationGlobalRelative(wp2x, wp2y, alt)
waypoint3 = LocationGlobalRelative(wp3x, wp3y, alt)
home      = LocationGlobalRelative(hx, hy, alt)

print("Going to second waypoint")
vehicle.simple_goto(waypoint2, groundspeed=2)
vehicle.simple_goto(waypoint2, groundspeed=2)
time.sleep(30)

px,py = get_cordinates()
time.sleep(0.001)

if (px==0 and py==0):
    print("No Object Detected")
    print("Second waypoint reached!")

    print("Going to third waypoint")
    vehicle.simple_goto(waypoint3, groundspeed=2)
    vehicle.simple_goto(waypoint3, groundspeed=2)
    time.sleep(30)
    px,py = get_cordinates()
    time.sleep(0.001)
    print(px,py)
    if (px==0 and py==0):
        print("No Object Detected")
        print("Third waypoint reached!")
    
        print("Going home")
        vehicle.simple_goto(home, groundspeed=2)
        vehicle.simple_goto(home, groundspeed=2)
        time.sleep(30)
        px,py = get_cordinates()
        time.sleep(0.001) 
        print(px,py)
        if (px==0 and py==0):
            print("No Object Detected")
            print("Reached Home!")
        
        elif (px!=0 and py!=0): 
            print("Object Detected")
            print("Performing kamikaze!")
            vehicle.simple_goto(get_targetpoint(hx,hy), groundspeed=20)
            print("Kamikaze Attack accomplished!")
            xc, yc = get_current_location()
            print("Drone is at",xc,yc)

    elif (px!=0 and py!=0): 
        print("Object Detected")
        print("Performing kamikaze!")
        vehicle.simple_goto(get_targetpoint(wp3x,wp3y), groundspeed=20)
        print("Kamikaze Attack accomplished!")
        xc, yc = get_current_location()
        print("Drone is at",xc,yc)

elif (px!=0 and py!=0): 
    print("Object Detected")
    print("Performing kamikaze!")
    vehicle.simple_goto(get_targetpoint(wp2x,wp2y), groundspeed=10)
    print("Kamikaze Attack accomplished!")
    xc, yc = get_current_location()
    print("Drone is at",xc,yc)

# Shut down simulator if it was started.
if sitl:
    sitl.stop()
