from collections.abc import MutableMapping
import collections
collections.MutableMapping = collections.abc.MutableMapping

import dronekit
from dronekit import connect, VehicleMode
#from picamera2 import Picamera2, Preview
from time import sleep
from datetime import datetime
import socket
#import exceptions


vehicle = None


def connect_drone(connection_string, waitready=True, baudrate=57600):
    global vehicle
    print("Got here!")
    try:
        if vehicle == None:
            vehicle = connect(connection_string, wait_ready=waitready, baud=baudrate)
        print("drone connected")
    # Bad TCP connection
    except socket.error:
        print('No server exists!')

    # Bad TTY connection
    except OSError as e:
        print('No serial exists!')

    # API Error
    except dronekit.APIException:
        print ('Timeout!')


    except Exception as err:
        print(Exception, err)

def arm_and_takeoff(aTargetAltitude):
    global vehicle
    print("got to arm_and_takeoff")
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: "), vehicle.location.global_relative_frame.alt
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print("Reached target altitude")
            break
        sleep(1)


def disconnect_drone():
    global vehicle
    vehicle.close()
    print("drone disconnected")
    
def rtl():
    vehicle.mode = VehicleMode("RTL")

#register this as a service
# main

### INITIALIZATION BLOCK ###
# connect to drone
#connect_drone("/dev/ttyACM0") # for linux/raspberrypi

connect_drone("/dev/tty.usbmodem01") # for mac
arm_and_takeoff(20)
print("Ready to RTL")
#picam2 = Picamera2()
#picam2.start_preview(Preview.QTGL)
# start recording

### INFINITE LOOP ###
print("RTL in 15 seconds")
sleep(15)
rtl()
#while True:
    #	flight modes: stabilize, althold, loiter, and rtl
#	if the state is liftoff for the first time, trigger camera recording
#	if vehicle.mode=LOITER: trigger object detection / inference
#	while vehicle.mode==LOITER and if there is banana: do nothing
#		if you stop seeing banana: issue rtl


#	check if there is landed property
    
print(f"Vehicle mode is: {vehicle.mode}")

### END OF EXECUTION ###
# if landed: stop recording
disconnect_drone()






