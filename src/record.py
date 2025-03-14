import math
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

from picamera2 import Picamera2
import cv2
import numpy as np


vehicle = None # Global Vehicle instance
picam2 = None  # Global Picamera2 instance
ground_area = 0  # Ground area covered by the camera
area_detection_running = False  # Flag to control area detection loop

# Camera specifications for Arducam 12MP IMX708
HORIZONTAL_FOV = 102  # degrees (IMX708)
VERTICAL_FOV = 66    # degrees (IMX708)
DRONE_ALTITUDE = 20  # feet

def calculate_ground_area():
    """
    Calculates the ground area covered by the camera based on its FOV and the drone's altitude.
    """
    # Convert FOV from degrees to radians
    h_fov_rad = math.radians(HORIZONTAL_FOV)
    v_fov_rad = math.radians(VERTICAL_FOV)
    
    # Calculate the width and height of the ground area covered by the camera
    ground_width = 2 * DRONE_ALTITUDE * math.tan(h_fov_rad / 2)
    ground_height = 2 * DRONE_ALTITUDE * math.tan(v_fov_rad / 2)
    
    # Calculate the ground area
    ground_area = ground_width * ground_height
    return ground_area

def connect_drone(connection_string, waitready=True, baudrate=57600):
    global vehicle
    print("trying connection!")
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
        print('Timeout!')

    except Exception as err:
        print(Exception, err)

def arm_and_takeoff(aTargetAltitude):
    global vehicle, picam2, ground_area
    print("got to arm_and_takeoff")
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise... (vehicle is not armable)")
        sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    # after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: "), vehicle.location.global_relative_frame.alt
        print(vehicle.wind_speed)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")

            # Start recording using Picamera2
            print("Starting camera recording...")
            picam2.start_recording("target_altitude_reached.h264")

            break
        sleep(1)

    # Calculate the ground area
    ground_area = calculate_ground_area()
    print(f"Ground area covered by the camera: {ground_area:.2f} square feet")

    # Start area detection
    start_area_detection()

def start_area_detection():
    global picam2, area_detection_running
    print("Starting area detection...")
    area_detection_running = True

    # Start capturing frames
    while area_detection_running:
        frame = picam2.capture_array()  # Capture a frame using Picamera2

        # Display the frame
        cv2.imshow("Camera View", frame)
        
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

def disconnect_drone():
    global vehicle, picam2

    # Stop Picamera2 recording if still running
    if picam2 is not None:
        print("Stopping camera recording...")
        picam2.stop_recording()

    vehicle.close()
    print("drone disconnected")

def rtl():
    global vehicle, picam2, area_detection_running
    vehicle.mode = VehicleMode("RTL")
    print("RTL initiated...")

    # Wait until RTL is complete and vehicle is landed
    while vehicle.mode == VehicleMode("RTL"):
        print("Waiting for RTL to complete...")
        sleep(1)

    # Signal to stop area detection
    area_detection_running = False

    # Stop recording when RTL is done
    print("RTL complete, stopping camera recording...")
    picam2.stop_recording()

# Define a callback function to process the VFR_HUD message
def vfr_hud_callback(self, name, message):
    wind_speed = message.windspeed
    print(f"Current wind speed: {wind_speed} m/s")


#register this as a service
# main

### INITIALIZATION BLOCK ###
# connect to drone
connect_drone("/dev/ttyACM0") # for linux/raspberrypi
#connect_drone("/dev/tty.usbmodem01") # for mac

# Initialize Picamera2
picam2 = Picamera2()

# Configure Picamera2 for the Arducam 12MP IMX708 resolution (full resolution: 4056x3040)
config = picam2.create_still_configuration(main={"size": (4056, 3040)})
picam2.configure(config)

# Uncomment this to preview on-screen (optional)
# picam2.start_preview(Preview.QTGL)

arm_and_takeoff(20)
print("ready")
print("Ready to RTL")

# Add the message listener to the vehicle
vehicle.add_message_listener('VFR_HUD', vfr_hud_callback)

#picam2 = Picamera2()
#picam2.start_preview(Preview.QTGL)
# start recording

### INFINITE LOOP ###
print("RTL in 120 seconds")
sleep(120)
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