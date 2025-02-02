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
fire_percentages = [] # List to store fire percentages
fire_detection_running = False # Flag to control fire detection loop

# Camera specifications
HORIZONTAL_FOV = 60  # degrees
VERTICAL_FOV = 45    # degrees
DRONE_ALTITUDE = 20  # feet

def detect_fire(frame):
    """
    Detects fire in the given frame and returns the percentage of the frame that is fire.
    """
    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define the range of fire color in HSV
    lower_fire = np.array([0, 50, 50])
    upper_fire = np.array([35, 255, 255])
    
    # Threshold the HSV image to get only fire colors
    mask = cv2.inRange(hsv, lower_fire, upper_fire)
    
    # Calculate the percentage of the frame that is fire
    fire_percentage = (cv2.countNonZero(mask) / (frame.size / 3)) * 100
    
    return fire_percentage

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

def calculate_fire_area(fire_percentage):
    """
    Calculates the area of the fire based on the percentage of the image flagged as fire.
    """
    ground_area = calculate_ground_area()
    fire_area = (fire_percentage / 100) * ground_area
    return fire_area

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
        print('Timeout!')

    except Exception as err:
        print(Exception, err)

def arm_and_takeoff(aTargetAltitude):
    global vehicle, picam2
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

    # Start fire detection
    start_fire_detection()

def start_fire_detection():
    global picam2, fire_percentages, fire_detection_running
    print("Starting fire detection...")
    fire_detection_running = True

    # Start capturing frames
    while fire_detection_running:
        frame = picam2.capture_array()  # Capture a frame using Picamera2

        # Detect fire in the frame
        fire_percentage = detect_fire(frame)
        fire_percentages.append(fire_percentage)  # Save the percentage to the list
        print(f"Fire detected in {fire_percentage:.2f}% of the frame")

        # Calculate the fire area based on the percentage
        fire_area = calculate_fire_area(fire_percentage)
        print(f"Estimated fire area: {fire_area:.2f} square feet")

        # Display the frame with fire detection result
        cv2.imshow("Fire Detection", frame)
        
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

def calculate_average_fire_percentage():
    """
    Calculates the average percentage of fire detected over time.
    """
    if not fire_percentages:
        return 0.0
    return sum(fire_percentages) / len(fire_percentages)

def calculate_average_fire_area():
    """
    Calculates the average fire area over time based on the stored percentages.
    """
    average_fire_percentage = calculate_average_fire_percentage()
    return calculate_fire_area(average_fire_percentage)

def disconnect_drone():
    global vehicle, picam2

    # Stop Picamera2 recording if still running
    if picam2 is not None:
        print("Stopping camera recording...")
        picam2.stop_recording()

    vehicle.close()
    print("drone disconnected")

def rtl():
    global vehicle, picam2, fire_detection_running
    vehicle.mode = VehicleMode("RTL")
    print("RTL initiated...")

    # Wait until RTL is complete and vehicle is landed
    while vehicle.mode == VehicleMode("RTL"):
        print("Waiting for RTL to complete...")
        sleep(1)

    # Signal to stop fire detection
    fire_detection_running = False

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

# Uncomment this to preview on-screen (optional)
# picam2.start_preview(Preview.QTGL)

arm_and_takeoff(20)
print("Ready to RTL")

# Add the message listener to the vehicle
vehicle.add_message_listener('VFR_HUD', vfr_hud_callback)

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

# Calculate and print the average fire percentage and area
average_fire_percentage = calculate_average_fire_percentage()
average_fire_area = calculate_average_fire_area()
print(f"Average fire percentage over time: {average_fire_percentage:.2f}%")
print(f"Average fire area over time: {average_fire_area:.2f} square feet")

### END OF EXECUTION ###
# if landed: stop recording
disconnect_drone()