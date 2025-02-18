import math
from collections.abc import MutableMapping
import collections
collections.MutableMapping = collections.abc.MutableMapping

import dronekit
from dronekit import connect, VehicleMode
from time import sleep
from datetime import datetime
import socket
import cv2
import numpy as np
from picamera2 import Picamera2


vehicle = None  # Global Vehicle instance
picam2 = None  # Global Picamera2 instance
ground_area = []  # Ground area covered by the camera
area_detection_running = False  # Flag to control area detection loop

# Camera specifications for Arducam 12MP IMX708
HORIZONTAL_FOV = 102  # degrees (IMX708)
VERTICAL_FOV = 66  # degrees (IMX708)


"""
Table of Contents:
1. calculate_ground_area() - Calculates the ground area covered by the camera.
2. connect_drone() - Connects to the drone.
3. start_area_detection() - Starts area detection by capturing frames.
4. disconnect_drone() - Disconnects from the drone.
5. rtl() - Initiates Return to Launch (RTL) mode.
6. Initialization Block - Initializes the drone connection and camera.
7. Infinite Loop - Waits for LOITER mode, starts area detection, and initiates RTL.
"""

def calculate_ground_area():
    """
    Calculates the ground area covered by the camera based on its FOV and the drone's altitude.
    """
    # Convert FOV from degrees to radians
    h_fov_rad = math.radians(HORIZONTAL_FOV)
    v_fov_rad = math.radians(VERTICAL_FOV)
    
    # Calculate the width and height of the ground area covered by the camera
    DRONE_ALTITUDE = vehicle.vehicle.location.global_relative_frame.alt  # feet
    ground_width = 2 * DRONE_ALTITUDE * math.tan(h_fov_rad / 2)
    ground_height = 2 * DRONE_ALTITUDE * math.tan(v_fov_rad / 2)
    
    # Calculate the ground area
    ground_area = ground_width * ground_height
    return ground_area

def connect_drone(connection_string, waitready=True, baudrate=57600):
    """
    Connects to the drone using the provided connection string and baudrate.
    """
    global vehicle
    print("Trying to connect to the drone...")
    try:
        if vehicle is None:
            vehicle = connect(connection_string, wait_ready=waitready, baud=baudrate)
        print("Drone connected")
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

def start_area_detection():
    """
    Starts area detection by capturing frames from the camera and calculating the ground area.
    """
    global picam2, area_detection_running, ground_area
    print("Starting area detection...")
    area_detection_running = True
    counter = 0
    # Start capturing frames
    while area_detection_running:
        frame = picam2.capture_array()  # Capture a frame using Picamera2

        # Display the frame
        cv2.imshow("Camera View", frame)

        # Calculate the ground area
        area = calculate_ground_area()
        ground_area.append(area)
        print(f"Ground area covered by the camera: {ground_area:.2f} square feet")

        # Wait for 10 seconds before capturing the next frame
        sleep(10)
        counter += 1

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if counter == 2:
            break
    cv2.destroyAllWindows()

def disconnect_drone():
    """
    Disconnects from the drone and stops the camera recording if it is running.
    """
    global vehicle, picam2

    # Stop Picamera2 recording if still running
    if picam2 is not None:
        print("Stopping camera recording...")
        picam2.stop_recording()

    vehicle.close()
    print("Drone disconnected")

def rtl():
    """
    Initiates Return to Launch (RTL) mode and waits until the drone completes RTL and lands.
    """
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

# Register this as a service
# main

### INITIALIZATION BLOCK ###
# Connect to drone
connect_drone("/dev/ttyACM0")  # for linux/raspberrypi
# connect_drone("/dev/tty.usbmodem01") # for mac

try:
    print("vehicle connection:", vehicle.battery)
except:
    print("vehicle is not connected")

# Initialize Picamera2
picam2 = Picamera2()

# Configure Picamera2 for the Arducam 12MP IMX708 resolution (full resolution: 4056x3040)
config = picam2.create_still_configuration(main={"size": (4056, 3040)})
picam2.configure(config)

# Uncomment this to preview on-screen (optional)
# picam2.start_preview(Preview.QTGL)

print("Ready to start area detection and RTL")

### INFINITE LOOP ###
# Wait until vehicle is in LOITER mode
while vehicle.mode != VehicleMode("LOITER"):
    print(f"Waiting for LOITER mode. Current mode: {vehicle.mode}")
    sleep(1)

# Start area detection
start_area_detection()

# Wait for 1 minute (60 seconds)
sleep(60)

# Initiate RTL
rtl()

print(f"Vehicle mode is: {vehicle.mode}")
print("Areas are:", ground_area)
### END OF EXECUTION ###
# Disconnect from drone
disconnect_drone()