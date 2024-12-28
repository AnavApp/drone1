from collections.abc import MutableMapping
import collections


collections.MutableMapping = collections.abc.MutableMapping
from dronekit import connect, VehicleMode
from picamera2 import Picamera2, Preview
from time import sleep
from datetime import datetime
vehicle = None


def connect_drone(connection_string, waitready=True, baudrate=57600):
    global vehicle
    if vehicle == None:
        vehicle = connect(connection_string, wait_ready=waitready, baud=baudrate)
    print("drone connected")

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
connect_drone("/dev/ttyACM0")
picam2 = Picamera2()
picam2.start_preview(Preview.QTGL)
# start recording

### INFINITE LOOP ###

#while True:
    #	flight modes: stabilize, althold, loiter, and rtl
#	if the state is liftoff for the first time, trigger camera recording
#	if vehicle.mode=LOITER: trigger object detection / inference
#	while vehicle.mode==LOITER and if there is banana: do nothing
#		if you stop seeing banana: issue rtl


#	check if there is landed property
    



rtl()
print(f"Vehicle mode is: {vehicle.mode}")

### END OF EXECUTION ###
# if landed: stop recording
disconnect_drone()






