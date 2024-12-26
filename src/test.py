from collections.abc import MutableMapping
import collections

collections.MutableMapping = collections.abc.MutableMapping
from dronekit import connect, VehicleMode

vehicle = None

def connect_drone(connection_string, waitready=True, baudrate=57600):
    global vehicle
    if vehicle == None:
        vehicle = connect(connection_string, wait_ready=waitready, baud=baudrate)
    print("drone connected")

connect_drone("/dev/ttyACM0")


print("Autopilot Firmware version: %s" % vehicle.version)


def disconnect_drone():
    global vehicle
    vehicle.close()
    #print("drone disconnected")

#disconnect_drone()

def rtl():
    vehicle.mode = VehicleMode("RTL")

vehicle.simple_takeoff(10)

for i in range(5):
    print(f"Last heartbeat is: {vehicle.last_heartbeat}")

rtl()

for i in range(5):
    print(f"Last heartbeat is: {vehicle.last_heartbeat}")

print(f"Vehicle mode is: {vehicle.mode}")