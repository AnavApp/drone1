from collections.abc import MutableMapping
import collections
import psutil

collections.MutableMapping = collections.abc.MutableMapping

collections.MutableMapping = collections.abc.MutableMapping
my_pid = None
pids = psutil.pids()
print ("Start simulator (SITL)")
import dronekit_sitl
sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()

# Import DroneKit-Python
from dronekit import connect, VehicleMode

# Connect to the Vehicle.
print("Connecting to vehicle on: %s" % (connection_string,))
#vehicle = connect(connection_string, wait_ready=True)
vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)
for pid in pids:
    try:
        ps = psutil.Process(pid)
        name = ps.name()
    except psutil.NoSuchProcess:  # Catch the error caused by the process no longer existing
        pass  # Ignore it
    else:
        if "solitaire.exe" in name:
            print(f"{name} running with pid: {pid}")

# Get some vehicle attributes (state)


print ("Get some vehicle attribute values:")
print (" GPS: %s" % vehicle.gps_0)
print (" Battery: %s" % vehicle.battery)
print (" Last Heartbeat: %s" % vehicle.last_heartbeat)
print (" Is Armable?: %s" % vehicle.is_armable)
print (" System status: %s" % vehicle.system_status.state)
print (" Mode: %s" % vehicle.mode.name)    # settable

# Close vehicle object before exiting script
vehicle.close()

# Shut down simulator
sitl.stop()
print("Completed")