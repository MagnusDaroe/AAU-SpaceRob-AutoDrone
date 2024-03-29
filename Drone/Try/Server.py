import subprocess
import asyncio
from mavsdk import System
from serial.tools.list_ports import comports
import time
from mavsdk.action import OrbitYawBehavior

ServerAdress = 'C:/Users/simon/Documents/Robotteknologi/4_semester/P4/Mavsdk_server/mavsdk-windows-x64-release/bin/mavsdk_server_bin' # Adjust the path as needed
port = '50051'  # Adjust the port as needed
USB_search_string = 'ArduPilot'  # Adjust the search string as needed
serial_port = "COM7"
baud_rate = 57600

async def print_altitude(drone):
    async for altitude in drone.telemetry.altitude():
        print(altitude)

async def print_battery(drone):
    async for battery in drone.telemetry.battery():
        print(f"Battery: {battery.remaining_percent}")

async def print_gps_info(drone):
    async for gps_info in drone.telemetry.gps_info():
        print(f"GPS info: {gps_info}")

async def print_in_air(drone):
    async for in_air in drone.telemetry.in_air():
        print(f"In air: {in_air}")

async def print_position(drone):
    async for position in drone.telemetry.position():
        print(position)

def connect_to_drone():
    while True:
        try:
            port = next("serial://{}:57600".format(p.device) for p in comports() if USB_search_string in p.description)
            print("Drone device found at port:", port)
            return port
        except StopIteration:
            print("No serial port found. Trying again in 5 seconds...")
            time.sleep(5)

async def main():
    # Start mavsdk_server and connect to the drone
    drone_port = connect_to_drone()
    server_process = subprocess.Popen([ServerAdress, '-p', port, drone_port])  # Adjust the path and port as needed
    time.sleep(2)  # Let the server start before trying to connect

    # Connect to the Pixhawk via USB
    drone = System(mavsdk_server_address="localhost")
    
    # Connect to the drone
    print("trying to connect")
    await drone.connect(system_address=drone_port)  
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("We are connected")
            break

    # arm drone
    print("Arming...")
    await drone.action.arm()
    print("Armed")

    # Start the tasks
    #asyncio.ensure_future(print_battery(drone))
    #asyncio.ensure_future(print_gps_info(drone))
    #asyncio.ensure_future(print_in_air(drone))
    #asyncio.ensure_future(print_position(drone))
    print("Printing data")
    await print_altitude(drone) 

    # wait for 5 seconds
    await asyncio.sleep(5)

    print("Disarming...")
    await drone.action.disarm()
    print("Disarmed")
   


    await asyncio.sleep(10)  # Adjust the time as needed

    # Terminate the mavsdk_server process
    #server_process.terminate()
    print("mavsdk_server terminated")

# Run the main function asynchronously
asyncio.run(main())
