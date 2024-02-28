import subprocess
import asyncio
from mavsdk import System
from serial.tools.list_ports import comports
import time

ServerAdress = 'C:/MagnusProjekter/mavsdk-windows-x64-release/bin/mavsdk_server_bin' # Adjust the path as needed
port = '50051'  # Adjust the port as needed
USB_search_string = 'Seriel USB-enhed'  # Adjust the search string as needed

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
    server_process = subprocess.Popen([ServerAdress, '-p', port])  # Adjust the path and port as needed
    drone_port = connect_to_drone()

    # Connect to the Pixhawk via USB
    drone = System(mavsdk_server_address=f"localhost:{port}")
    await drone.connect(system_address=drone_port)
    print("Connected to Pixhawk via USB")
    # Your code logic here...

    # Optionally, wait for the drone operation to complete
    await asyncio.sleep(10)  # Adjust the time as needed

    # Terminate the mavsdk_server process
    server_process.terminate()
    print("mavsdk_server terminated")

# Run the main function asynchronously
asyncio.run(main())
