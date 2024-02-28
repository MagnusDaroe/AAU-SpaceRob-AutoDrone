import subprocess
import asyncio
from mavsdk import System
from serial.tools.list_ports import comports
import serial

ser = serial.Serial()

def find_serial_port():
    ports = list(comports())
    for p in ports:
        if 'Seriel USB-enhed' in p.description:
            port = "serial://{}:57600".format(p.device)
            return port
    # If no port is found
    print("No serial port found")
    return None

def connect_to_drone():
    # Call the function to find the serial port
    Drone_port = find_serial_port()
    if Drone_port is not None:
        print("Drone device found at port:", Drone_port)
    elif Drone_port is None:
        print("No serial port found")
        print("Please connect the drone to the computer")
        # Exit the program
        exit()

async def main():
    # Start mavsdk_server and connect to the drone
    server_process = subprocess.Popen(['C:/MagnusProjekter/mavsdk-windows-x64-release/bin/mavsdk_server_bin', '-p', '50051'])  # Adjust the path and port as needed
    connect_to_drone()

    # Connect to the Pixhawk via USB
    drone = System(mavsdk_server_address=server_process, port=50051)
    await drone.connect(system_address="serial:///tty/COM3\[:57600\]")
    print("Connected to Pixhawk via USB")
    # Your code logic here...

    # Optionally, wait for the drone operation to complete
    await asyncio.sleep(10)  # Adjust the time as needed

    # Terminate the mavsdk_server process
    server_process.terminate()
    print("mavsdk_server terminated")

# Run the main function asynchronously
asyncio.run(main())
