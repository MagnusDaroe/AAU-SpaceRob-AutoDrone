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

# Call the function to find the serial port
Drone_port = find_serial_port()
if Drone_port is not None:
    print("Drone device found at port:", Drone_port)
    print("Port address:", Drone_port)
