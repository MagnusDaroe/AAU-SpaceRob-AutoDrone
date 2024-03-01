import asyncio
from mavsdk import *


async def run():
    # Connect to the drone
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyACM2")

    # Create tasks to run both print_armed functions concurrently
    task1 = asyncio.create_task(print_armed1(drone))
    task2 = asyncio.create_task(print_armed2(drone))

    # Wait for both tasks to complete
    await asyncio.gather(task1, task2)
    

async def print_armed2(drone):
    # Check if it's armed (true or false)
    async for is_armed in drone.telemetry.armed():
        print(f"-----------")
        #print(f"-- Armed: {is_armed}")

async def print_armed1(drone):
    # Check if it's armed (true or false)
    async for attitude in drone.telemetry.attitude_euler():
        roll = attitude.roll_deg
        pitch = attitude.pitch_deg
        yaw = attitude.yaw_deg
        print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
        # Add a delay to avoid overwhelming the event loop
        #await asyncio.sleep(1)

asyncio.run(run())
