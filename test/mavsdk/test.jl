using PyCall


py"""
import asyncio
from mavsdk import System

async def run():
    # Init the drone
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # Start the tasks
    asyncio.ensure_future(print_battery(drone))

async def print_battery(drone):
    async for battery in drone.telemetry.battery():
        print(f"Battery: {battery.remaining_percent}")
"""

# Start the main function
py"asyncio.ensure_future(run())"
# Runs the event loop until the program is canceled with e.g. CTRL-C
py"asyncio.get_event_loop().run_forever()()
