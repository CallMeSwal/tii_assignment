#!/usr/bin/env python3

import asyncio
from mavsdk import System

print("hi1")
async def run():
    print("hi2")
    drone = System()
    print("hi3")
    await drone.connect(system_address="udp://127.0.0.1:14556@:14557")
    print("hi4")
    print("Waiting for drone to connect...")
    print("hi5")
    async for state in drone.core.connection_state():
        print("hi6")
        if state.is_connected:
            print(f"-- Connected to drone!")
            break
    print("hi7")
    info = await drone.info.get_version()
    print(info)


if __name__ == "__main__":
    print("hi8")
    loop = asyncio.get_event_loop()
    print("hi9")
    loop.run_until_complete(run())