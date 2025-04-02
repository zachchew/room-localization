# import asyncio
# from mavsdk import System
# from mavsdk.offboard import OffboardError, PositionNedYaw
# import argparse

# async def run(x, y, z):
#     drone = System()
#     await drone.connect(system_address="udp://:14540")  # Change if using serial or other port

#     print("Waiting for drone...")
#     async for state in drone.core.connection_state():
#         if state.is_connected:
#             print("✅ Drone discovered!")
#             break

#     print("Waiting for drone to have a global/local position estimate...")
#     async for health in drone.telemetry.health():
#         if health.is_local_position_ok:
#             print("✅ Local position is good.")
#             break

#     print("Arming drone...")
#     await drone.action.arm()

#     print("Setting initial Offboard setpoint...")
#     await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -z, 0.0))  # Z down in NED
#     await asyncio.sleep(1)

#     try:
#         print("Starting Offboard mode...")
#         await drone.offboard.start()
#     except OffboardError as error:
#         print(f"❌ Failed to start Offboard: {error._result.result}")
#         await drone.action.disarm()
#         return

#     print(f"Flying to position: X={x}, Y={y}, Z={z} (NED)")
#     await drone.offboard.set_position_ned(PositionNedYaw(x, y, -z, 0.0))

#     await asyncio.sleep(10)

#     print("Holding position...")
#     await asyncio.sleep(10)

#     print("Stopping Offboard mode...")
#     await drone.offboard.stop()
#     await drone.action.land()

# if __name__ == "__main__":
#     parser = argparse.ArgumentParser()
#     parser.add_argument("--x", type=float, default=0.0)
#     parser.add_argument("--y", type=float, default=0.0)
#     parser.add_argument("--z", type=float, default=1.0)
#     args = parser.parse_args()

#     asyncio.run(run(args.x, args.y, args.z))
