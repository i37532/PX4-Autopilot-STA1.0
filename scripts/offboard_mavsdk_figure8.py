#!/usr/bin/env python3
import asyncio
import math
import time

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


SETPOINT_HZ = 20.0
SETPOINT_DT = 1.0 / SETPOINT_HZ

FIGURE8_DURATION_S = 30.0
FIGURE8_AMP_NORTH_M = 10.0
FIGURE8_AMP_EAST_M = 10.0


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for vehicle connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected.")
            break

    print("Waiting for local position and armable state...")
    async for health in drone.telemetry.health():
        if health.is_local_position_ok and health.is_armable:
            print("Local position OK and armable.")
            break

    current_pos = None

    async def position_listener():
        nonlocal current_pos
        async for pv in drone.telemetry.position_velocity_ned():
            current_pos = pv.position

    pos_task = asyncio.create_task(position_listener())

    while current_pos is None:
        await asyncio.sleep(0.1)

    async def get_initial_yaw():
        async for att in drone.telemetry.attitude_euler():
            return math.radians(att.yaw_deg)

    yaw = await get_initial_yaw()
    print(f"Using initial yaw: {math.degrees(yaw):.1f} deg")
    start_pos = current_pos

    async def send_setpoint(pos, duration_s):
        end_time = time.time() + duration_s
        while time.time() < end_time:
            await drone.offboard.set_position_ned(pos)
            await asyncio.sleep(SETPOINT_DT)

    async def goto_position(target, timeout_s=30.0, tolerance_m=0.3):
        end_time = time.time() + timeout_s
        while time.time() < end_time:
            await drone.offboard.set_position_ned(target)
            if current_pos is not None:
                dx = target.north_m - current_pos.north_m
                dy = target.east_m - current_pos.east_m
                dz = target.down_m - current_pos.down_m
                if math.sqrt(dx * dx + dy * dy + dz * dz) <= tolerance_m:
                    break
            await asyncio.sleep(SETPOINT_DT)

    async def fly_figure8(center, duration_s, amp_n, amp_e):
        start_time = time.time()
        while True:
            t = time.time() - start_time
            if t >= duration_s:
                break
            theta = 2.0 * math.pi * t / duration_s
            north = center.north_m + amp_n * math.sin(theta)
            east = center.east_m + amp_e * math.sin(2.0 * theta)
            down = center.down_m
            await drone.offboard.set_position_ned(PositionNedYaw(north, east, down, yaw))
            await asyncio.sleep(SETPOINT_DT)

    # Send a few setpoints before starting offboard
    initial_sp = PositionNedYaw(start_pos.north_m, start_pos.east_m, start_pos.down_m, yaw)
    for _ in range(int(1.0 / SETPOINT_DT)):
        await drone.offboard.set_position_ned(initial_sp)
        await asyncio.sleep(SETPOINT_DT)

    print("Arming...")
    await drone.action.arm()

    print("Starting offboard...")
    try:
        await drone.offboard.start()
    except OffboardError as exc:
        print(f"Offboard start failed: {exc._result.result}")
        await drone.action.disarm()
        pos_task.cancel()
        return

    target_takeoff = PositionNedYaw(start_pos.north_m, start_pos.east_m, -10.0, yaw)
    print("Taking off to 10 m...")
    await goto_position(target_takeoff, timeout_s=30.0, tolerance_m=0.5)

    print("Hover 10 s...")
    await send_setpoint(target_takeoff, 10.0)

    print("Flying figure-8 path...")
    await fly_figure8(target_takeoff, FIGURE8_DURATION_S, FIGURE8_AMP_NORTH_M, FIGURE8_AMP_EAST_M)

    print("Hover 15 s...")
    await send_setpoint(target_takeoff, 15.0)

    target_down = PositionNedYaw(target_takeoff.north_m, target_takeoff.east_m, -0.5, yaw)
    print("Descending...")
    await goto_position(target_down, timeout_s=30.0, tolerance_m=0.5)

    print("Stopping offboard and landing...")
    await drone.offboard.stop()
    await drone.action.land()

    pos_task.cancel()


if __name__ == "__main__":
    asyncio.run(run())
