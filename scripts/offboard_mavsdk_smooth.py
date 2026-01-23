#!/usr/bin/env python3
import asyncio
import math
import time

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


SETPOINT_HZ = 20.0
SETPOINT_DT = 1.0 / SETPOINT_HZ

MAX_VEL_MPS = 1.0
MIN_SEG_TIME_S = 2.0


def _quintic_blend(t: float) -> float:
    # 0..1 -> 0..1 with zero vel/acc at endpoints
    return 10.0 * t ** 3 - 15.0 * t ** 4 + 6.0 * t ** 5


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

    async def send_setpoint(pos, duration_s):
        end_time = time.time() + duration_s
        while time.time() < end_time:
            await drone.offboard.set_position_ned(pos)
            await asyncio.sleep(SETPOINT_DT)

    async def smooth_move(start, target, max_vel=MAX_VEL_MPS, min_duration=MIN_SEG_TIME_S):
        dx = target[0] - start[0]
        dy = target[1] - start[1]
        dz = target[2] - start[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        duration = max(min_duration, dist / max_vel if max_vel > 0.0 else min_duration)
        steps = max(1, int(duration / SETPOINT_DT))
        for i in range(steps + 1):
            t = i / steps
            s = _quintic_blend(t)
            north = start[0] + s * dx
            east = start[1] + s * dy
            down = start[2] + s * dz
            await drone.offboard.set_position_ned(PositionNedYaw(north, east, down, yaw))
            await asyncio.sleep(SETPOINT_DT)

    def pos_tuple(pos):
        return (pos.north_m, pos.east_m, pos.down_m)

    # Send a few setpoints before starting offboard
    start_pos = current_pos
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

    # Takeoff to 10 m
    target_takeoff = (start_pos.north_m, start_pos.east_m, -10.0)
    print("Taking off to 10 m (smooth)...")
    await smooth_move(pos_tuple(current_pos), target_takeoff)

    print("Hover 10 s...")
    await send_setpoint(PositionNedYaw(*target_takeoff, yaw), 10.0)

    # Move +X (north) 10 m
    target_move = (target_takeoff[0] + 10.0, target_takeoff[1], target_takeoff[2])
    print("Moving +X (north) 10 m (smooth)...")
    await smooth_move(pos_tuple(current_pos), target_move)

    print("Hover 15 s...")
    await send_setpoint(PositionNedYaw(*target_move, yaw), 15.0)

    # Descend near ground
    target_down = (target_move[0], target_move[1], -0.5)
    print("Descending (smooth)...")
    await smooth_move(pos_tuple(current_pos), target_down)

    print("Stopping offboard and landing...")
    await drone.offboard.stop()
    await drone.action.land()

    pos_task.cancel()


if __name__ == "__main__":
    asyncio.run(run())
