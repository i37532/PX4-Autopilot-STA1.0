#!/usr/bin/env python3
import asyncio
import math
import time

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
from mavsdk.telemetry import LandedState


SETPOINT_HZ = 20.0
SETPOINT_DT = 1.0 / SETPOINT_HZ

MAX_VEL_MPS = 1.0
MIN_SEG_TIME_S = 2.0

TAKEOFF_ALT_M = 5.0
HOVER_TIME_S = 10.0
DESCEND_ALT_M = 0.5
YAW_AFTER_TAKEOFF_DEG = None  # Set to a number to rotate after takeoff; None keeps initial yaw.


def _quintic_blend(t: float) -> float:
    # 0..1 -> 0..1 with zero vel/acc at endpoints
    return 10.0 * t ** 3 - 15.0 * t ** 4 + 6.0 * t ** 5


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    # await drone.connect(system_address="serial:///dev/ttyUSB0:57600")

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
    current_yaw = None

    async def position_listener():
        nonlocal current_pos
        async for pv in drone.telemetry.position_velocity_ned():
            current_pos = pv.position

    pos_task = asyncio.create_task(position_listener())

    async def yaw_listener():
        nonlocal current_yaw
        async for att in drone.telemetry.attitude_euler():
            current_yaw = att.yaw_deg

    yaw_task = asyncio.create_task(yaw_listener())

    async def cleanup_tasks():
        for task in (pos_task, yaw_task):
            task.cancel()
        await asyncio.gather(pos_task, yaw_task, return_exceptions=True)

        close = getattr(drone, "close", None)
        if close is not None:
            try:
                result = close()
                if asyncio.iscoroutine(result):
                    await result
            except Exception:
                pass

        await asyncio.sleep(0.05)

    while current_pos is None or current_yaw is None:
        await asyncio.sleep(0.1)

    yaw_initial = current_yaw
    yaw_after_takeoff = yaw_initial if YAW_AFTER_TAKEOFF_DEG is None else YAW_AFTER_TAKEOFF_DEG
    print(f"Using initial yaw: {yaw_initial:.1f} deg")
    if YAW_AFTER_TAKEOFF_DEG is not None:
        print(f"Yaw after takeoff: {YAW_AFTER_TAKEOFF_DEG:.1f} deg")

    def yaw_now():
        return current_yaw if current_yaw is not None else yaw_after_takeoff

    async def send_setpoint(pos, duration_s, yaw_fn=None):
        end_time = time.time() + duration_s
        while time.time() < end_time:
            if yaw_fn is None:
                await drone.offboard.set_position_ned(pos)
            else:
                await drone.offboard.set_position_ned(
                    PositionNedYaw(pos.north_m, pos.east_m, pos.down_m, yaw_fn())
                )
            await asyncio.sleep(SETPOINT_DT)

    async def wait_until_landed(timeout_s=30.0):
        end_time = time.time() + timeout_s
        async for state in drone.telemetry.landed_state():
            if state == LandedState.ON_GROUND:
                return True
            if time.time() > end_time:
                return False
        return False

    async def smooth_move(start, target, yaw_sp, max_vel=MAX_VEL_MPS, min_duration=MIN_SEG_TIME_S, yaw_fn=None):
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
            yaw_cmd = yaw_fn() if yaw_fn is not None else yaw_sp
            await drone.offboard.set_position_ned(PositionNedYaw(north, east, down, yaw_cmd))
            await asyncio.sleep(SETPOINT_DT)

    def pos_tuple(pos):
        return (pos.north_m, pos.east_m, pos.down_m)

    # Send a few setpoints before starting offboard
    start_pos = current_pos
    for _ in range(int(1.0 / SETPOINT_DT)):
        await drone.offboard.set_position_ned(
            PositionNedYaw(start_pos.north_m, start_pos.east_m, start_pos.down_m, yaw_now())
        )
        await asyncio.sleep(SETPOINT_DT)

    print("Arming...")
    await drone.action.arm()

    print("Starting offboard...")
    try:
        await drone.offboard.start()
    except OffboardError as exc:
        print(f"Offboard start failed: {exc._result.result}")
        await drone.action.disarm()
        await cleanup_tasks()
        return

    # Takeoff to 5 m
    target_takeoff = (start_pos.north_m, start_pos.east_m, -TAKEOFF_ALT_M)
    print("Taking off to 5 m (smooth)...")
    await smooth_move(pos_tuple(current_pos), target_takeoff, yaw_after_takeoff, yaw_fn=yaw_now)

    print("Hover 10 s...")
    await send_setpoint(PositionNedYaw(*target_takeoff, yaw_after_takeoff), HOVER_TIME_S)

    # Descend near ground
    target_down = (target_takeoff[0], target_takeoff[1], -DESCEND_ALT_M)
    print("Descending (smooth)...")
    await smooth_move(pos_tuple(current_pos), target_down, yaw_after_takeoff)

    print("Landing...")
    hold_sp = PositionNedYaw(*target_down, yaw_after_takeoff)
    hold_task = asyncio.create_task(send_setpoint(hold_sp, 30.0))
    landed = False
    try:
        await drone.action.land()
        landed = await wait_until_landed(timeout_s=30.0)
    finally:
        hold_task.cancel()
        await asyncio.gather(hold_task, return_exceptions=True)
    if not landed:
        print("Landing not confirmed, stopping offboard anyway...")

    print("Stopping offboard...")
    try:
        await drone.offboard.stop()
    except OffboardError as exc:
        print(f"Offboard stop failed: {exc._result.result}")

    await cleanup_tasks()


if __name__ == "__main__":
    asyncio.run(run())
