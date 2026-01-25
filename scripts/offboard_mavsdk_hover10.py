#!/usr/bin/env python3
import asyncio
import math
import time

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
from mavsdk.telemetry import LandedState


SETPOINT_HZ = 20.0
SETPOINT_DT = 1.0 / SETPOINT_HZ

TAKEOFF_ALT_M = 10.0
HOVER_TIME_S = 10.0
DESCEND_ALT_M = 0.5
YAW_AFTER_TAKEOFF_DEG = None  # Set to a number to rotate after takeoff; None keeps initial yaw.


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
    start_pos = current_pos

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

    async def goto_position(target, timeout_s=30.0, tolerance_m=0.3, yaw_fn=None):
        end_time = time.time() + timeout_s
        while time.time() < end_time:
            if yaw_fn is None:
                await drone.offboard.set_position_ned(target)
            else:
                await drone.offboard.set_position_ned(
                    PositionNedYaw(target.north_m, target.east_m, target.down_m, yaw_fn())
                )
            if current_pos is not None:
                dx = target.north_m - current_pos.north_m
                dy = target.east_m - current_pos.east_m
                dz = target.down_m - current_pos.down_m
                if math.sqrt(dx * dx + dy * dy + dz * dz) <= tolerance_m:
                    break
            await asyncio.sleep(SETPOINT_DT)

    # Send a few setpoints before starting offboard
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

    # Takeoff to 10 m
    target_takeoff = PositionNedYaw(start_pos.north_m, start_pos.east_m, -TAKEOFF_ALT_M, yaw_after_takeoff)
    print("Taking off to 10 m...")
    await goto_position(target_takeoff, timeout_s=30.0, tolerance_m=0.5, yaw_fn=yaw_now)

    print("Hover 10 s...")
    await send_setpoint(target_takeoff, HOVER_TIME_S)

    # Descend near ground
    target_down = PositionNedYaw(target_takeoff.north_m, target_takeoff.east_m, -DESCEND_ALT_M, yaw_after_takeoff)
    print("Descending...")
    await goto_position(target_down, timeout_s=30.0, tolerance_m=0.5)

    print("Landing...")
    hold_task = asyncio.create_task(send_setpoint(target_down, 30.0))
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
