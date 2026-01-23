#!/usr/bin/env python3
import asyncio
import time

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
from mavsdk.telemetry import LandedState

SETPOINT_HZ = 20.0
TAKEOFF_ALT_M = 10.0
HOVER1_S = 10.0
MOVE_X_M = 10.0
HOVER2_S = 15.0
POSITION_TOL_M = 0.5
CONNECT_URI = "udp://:14540"


class SetpointStream:
    def __init__(self, offboard):
        self._offboard = offboard
        self._setpoint = PositionNedYaw(0.0, 0.0, 0.0, 0.0)
        self._running = False

    def set(self, sp: PositionNedYaw) -> None:
        self._setpoint = sp

    async def run(self) -> None:
        self._running = True
        period_s = 1.0 / SETPOINT_HZ

        while self._running:
            try:
                await self._offboard.set_position_ned(self._setpoint)
            except OffboardError as err:
                print(f"[offboard] setpoint error: {err._result.result}")
            await asyncio.sleep(period_s)

    def stop(self) -> None:
        self._running = False


async def wait_for_connection(system: System) -> None:
    async for state in system.core.connection_state():
        if state.is_connected:
            print("[offboard] connected")
            return


async def wait_for_health(system: System) -> None:
    async for health in system.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("[offboard] health ok")
            return


async def position_listener(system: System, out: dict) -> None:
    async for pv in system.telemetry.position_velocity_ned():
        out["pos"] = pv.position


async def wait_until_position(out: dict, north: float, east: float, down: float, timeout_s: float) -> bool:
    start = time.monotonic()
    while time.monotonic() - start < timeout_s:
        pos = out.get("pos")
        if pos:
            if (abs(pos.north_m - north) <= POSITION_TOL_M and
                    abs(pos.east_m - east) <= POSITION_TOL_M and
                    abs(pos.down_m - down) <= POSITION_TOL_M):
                return True
        await asyncio.sleep(0.1)
    return False


async def wait_for_landed(system: System, timeout_s: float) -> bool:
    start = time.monotonic()
    async for state in system.telemetry.landed_state():
        if state == LandedState.ON_GROUND:
            return True
        if time.monotonic() - start > timeout_s:
            return False


async def main() -> None:
    system = System()
    await system.connect(system_address=CONNECT_URI)

    await wait_for_connection(system)
    await wait_for_health(system)

    pos_state = {"pos": None}
    pos_task = asyncio.create_task(position_listener(system, pos_state))

    offboard = system.offboard
    stream = SetpointStream(offboard)
    stream_task = asyncio.create_task(stream.run())

    # Send a few setpoints before starting offboard
    stream.set(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(1.0)

    await system.action.arm()

    try:
        await offboard.start()
        print("[offboard] started")
    except OffboardError as err:
        print(f"[offboard] start failed: {err._result.result}")
        await system.action.disarm()
        stream.stop()
        await stream_task
        pos_task.cancel()
        return

    # Takeoff to 10 m
    stream.set(PositionNedYaw(0.0, 0.0, -TAKEOFF_ALT_M, 0.0))
    await wait_until_position(pos_state, 0.0, 0.0, -TAKEOFF_ALT_M, timeout_s=20.0)
    await asyncio.sleep(HOVER1_S)

    # Fly +X (north) 10 m
    stream.set(PositionNedYaw(MOVE_X_M, 0.0, -TAKEOFF_ALT_M, 0.0))
    await wait_until_position(pos_state, MOVE_X_M, 0.0, -TAKEOFF_ALT_M, timeout_s=20.0)
    await asyncio.sleep(HOVER2_S)

    # Descend and land
    stream.set(PositionNedYaw(MOVE_X_M, 0.0, -0.2, 0.0))
    await wait_until_position(pos_state, MOVE_X_M, 0.0, -0.2, timeout_s=20.0)

    await offboard.stop()
    await system.action.land()
    await wait_for_landed(system, timeout_s=20.0)
    await system.action.disarm()

    stream.stop()
    await stream_task
    pos_task.cancel()


if __name__ == "__main__":
    asyncio.run(main())
