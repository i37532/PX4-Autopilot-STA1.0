# PX4 Drone Autopilot

[![Releases](https://img.shields.io/github/release/PX4/PX4-Autopilot.svg)](https://github.com/PX4/PX4-Autopilot/releases) [![DOI](https://zenodo.org/badge/22634/PX4/PX4-Autopilot.svg)](https://zenodo.org/badge/latestdoi/22634/PX4/PX4-Autopilot)

[![Build Targets](https://github.com/PX4/PX4-Autopilot/actions/workflows/build_all_targets.yml/badge.svg?branch=main)](https://github.com/PX4/PX4-Autopilot/actions/workflows/build_all_targets.yml) [![SITL Tests](https://github.com/PX4/PX4-Autopilot/workflows/SITL%20Tests/badge.svg?branch=master)](https://github.com/PX4/PX4-Autopilot/actions?query=workflow%3A%22SITL+Tests%22)

[![Discord Shield](https://discordapp.com/api/guilds/1022170275984457759/widget.png?style=shield)](https://discord.gg/dronecode)

This repository holds the [PX4](http://px4.io) flight control solution for drones, with the main applications located in the [src/modules](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules) directory. It also contains the PX4 Drone Middleware Platform, which provides drivers and middleware to run drones.

PX4 is highly portable, OS-independent and supports Linux, NuttX and MacOS out of the box.

* Official Website: http://px4.io (License: BSD 3-clause, [LICENSE](https://github.com/PX4/PX4-Autopilot/blob/main/LICENSE))
* [Supported airframes](https://docs.px4.io/main/en/airframes/airframe_reference.html) ([portfolio](https://px4.io/ecosystem/commercial-systems/)):
  * [Multicopters](https://docs.px4.io/main/en/frames_multicopter/)
  * [Fixed wing](https://docs.px4.io/main/en/frames_plane/)
  * [VTOL](https://docs.px4.io/main/en/frames_vtol/)
  * [Autogyro](https://docs.px4.io/main/en/frames_autogyro/)
  * [Rover](https://docs.px4.io/main/en/frames_rover/)
  * many more experimental types (Blimps, Boats, Submarines, High Altitude Balloons, Spacecraft, etc)
* Releases: [Downloads](https://github.com/PX4/PX4-Autopilot/releases)

## Releases

Release notes and supporting information for PX4 releases can be found on the [Developer Guide](https://docs.px4.io/main/en/releases/).

## Building a PX4 based drone, rover, boat or robot

The [PX4 User Guide](https://docs.px4.io/main/en/) explains how to assemble [supported vehicles](https://docs.px4.io/main/en/airframes/airframe_reference.html) and fly drones with PX4. See the [forum and chat](https://docs.px4.io/main/en/#getting-help) if you need help!


## Changing Code and Contributing

This [Developer Guide](https://docs.px4.io/main/en/development/development.html) is for software developers who want to modify the flight stack and middleware (e.g. to add new flight modes), hardware integrators who want to support new flight controller boards and peripherals, and anyone who wants to get PX4 working on a new (unsupported) airframe/vehicle.

Developers should read the [Guide for Contributions](https://docs.px4.io/main/en/contribute/).
See the [forum and chat](https://docs.px4.io/main/en/#getting-help) if you need help!


## Weekly Dev Call

The PX4 Dev Team syncs up on a [weekly dev call](https://docs.px4.io/main/en/contribute/).

> **Note** The dev call is open to all interested developers (not just the core dev team). This is a great opportunity to meet the team and contribute to the ongoing development of the platform. It includes a QA session for newcomers. All regular calls are listed in the [Dronecode calendar](https://www.dronecode.org/calendar/).


## Maintenance Team

See the latest list of maintainers on [MAINTAINERS](MAINTAINERS.md) file at the root of the project.

For the latest stats on contributors please see the latest stats for the Dronecode ecosystem in our project dashboard under [LFX Insights](https://insights.lfx.linuxfoundation.org/foundation/dronecode). For information on how to update your profile and affiliations please see the following support link on how to [Complete Your LFX Profile](https://docs.linuxfoundation.org/lfx/my-profile/complete-your-lfx-profile). Dronecode publishes a yearly snapshot of contributions and achievements on its [website under the Reports section](https://dronecode.org).

## Supported Hardware

For the most up to date information, please visit [PX4 User Guide > Autopilot Hardware](https://docs.px4.io/main/en/flight_controller/).

## Project Governance

The PX4 Autopilot project including all of its trademarks is hosted under [Dronecode](https://www.dronecode.org/), part of the Linux Foundation.

<a href="https://www.dronecode.org/" style="padding:20px" ><img src="https://dronecode.org/wp-content/uploads/sites/24/2020/08/dronecode_logo_default-1.png" alt="Dronecode Logo" width="110px"/></a>
<div style="padding:10px">&nbsp;</div>

## ISTA 速度环改动说明

### 修改/新增的文件

| 文件 | 操作 |
| --- | --- |
| [src/modules/mc_pos_control/PositionControl/IstaController.hpp](src/modules/mc_pos_control/PositionControl/IstaController.hpp) | **新建** - ISTA 控制器头文件 |
| [src/modules/mc_pos_control/PositionControl/IstaController.cpp](src/modules/mc_pos_control/PositionControl/IstaController.cpp) | **新建** - BBSTA 三分支算法实现 |
| [src/modules/mc_pos_control/mc_pos_control_ista_params.c](src/modules/mc_pos_control/mc_pos_control_ista_params.c) | **新建** - ISTA 参数定义 |
| [src/modules/mc_pos_control/PositionControl/PositionControl.hpp](src/modules/mc_pos_control/PositionControl/PositionControl.hpp) | **修改** - 添加 ISTA 接口和成员 |
| [src/modules/mc_pos_control/PositionControl/PositionControl.cpp](src/modules/mc_pos_control/PositionControl/PositionControl.cpp) | **修改** - velocityControl 分支 + reset + 限幅/抗饱和/悬停处理 |
| [src/modules/mc_pos_control/MulticopterPositionControl.hpp](src/modules/mc_pos_control/MulticopterPositionControl.hpp) | **修改** - 参数声明 |
| [src/modules/mc_pos_control/MulticopterPositionControl.cpp](src/modules/mc_pos_control/MulticopterPositionControl.cpp) | **修改** - 参数传递 + 日志 |
| [src/modules/mc_pos_control/PositionControl/CMakeLists.txt](src/modules/mc_pos_control/PositionControl/CMakeLists.txt) | **修改** - 添加 IstaController 源文件 |

### ISTA 参数

| 参数 | 默认值 | 描述 |
| --- | --- | --- |
| `MPC_VEL_ISTA_EN` | 0 | 启用开关 (0=PID, 1=ISTA) |
| `MPC_ISTA_L1_XY` | 1.0 | XY 轴 λ1 |
| `MPC_ISTA_L2_XY` | 0.5 | XY 轴 λ2 |
| `MPC_ISTA_L1_Z` | 1.2 | Z 轴 λ1 |
| `MPC_ISTA_L2_Z` | 0.8 | Z 轴 λ2 |
| `MPC_ISTA_KEEP_D` | 1 | 保留 D-term |
| `MPC_ISTA_HOV_DB` | 0.05 | 悬停速度死区 (m/s) |
| `MPC_ISTA_HOV_TC` | 0.5 | 悬停时 nu 衰减时间常数 (s) |
| `MPC_ISTA_EPS` | 0.05 | 边界层宽度 (m/s, 0=关闭平滑) |

### 参数调参建议

- **基础流程**：先 `MPC_VEL_ISTA_EN=1`，其余保持默认，确认能稳定起飞/悬停。
- **抖动/左右摆**：优先减小 `MPC_ISTA_L1_XY`、`MPC_ISTA_L2_XY`（例如各减半），保持 `MPC_ISTA_KEEP_D=1`。
- **轻微漂移**：增大 `MPC_ISTA_HOV_DB`（例如 0.05→0.08）或减小 `MPC_ISTA_HOV_TC`（例如 0.5→0.3）。
- **高频抖动/啸叫**：小幅增大 `MPC_ISTA_EPS`（例如 0.005→0.02），过大会引入迟滞和偏移。
- **Z 轴跟随慢/抖**：小幅调整 `MPC_ISTA_L1_Z`、`MPC_ISTA_L2_Z`，幅度不宜过大。
