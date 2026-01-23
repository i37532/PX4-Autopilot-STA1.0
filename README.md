

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

## ISTA 角速率环改动说明

### 修改/新增的文件

| 文件 | 操作 |
| --- | --- |
| [src/lib/rate_control/rate_control.hpp](src/lib/rate_control/rate_control.hpp) | **修改** - 增加 ISTA 运行分支与参数接口 |
| [src/lib/rate_control/rate_control.cpp](src/lib/rate_control/rate_control.cpp) | **修改** - ISTA 控制律 + 平滑/死区/nu 衰减 |
| [src/modules/mc_rate_control/mc_rate_control_params.c](src/modules/mc_rate_control/mc_rate_control_params.c) | **修改** - 新增 ISTA 参数定义 |
| [src/modules/mc_rate_control/MulticopterRateControl.hpp](src/modules/mc_rate_control/MulticopterRateControl.hpp) | **修改** - 参数声明 |
| [src/modules/mc_rate_control/MulticopterRateControl.cpp](src/modules/mc_rate_control/MulticopterRateControl.cpp) | **修改** - 参数传递到 RateControl |

### ISTA 参数

| 参数 | 默认值 | 描述 |
| --- | --- | --- |
| `MC_RATE_ISTA_EN` | 0 | 启用开关 (0=PID, 1=ISTA) |
| `MC_RATE_ISTA_EPS` | 0.02 | 边界层宽度 (rad/s, 0=关闭平滑) |
| `MC_RATE_ISTA_DB` | 0.02 | 角速率死区 (rad/s) |
| `MC_RATE_ISTA_TC` | 0.5 | nu 衰减时间常数 (s) |

### 参数调参建议

- **基础流程**：先 `MC_RATE_ISTA_EN=1`，保持 `MC_ROLLRATE_I/MC_PITCHRATE_I=0`，确认悬停无抖动。
- **抖动明显**：先增大 `MC_RATE_ISTA_EPS`（例如 0.08→0.12），再增大 `MC_RATE_ISTA_DB`（例如 0.03→0.05）。
- **残余小振荡**：减小 `MC_RATE_ISTA_TC`（例如 0.5→0.3）加快 nu 衰减。
- **需要抗扰**：在稳定基础上小步增 `MC_ROLLRATE_I/MC_PITCHRATE_I`（例如 0.005→0.02），抖动或漂移出现则回退。



## Docker + Gazebo Classic 快速启动

### 启动 Docker 环境（带 GUI）

```sh
xhost +local:docker
docker run -it --rm --privileged --network host \
  --env=LOCAL_USER_ID="$(id -u)" \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v /home/i37/PX4-Autopilot-STA1.0:/src/PX4-Autopilot:rw \
  px4io/px4-dev-simulation-focal:2022-08-12 bash
```

### 进入容器后编译并启动 Gazebo Classic

```sh
cd /src/PX4-Autopilot
git submodule update --init --recursive

# 使用独立 build 目录避免 CMakeCache 路径冲突
export PX4_BUILD_DIR=/src/PX4-Autopilot/build-docker
make px4_sitl_default gazebo-classic
```

### 修改源码后如何重新编译

源码在宿主机修改后，容器内会自动同步（挂载目录），直接重新编译即可：

```sh
cd /src/PX4-Autopilot
export PX4_BUILD_DIR=/src/PX4-Autopilot/build-docker
make px4_sitl_default gazebo-classic
```



## 编译固件

```
docker run -it --rm --privileged --network host \
  -v /home/i37/PX4-Autopilot-STA1.0:/src/PX4-Autopilot:rw \
  px4io/px4-dev-nuttx-focal:2022-08-12 bash

cd /src/PX4-Autopilot
export PX4_BUILD_DIR=/src/PX4-Autopilot/build-nuttx-docker
make px4_fmu-v5_default
```

路径：

```
build/px4_fmu-v5_default/px4_fmu-v5_default.px4
```

