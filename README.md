

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
- **Z 轴跟随慢/抖**：小幅调整 `MPC_ISTA_L1_Z`、`MPC_ISTA_L2_Z`，幅度不宜过大。\\
---
- 漂移：低频/直流偏置，位置单向慢慢走，速度均值不为 0（不怎么过零），随时间累计偏移。
- 抖动/左右摆：低频周期性摆动（典型 0.3–2 Hz），位置/速度围绕 0 往返，频繁过零，均值接近 0。
- 高频抖动/啸叫：高频小幅抖（>5–10 Hz），位置变化不大但速度/加速度/推力快速抖动，肉眼像“震”。
