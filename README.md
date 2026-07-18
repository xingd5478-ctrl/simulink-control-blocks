# Simulink Control Blocks — 控制工程积木库

[![MATLAB](https://img.shields.io/badge/MATLAB-R2020a+-0076A8?style=flat-square&logo=matlab)](https://www.mathworks.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green?style=flat-square)](LICENSE)
[![Blocks](https://img.shields.io/badge/Blocks-12-blue?style=flat-square)](blocks/)

12 个可直接拖进 Simulink 模型使用的积木模块。每个都是 **Masked Subsystem**——双击弹出参数对话框，右键 Look Under Mask 看内部实现。

## 快速开始

```matlab
>> build_all_blocks       % 一键生成全部 12 个 .slx 文件
```

然后打开任意一个 `blocks/` 下的 .slx 文件，把里面的积木方块拖到你的模型里用。

## 积木列表

### 控制器 (4)

| 积木 | 用法 | 参数 |
|------|------|------|
| **PID** | 输入误差 e，输出控制量 u | Kp, Ki, Kd, N(滤波), 上下限, Kaw(抗饱和) |
| **LQR** | 输入状态 x + 参考 r，输出 u=-Kx+Ki∫e | K_vec, Ki, 上下限 |
| **SMC** | 输入误差 e + 导数 de，输出 u=-K·sat(s/φ) | λ(滑模面斜率), K, φ(边界层) |
| **LeadLag** | 输入信号，输出补偿后信号 C(s)=K(αTs+1)/(Ts+1) | K, T, α(>1超前, <1滞后) |

### 观测器 (2)

| 积木 | 用法 | 参数 |
|------|------|------|
| **Luenberger** | 输入 [u; y]，输出估计状态 x̂ | A, B_aug=[B L], C, D, x0 |
| **Kalman** | 输入 [u; y]，输出估计状态 x̂ | A, B_aug=[B K_kf], C, D, x0 |

### 滤波器 (3)

| 积木 | 用法 | 参数 |
|------|------|------|
| **LowPass** | 输入含噪信号，输出滤波后信号 | ωc (截止频率 rad/s) |
| **Notch** | 输入信号，输出陷波后信号 | ωn, ζ1(深度), ζ2(宽度) |
| **Complementary** | 输入高频+低频信号，输出融合信号 | α (0~1, 典型0.98) |

### 工具 (3)

| 积木 | 用法 | 参数 |
|------|------|------|
| **Rate Limiter** | 限制信号变化速率 | 上升速率, 下降速率 |
| **Anti-Windup** | 输入 u_raw + u_sat，输出回差反馈 | Kaw |
| **DynSat** | 可变上下限饱和，输入 [信号, UL, LL] | 无（通过端口设定） |

## 使用示例

把积木拖到模型里，连线，设参数，运行：

```matlab
% 例：PID 控制 MSD 系统
load_system('blocks/controllers/PID_Controller.slx');
add_block('PID_Controller/PID', 'mymodel/PID');
set_param('mymodel/PID', 'Kp', '50', 'Ki', '20', 'Kd', '5');
```

更多例子见 `examples/demo_pid_msd.m` 和 `test_all_blocks.m`。

## 项目结构

```
├── build_all_blocks.m        # 一键生成全部积木
├── test_all_blocks.m         # 12 个积木的回路测试
├── blocks/                   # .slx 积木文件
│   ├── controllers/          # PID, LQR, SMC, LeadLag
│   ├── observers/            # Luenberger, Kalman
│   ├── filters/              # LowPass, Notch, Complementary
│   └── utilities/            # RateLimiter, AntiWindup, DynSat
├── examples/                 # 使用示例
│   └── demo_pid_msd.m       # PID 积木控制 MSD 系统
├── LICENSE
└── README.md
```

## 配套教程

每个积木的设计原理和理论基础见 **[Simulink 控制工程教程 (30 课)](https://github.com/xingd5478-ctrl/simulink-control-tutorial)**。积木库是「工具箱」，教程是「说明书」——两个项目互补。

## License

MIT — 可自由使用、修改、分发。
