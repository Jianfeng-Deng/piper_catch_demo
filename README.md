# piper_catch_demo：双臂 Piper 抛接 demo

用两台 AgileX **PiPER** 机械臂做"左臂抛、右臂接"的 demo：先把运动控制（尤其是 **MIT 模式**）测清楚，再规划抛接轨迹，最后上真机。
这个仓库是**当前的工作快照**：运动测试脚本、一个独立的监视/遥操/规划前端和后端，以及项目现状文档。

> **安全提示**：这些代码会让真实机械臂高速运动。所有会发指令的路径默认关闭，需要显式解锁，并且都带停止条件；
> 但它们**没有在你的机器上验证过**。请先读 [docs/PROJECT_STATUS.md](docs/PROJECT_STATUS.md) 里的"安全约定"，并确保物理急停可达。

## 目录

| 路径 | 内容 |
|---|---|
| `common.py` | 公共工具：按 USB 序列号找 CAN 口、URDF 正运动学、慢速 MOVE J、归拢（stow）、反馈录制 |
| `t0_can_jitter.py` / `t1_gripper_latency.py` / `t2_tip_speed.py` / `t3_camera_rate.py` | 早期运控测试：CAN 抖动、夹爪延迟、MOVE J 末端速度、RealSense 帧率 |
| `stow.py` / `save_stow_pose.py` / `hold.py` | 归拢并失能、保存归拢位姿、终端版"急停保持" |
| `demo/backend/` | FastAPI 后端：只读遥测、MIT 逐步测试、主从遥操、移动到位姿、离线抛接规划 |
| `demo/frontend/` | 前端：URDF 预览、MIT 实时数据与曲线、手动位姿预览、上抛/斜抛预览、遥操与测试面板 |
| `demo/tests/` | 不需要真机的单元测试（用模拟臂验证状态机、停止条件、规划器） |
| `docs/PROJECT_STATUS.md` | **项目现状**：硬件、实测结果、已知问题、下一步 |
| `docs/motion_tests.md` | 早期运控测试的使用说明 |

## 环境

- Python 3.10+，`numpy`、`python-can`、`piper_sdk`（**MIT 需要 V3 接口 `C_PiperInterface_V3`，固件 ≥ S-V1.8-8**）
- 后端：`fastapi`、`uvicorn`、`pydantic`；逆动力学和速度测试还需要 `mujoco`
- 前端：浏览器直接打开，三维预览通过 CDN 加载 three.js 与 urdf-loader
- 一台装有 SocketCAN 的 Linux 主机，每条臂一个 USB-CAN 适配器

## 需要你自己提供的文件

1. `piper_identified.urdf`（放在仓库根目录）：带辨识惯量参数的 PiPER URDF，用于重力补偿和逆动力学。
   它来自另一个控制台工程，**没有放进本仓库**。我们使用的那份文件的 sha256 是
   `10ae37c55b4f97dae9a01b95eef0c2602008cc82df0c239dda803ee979643253`（`/health` 接口也会返回它），可用来核对你拿到的是不是同一份。
2. 三维预览用的 URDF 与网格（`piper_description_v100_adapter.urdf` + `meshes/`）和 `piper.json`（底座位姿、MIT 增益）：
   通过环境变量指向你自己的工程：`PIPER_WORKSPACE`（默认 `~/piper-workspace`）、`PIPER_MODEL_DIR`、`PIPER_PROFILE`。
3. USB-CAN 适配器序列号：在 `demo/backend/arms.py`（`ARMS`）和 `common.py`（`ARM_SERIAL`）里改成你自己的。
   **CAN 口名（can0…）会在重启后重新排列，所以代码按序列号解析，不要改成按口名。**

## 运行

```bash
# 单元测试（不需要真机；需要 mujoco 的用例在没有 mujoco 时自动跳过）
cd demo && python -m unittest discover -s tests

# 只读模式启动（不会发送任何 CAN 帧）
cd demo && DEMO_ALLOW_CONTROL=0 python -m uvicorn backend.app:app --host 127.0.0.1 --port 8071
# 浏览器打开 http://127.0.0.1:8071 （服务只监听本机回环，远程主机请用 SSH 端口转发）

# 解锁控制（会让机械臂运动：只在有人现场、急停可达时使用）
cd demo && DEMO_ALLOW_CONTROL=1 python -m uvicorn backend.app:app --host 127.0.0.1 --port 8071
```

更多细节见 [demo/README.md](demo/README.md)。

## 许可

尚未指定许可证。使用前请自行确认依赖项（PiPER SDK、URDF/网格）各自的许可。
