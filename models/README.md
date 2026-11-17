# 模型文件来源与许可

| 文件 | 来源 | 许可 / 说明 |
|---|---|---|
| `models/piper/piper_description_v100_adapter.urdf`、`models/piper/meshes/*.STL`、`models/piper/LICENSE`、`models/piper/v100-provenance.json` | AgileX 官方 [piper_ros](https://github.com/agilexrobotics/piper_ros)（提交 `ac41fcb…`）的 PiPER 模型，改成了与 SDK 一致的六关节加法兰约定，并带 100 mm 夹爪（v100）；改动细节见 `v100-provenance.json` | MIT（Copyright (c) 2024 RosenYin，见 `models/piper/LICENSE`）。网格由官方 DAE 转成 STL，只保留三维预览用到的 10 个 |
| `piper_identified.urdf` | 来自另一位同事的控制台工程（`configs/mit/piper_identified.urdf`），带**辨识出的惯量参数**，用于重力补偿和逆动力学。sha256 `10ae37c55b4f97dae9a01b95eef0c2602008cc82df0c239dda803ee979643253` | **许可未明确**。应作者的同意再公开/再分发；如果仓库公开，请先确认对方允许 |

TCP 取法兰沿 z 轴 0.13503 m，是名义值，**没有做接触标定**。
