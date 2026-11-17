# Piper 左抛右接 demo：运控测试脚本

左臂 = `can1`，右臂 = `can3`（来自 `piper.json` 的序列号映射）。FK 用 `piper_identified.urdf`，已被动验证与固件末端位姿一致（法兰 0.09 mm / 0°），TCP = 法兰沿 z 轴 +0.13503 m。

| 脚本 | 作用 | 是否发指令 |
|---|---|---|
| `t0_can_jitter.py` | CAN 反馈频率 / 抖动 / 丢帧 | 否（只收） |
| `t1_gripper_latency.py` | 夹爪 指令→动作 延迟、10/50/90% 时间、稳定时间 | 只动夹爪，臂不动 |
| `t2_tip_speed.py` | 臂末端最大速度/加速度，并估算"在峰值速度出手"的弹道射程 | 动臂（MOVE J，位置模式） |

`t1`、`t2` 默认是 **dry-run**（只打印计划和 FK 检查），必须加 `--execute` 才会发指令，并且会再要求输入 `YES`。

## 部署与运行

在远程新建自己的目录（不要动别人的目录/环境），用已有的 piper-workspace 环境只读使用：

```bash
scp -r piper_catch <user>@<host>:~/jeff_piper_catch
```

```bash
ssh -t <user>@<host> 'cd ~/jeff_piper_catch && <python-with-piper_sdk> t0_can_jitter.py'
```

```bash
ssh -t <user>@<host> 'cd ~/jeff_piper_catch && <python-with-piper_sdk> t1_gripper_latency.py --arm left --reps 20'
```

确认计划后加 `--execute`。数据写在 `data/`，拷回本地分析：

```bash
scp -r <user>@<host>:~/jeff_piper_catch/data ./data_remote
```

## 运行前必须确认

1. **没有别人在控制同一只臂**：同机上另一个控制台后端（例如监听 8062 的服务）可能占着 can1/can3。脚本发指令前会被动监听 1 秒，发现别的进程在发控制帧（0x150–0x15F、0x470–0x47F）就直接退出，但这只是兜底，请先当面确认。
2. 急停在手边；`t2` 前清空左臂周围空间。
3. 先 dry-run，看 TCP 路径和最低高度。

## 测试顺序与判读

1. **t0**：反馈应约 200 Hz，P99 间隔和最大间隔别远大于 5 ms；有丢帧先查线/USB。
2. **t1（空夹爪）**：看 `latency` 的均值、**std 和 P95**。出手位置误差约等于 末端速度 × 延迟抖动，所以抖动比均值更重要。再用实际瓶子/杯子重复（`--with-object bottle-65mm --close-mm` 小于物体宽度，让夹爪夹在物体上）。
3. **t2**：`--rates` 从 10% 逐步升到 100%；任一档未到位、关节速度超 `--vmax`（默认 3 rad/s）或臂状态异常就停止爬升。看 `peak_tip_speed` 随 rate 是否饱和，那就是固件限速。

## 已知限制

- 反馈约 200 Hz，时间分辨率约 ±5 ms；要更细的出手时刻需要外部高速相机。
- `t2` 用 MOVE J（固件内插）。MIT 模式能更快更准，留到第二阶段，需要重力补偿和增益调参。
- `t2` 默认只动 J2/J3（臂所在的竖直面内），J1 固定。按 `piper.json` 两臂并排、前向相同，所以这个面内的摆动**不会朝向对方臂**，可以安全地先测末端速度。真正的"左抛右接"要用 J1 向右臂方向甩，最大速度和碰撞风险都不同，需要单独加一个 J1 测试，别直接用 `--dq-deg 30 0 0 0 0 0` 之类的参数去试。
- 夹爪反馈单位按 0.001 mm 解释，尚未和实物对照；先用 t1 的 travel 值核对（张开 60 mm 应该读到约 60）。
- 本地只用合成数据验证了分析代码（夹爪时序和 FK 速度计算），**没有在真机上运行过任何发指令的脚本**。
