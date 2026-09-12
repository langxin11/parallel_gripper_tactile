# dmgripper-experiments 0.1.0

DM4310P 与双侧 PapillArray 的最小纯 Python 真机实验入口。它不依赖 ROS 或 MuJoCo，预接触阶段
跟踪由速度、加速度和加加速度约束生成的 minimum-jerk 闭合量轨迹，每个控制周期通过
曲柄滑块运动学逆解为关节位置与速度；双侧 `+Fz` 稳定接触后先平滑衰减期望速度，再进入
二阶导纳力跟踪。正常结束回到机械零位并失能，异常或 Ctrl-C 直接尽力失能。

先审阅默认计划：

```sh
uv run --package dmgripper-experiments dmgripper-force-demo
```

确认传感器完全无负载、急停可用后，执行一次 0.5 N、10 s 的基本实验：

```sh
uv run --package dmgripper-experiments dmgripper-force-demo \
  --target-force 0.5 --duration 10 --bias --execute
```

若刚刚已经可靠完成触觉清零，可以省略 `--bias`；程序仍会在使能前要求双侧力位于零力窗口。
CSV 默认写入带时间戳的 `outputs/real/dm_force_demo_*.csv`，可用 `--output` 指定路径。

PapillArray 首次收到采样率配置或清零命令后可能短暂无输出。CLI 会重复尝试读取，默认最多等待
`5 s` 获得首个有效包；可用 `--tactile-startup-timeout` 放宽首次连接时限。开始控制后仍以
`0.2 s` 判断触觉数据是否过期。使用 `--bias` 时先配置采样并以首个有效包确认数据流，再由同一
采集线程发送清零命令；随后按 ROS 2 流程保持无负载等待 `2 s`。

零力验证默认要求双侧 `|Fz|≤0.1 N` 连续 `0.5 s`，可用 `--zero-force-threshold`、
`--zero-force-stable` 和 `--zero-force-timeout` 按真机残余噪声调整。若已经独立确认清零，可显式
使用 `--skip-zero-check`；程序仍会等待首个有效触觉包，并保留运行中的数据过期和超力保护。
控制和零力验证使用与 ROS 2 接触处理器一致的 `10 Hz` 一阶低通非负法向力，CSV 同时保留原始
`Fz` 与滤波后的控制力。

正常回位使用从当前闭合量到机械零位 `0 rad` 的 minimum-jerk 轨迹。默认闭合量最大速度为
`0.012 m/s`，可用 `--return-closure-velocity` 调整。回位阶段单独使用 `kp=10`、`kd=0.5`
和 `2 N·m` 合成力矩上限，以克服零位附近的静摩擦。总截止时间不会短于轨迹规划时长；轨迹结束后
继续保持零位，直到实际位置进入 `0.02 rad` 容差，默认额外允许 `2 s` 收敛，通过后才失能。

## 倒水实验

`dmgripper-cup` 提供一个独立的真机倒水交互流程。默认模式只输出配置和输出目录，不导入运行时或
访问设备；显式加入 `--execute` 才会使能真机。它使用 Tyro dataclass 默认值、严格 YAML、Tyro
命令行覆盖的顺序解析配置：

```sh
uv run --package dmgripper-experiments dmgripper-cup \
  --config configs/hardware/dmgripper/cup.yaml \
  --controller pid --control.target-force-n 0.6
```

完整操作顺序、`ready`／`release`／`status` 交互命令、输出记录和重绘方法见
[`docs/dmgripper-cup.md`](../../docs/dmgripper-cup.md)。默认数值尚未经真机验证；执行前必须准备急停和
承接容器。
