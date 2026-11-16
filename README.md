# RoboBaton_4P_ROS2_demo

[中文使用文档](https://4p-docs.readthedocs.io/zh-cn/4pv2.main/usage/ros2-demo.html) · [English documentation](https://4p-doc-en.readthedocs.io/en/4pv2.main/usage/ros2-demo.html)

面向 X5 / ROS2 Humble 的 `ament_cmake` 示例包，发布 SC132 四目图像和 ICM-42670-P IMU 数据。V2 主分支为 `V2_main`；接线、部署、参数、topics、数据语义和排障说明统一维护在公开文档仓库 [4P_doc](https://github.com/Hessian-matrix/4P_doc/tree/V2_main)。

当前 IMU 使用内核 IIO 公共 ABI 6.0：实体 `libicm42670.so.6.0.0`、SONAME `libicm42670.so.6`，九个导出均为 `ICM42670_X5_6.0`。`host` 保留 `CLOCK_MONOTONIC` 主机估计，`tmst` 另提供映射至 `CLOCK_MONOTONIC_RAW` 的 sensor TMST；两者均保留原生 host 时间及全部 burst 证据。ROS2 在输出侧按所选原生时钟每 burst 显式桥接到 realtime，保留桥括号半宽。设备须由匹配内核预先绑定；提供独立 FSYNC 诊断，不提供用户态 SPI 或自动回退。全部 consumer 必须与新头/库成套重建，不做跨 major 链接。软件版本、文档修订、ABI 与发布状态独立。

## 仓库内容

- `src/`、`include/`、`lib/`：节点、compressed image_transport 插件、公开头文件与匹配的预编译库。
- `config/`、`launch/`：默认配置与启动入口。
- `script/`：交叉构建及 install 校验工具。
- 主要程序：`robobaton_sensors_node`、`robobaton_imu_rate_monitor`。

本仓库不编译底层相机/IMU 运行库。头文件、库、配置与程序须成套匹配；部署完整 install 树并保留符号链接，不与 non-ROS 运行包混用。

## 开始使用

系统镜像下载 / System image download：[4P_V2.0.0_image.tar.gz](https://www.hessian-matrix.com/wp-content/uploads/2026/automaticupdates/4P_V2.0.0_image.tar.gz)。烧录前备份数据并核对适用硬件与发布方 SHA-256；使用预装系统无需重刷，详见[系统烧录](https://4p-docs.readthedocs.io/zh-cn/4pv2.main/ops/system-flashing.html) / [System Flashing](https://4p-doc-en.readthedocs.io/en/4pv2.main/ops/system-flashing.html)。

匹配系统镜像已预装 `/root/ros2_demo/install` 和 `/opt/ros/humble`，无需重复安装；其他系统先按[源码编译](https://4p-docs.readthedocs.io/zh-cn/4pv2.main/development/open-source-build.html)准备 install 包，并按[部署、升级与回滚](https://4p-docs.readthedocs.io/zh-cn/4pv2.main/development/deployment-and-upgrade.html)完成部署。开机自启的 non-ROS 与 ROS2 不能同时采集，IMU-only 也需交接 owner。先等待录制收尾，在 X5 上正常停止 non-ROS：

```bash
cd /root/demo && ./start_sensor_demo.sh stop
```

仅在停止命令成功、旧采集进程退出后继续；超时或失败时不得启动第二实例。保持 `cam-service` 运行，再执行：

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_demo/install/robobaton_ros2_env.bash
ros2 launch robobaton_4p_ros2_demo robobaton_sensors.launch.py
```

按 `Ctrl+C` 正常停止。保持 `cam-service` 运行，不与其他相机应用并发。依赖、SHM 配置、参数和订阅方法见 [ROS2 Demo 使用](https://4p-docs.readthedocs.io/zh-cn/4pv2.main/usage/ros2-demo.html)；ROS2 路径不提供 RTSP。

默认 FastDDS profile 仅启用 SHM、禁用内置传输，发布与订阅限同一主机；不提供跨电脑 UDP 发现或数据传输。overlay hook 会覆盖两个默认 profile 环境变量，不能靠提前设置变量切换网络传输；跨机配置须另行确认与验证。

默认 YAML `ros__parameters` 顶层提供 `camera_timestamp_path: observer|official`（默认 `official`）和 `imu_timestamp_mode: host|tmst`（默认 `host`），当前明确使用相机 official + IMU host；全部四种组合保留，均使用同一内核 IIO V4 采集。也可直接运行时传 `--ros-args -p camera_timestamp_path:=official -p imu_timestamp_mode:=tmst`。参数只读，修改必须停止并重启，无热切换或失败回退；非法值即使该传感器禁用也在硬件启动前拒绝。

相机 consumer 在启动前设置 `SC132_TIMESTAMP_PATH`。默认 `official` 使用同帧 RAW IRQ `timestamps − (tv − trig_tv) × 1000` 估计（AON 微秒 delta 转 ns），不启动 SIF MMIO observer，因此其 `500µs` 调度窗口准入不再终止默认采集。逐组 RAW→realtime 使用实际测得的桥括号，即使超过 observer 的 `200µs` 投影预算也不因宽度退出，不以当前时刻替代触发；物理不确定度未知（`UINT32_MAX`），有界 timing validity `0` 不表示图像无效或已物理同步。显式 `observer` 是诊断／精度选择，保留严格 `500µs` SIF 和 `200µs` 投影准入与失败停止。IMU host 使用 MONOTONIC 主机估计，tmst 将 RAW sample 映射到输出 realtime，两者均保留原始 MONOTONIC host 与 burst 证据；TMST `200µs` 计算合同及 IMU 输出桥不变。timing JSON 为 `robobaton_imu_timing_v3`，包含模式、原生时间、`fifo_tmst`、样本不确定度及所选桥时钟。host RAW sample 为零、不确定度未知；FSYNC 为独立诊断，不校正主时间戳。真实非法元数据、时钟读取／算术、传输、所有权及设备故障仍失败，不承诺任意负载无崩溃或持续采集。本次仅本地更新，不新增最终集成组成部署、实板运行或物理精度验收。

## 文档入口

- [首次上电](https://4p-docs.readthedocs.io/zh-cn/4pv2.main/getting-started/first-boot.html) · [硬件连接与安全](https://4p-docs.readthedocs.io/zh-cn/4pv2.main/getting-started/hardware-and-safety.html)
- [数据合同](https://4p-docs.readthedocs.io/zh-cn/4pv2.main/development/data-contracts.html) · [API 参考](https://4p-docs.readthedocs.io/zh-cn/4pv2.main/development/api-reference.html)
- [故障排查](https://4p-docs.readthedocs.io/zh-cn/4pv2.main/troubleshooting.html)
- [版本、已知限制与支持](https://4p-docs.readthedocs.io/zh-cn/4pv2.main/release-and-support.html)

许可证及第三方组件说明见本仓库 [LICENSE](LICENSE) 和公开发布说明。

## FSYNC 独立诊断

ABI6 配套内核 IIO V4。`fsync_diagnostics` 只接受 `0/1`，默认 `1`；non-ROS 在 `imu.fsync_diagnostics: true|false` 或 `--imu-fsync-diagnostics on|off` 配置，ROS2 为启动时只读 bool 参数 `imu_fsync_diagnostics`。切换需正常停止并重启。独立 reader 使用 `--fsync-diagnostics on|off`，`imu_reader_demo` 使用 `--imu-fsync-diagnostics on|off`。

sample 在原布局后追加六字段：`fifo_header`、`fsync_flags`、`fsync_sequence`、`fsync_raw_ns`、`fsync_uncertainty_ns`、`fsync_delta_us`。`fifo_header & 0xfc` 等于 `0x68` 时为普通包，等于 `0x6c` 时为真实 FSYNC 标记；标记包的 `fifo_tmst` 是 FSYNC delta（µs），不是绝对 sample tick。flags 位为 `1=ENABLED`、`2=TAGGED`、`4=TIME_VALID`、`8=UNRESOLVED`；普通样本事件序号为零，真实 tag 事件序号在会话内递增。没有有效边沿时必须为 `fsync_raw_ns=0`、`fsync_uncertainty_ns=UINT32_MAX`，不得将其解释为零误差。

`fsync_raw_ns` 是独立 RAW 域边沿诊断，未用于改写 host 主时间戳或自动匹配相机帧。替换掉的绝对 tick 由实际左右端点重建，名义 ODR 只提供允许区间。host 模式的诊断映射失败会保留未解析 tag 并继续正常采集；tmst 模式仍要求有效映射并在失败时停止。bag/ROS2 的 `robobaton_imu_timing_v3` 与 MP4 `imu.csv` 保留六字段，不新增独立 FSYNC 文件。此合同不认证逐帧无长尾、LPWM 波形修复或物理同步 ≤1 ms。
