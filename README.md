# RoboBaton_4P_ROS2_demo

RoboBaton 4P V2 的 ROS2 `ament_cmake` 示例包，发布 SC132 四目 NV12 图像和 ICM-42670-P IMU 数据。

> **最终用户说明以 [4P_doc](https://4p-docs.readthedocs.io/zh-cn/latest/index.html) 为准。** 本 README 只保留 ROS2 仓库入口、最小构建/运行方式和支持边界。

当前 IMU 使用内核 IIO 公共 ABI 5.0：实体 `libicm42670.so.5.0.0`、SONAME `libicm42670.so.5`，九个导出均为 `ICM42670_X5_5.0`。`host` 保留 `CLOCK_MONOTONIC` 主机估计，`tmst` 另提供映射至 `CLOCK_MONOTONIC_RAW` 的 sensor TMST；两者均保留原生 host 时间及全部 burst 证据。ROS2 在输出侧按所选原生时钟每 burst 显式桥接到 realtime，保留桥括号半宽。设备须由匹配内核预先绑定；不提供用户态 SPI、FSYNC 或自动回退。全部 consumer 必须与新头/库成套重建，不做跨 major 链接。软件版本、文档修订、ABI 与发布状态独立。

## 目标产物

- 节点：`robobaton_sensors_node`
- IMU 频率检查工具：`robobaton_imu_rate_monitor`
- Launch：`launch/robobaton_sensors.launch.py`
- 默认配置：`config/robobaton_sensors.yaml`
- 板端 install 目录：`/root/ros2_demo/install`

## Topics 摘要

```text
/robobaton/cam0..cam3/image_raw
/robobaton/cam0..cam3/image_raw/compressed
/robobaton/cam0..cam3/camera_info
/robobaton/cam0..cam3/exposure
/robobaton/imu/data
/robobaton/imu/temperature
/robobaton/imu/timing           # 每样本原生时钟及输出桥 JSON
```

完整 topic、消息类型、QoS、相机映射、时间戳和数据语义见 [ROS2 Demo 使用](https://4p-docs.readthedocs.io/zh-cn/latest/usage/ros2-demo.html) 和 [数据合同](https://4p-docs.readthedocs.io/zh-cn/latest/development/data-contracts.html)。

`/robobaton/camN/exposure` 使用 `std_msgs/msg/String` 发布 `robobaton_exposure_v1` JSON，按相机和 `timestamp_ns` 对照 Image 的触发沿 stamp。`exposure.status=1` 是随帧冻结的最新 ISP 估计值，不证明该帧实际生效曝光；`0` 表示不可用。它不改变图像时间戳，也不提供真实曝光中点。

曝光 topic 为 Reliable KeepLast(8)，按相机及 Image stamp 对照，不把 `source_timestamp` 当 realtime。IMU timing 为 Reliable KeepLast(256)，`robobaton_imu_timing_v2` 保留 sample MONOTONIC、burst RAW、generation/sequence/evidence、来源及 bridge 基点/半宽。该半宽只约束观测，不是物理精度；JSON 整数须无损解析，发布成功不等于 DDS 全部收到。

上述为默认触发沿语义。显式 `SC132_TEST_EXPOSURE_MIDPOINT=1` 测试模式会把图像、CameraInfo 和曝光 topic 的 stamp 改为触发时间加半曝光估计；不改变 IMU，不是已校准曝光中心，曝光不可用时停止。仅重建了支持开关的库后可用，详见[数据合同](https://4p-docs.readthedocs.io/zh-cn/latest/development/data-contracts.html#test-exposure-midpoint)。

## 版本查询

安装后执行：

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_demo/install/robobaton_ros2_env.bash

/root/ros2_demo/install/lib/robobaton_4p_ros2_demo/robobaton_sensors_node --version
/root/ros2_demo/install/lib/robobaton_4p_ros2_demo/robobaton_imu_rate_monitor --version
```

版本查询不会初始化 ROS graph、相机或 IMU。产品版本、ABI 和发布组合见 [产品版本与兼容性](https://4p-docs.readthedocs.io/zh-cn/latest/getting-started/product-and-compatibility.html)、[API 参考](https://4p-docs.readthedocs.io/zh-cn/latest/development/api-reference.html) 和 [版本更新记录](https://4p-docs.readthedocs.io/zh-cn/latest/changelog.html)。

## 构建

构建前需要：

- 本仓库 `./lib/libicm42670.so` 和 `./lib/libsc132.so`；
- `x5_4cam_cross_toolchain_20261009` 工具包，包含目标侧 ROS2（含 image_transport/pluginlib）、媒体头库和固定的 Arm GNU AArch64 编译器；
- Ubuntu 22.04 x86_64 主机、ROS Humble 和 `python3-colcon-common-extensions`；按下方源码编译页配置官方软件源并安装，系统 Python 必须能导入 ament。

本仓库只构建 ROS2 consumer 和 compressed image_transport plugin，不从父仓库读取 producer 源码，也不在这里重新构建 producer 库。

`lib/` 与 `install/lib/robobaton_4p_ros2_demo/` 中的 ICM/SC132 使用一份 `libNAME.so.M.m.p` 实体、`libNAME.so.M -> libNAME.so.M.m.p`、`libNAME.so -> libNAME.so.M` 两个同目录相对链接。CMake 和 install verifier 拒绝展开为普通文件的别名及错误链接；ROS2 不包含 PRRTSP。完整交付请用保留链接的 `tar`、`cp -a` 或 `rsync -a`；`abi_manifest.sha256` 的解析后字节校验不能单独证明链接元数据正确。

```bash
export X5_TOOLCHAIN_ROOT="/path/to/x5_4cam_cross_toolchain_20261009"
source /opt/ros/humble/setup.bash
/usr/bin/python3 -c 'import ament_package, catkin_pkg, em, colcon_core'
source "$X5_TOOLCHAIN_ROOT/setup_4cam_cross_env.sh"
CMAKE_BUILD_PARALLEL_LEVEL=4 script/build_x5_ros2.sh \
  --cross-root "$X5_CROSS_ROOT" --parallel-workers 1
python3 script/verify_install.py 1.ros2_build/install
```

`X5_CROSS_ROOT` 指向工具包内 `cross_compile/new`。主机只加载主机 ROS setup；不要 source 包内 AArch64 ROS setup。切换工具包后使用新的 `--build-root`；显式 `--toolchain-file` 优先于交叉环境脚本。

脚本将产物放到本仓库的 `1.ros2_build/{build,install,log}`，默认生成 merged install。完整依赖、构建参数、install verifier 和发布包规则见 [公开 Demo 源码编译](https://4p-docs.readthedocs.io/zh-cn/latest/development/open-source-build.html)。

## 部署与运行

部署到 `/root/ros2_demo/install` 前，必须使用完整 archive checksum、解包后的 runtime `abi_manifest.sha256`、旧应用退出检查、旧目录备份、原子切换和 smoke 验证。失败时恢复最近备份。完整命令见 [部署、升级与回滚](https://4p-docs.readthedocs.io/zh-cn/latest/development/deployment-and-upgrade.html)。

启动默认四路相机和 IMU：

```bash
source /root/ros2_demo/install/robobaton_ros2_env.bash
ros2 launch robobaton_4p_ros2_demo robobaton_sensors.launch.py
```

默认 YAML `ros__parameters` 顶层提供 `camera_timestamp_path: observer|official`（默认 `observer`）和 `imu_timestamp_mode: host|tmst`（默认 `host`），四种组合共用同一内核 IIO 采集。也可直接运行时传 `--ros-args -p camera_timestamp_path:=official -p imu_timestamp_mode:=tmst`。参数只读，修改必须停止并重启；非法值即使该传感器禁用也在硬件启动前拒绝。

相机 consumer 在启动前设置 `SC132_TIMESTAMP_PATH`。`observer` 使用原有有界 SIF MMIO observer；`official` 使用同帧 `timestamps - (tv - trig_tv)` 估计，不启动 observer，物理不确定度未知（`UINT32_MAX`），不授予有界 timing validity。两路径都输出 realtime，无失败回退。TMST 模式将 RAW sample 映射到输出 realtime，保留原始 MONOTONIC host 与 burst 证据；timing JSON 增加 `timestamp_mode`、`fifo_tmst`、`sample_raw_ns`、`timestamp_uncertainty_ns` 及正确桥时钟标签。host RAW sample 为零、不确定度未知；FSYNC 仍关闭/不支持。

只运行 IMU：

```bash
ros2 run robobaton_4p_ros2_demo robobaton_sensors_node --ros-args \
  -p enable_camera:=false -p enable_imu:=true
```

检查 IMU 接收频率：

```bash
ros2 run robobaton_4p_ros2_demo robobaton_imu_rate_monitor
```

默认 `imu.sample_rate_hz=800`，monitor 的稳定窗口应接近 `800Hz`。相机帧率和 IMU 采样率相互独立。

当前源码 FastDDS profile 为每个 participant 预留 `256 MiB`；三个独立进程至少需 `768 MiB` 加 DDS 元数据和端口资源。启动前检查 `/dev/shm`、可用 RAM 及其他 participant，订阅端使用相同 profile。容量加固不代表全部历史 SHM 初始化错误已定位，也不保证任意进程数或负载。

## 支持边界摘要

- 相机支持 `25/30/40/50/60fps`，默认 `30fps`；只支持单路或完整四路，不支持 2/3 路组合。
- `camera.rotate_degrees=180` 只支持 `30fps`；其他公开帧率使用 `180` 会被拒绝。
- raw 图像为 NV12；compressed 使用 X5 硬件 JPEG；ROS2 路径不提供 RTSP。
- IMU 原生支持 `25/50/100/200/400/800/1600Hz`，默认800Hz；IIO watermark 固定1，`imu.buffer_capacity` 默认4096且只接受256..8192的2次幂；不重采样。
- 所有相机采集入口仅支持 `vin_lpwm`（VIN LPWM 硬件触发），`observer`/`official` 两种时间路径均输出 `CLOCK_REALTIME`；Image/CameraInfo 与 IMU/Temperature 的 `header.stamp` 使用输出侧时间，不添加固定 offset。运行中跟随系统校时，允许回拨；不是仿真 `/clock`，`use_sim_time` 不转换这些 stamp。仅 `observer` 使用 SIF MMIO observer，要求 X5 `/dev/mem` 只读访问；所选路径映射/metadata 无效即停止、不互相兜底。同域和共用 LPWM IO 不保证样本/曝光关联；不提供 TF、外参、相机标定或可用的 IMU orientation。
- `CameraInfo` 只有当前帧宽高，标定字段为空；完整时间戳和 buffer 语义见数据合同。
- `imu.sample_drop_policy` 为 `allow_counted`（默认）或 `strict`，仅控制无效六轴 payload；旧 `imu.read_mode` 与 FSYNC 参数已删除。IMU/Temperature Header 按 `imu_timestamp_mode` 使用每 burst 显式 MONOTONIC→REALTIME 或 RAW→REALTIME 桥，保留原生证据，不提供触发—样本关联。
- STOP 期间保持输入与发布线程工作，完整排空 cohort 并 ACK 后终结；最终 health 包括失败 stop 诊断、record/ACK、fatal 与尾部状态，`transport_complete` 不表示所有物理样本都已交付。关闭 rate metrics 时发布计数仍为 unavailable，不能用 producer 数量代替 sink/DDS 接收。
- `diagnostics.rate_metrics_enabled=false` 时，最终 `IMU_RUNTIME_HEALTH` 明示 `imu_published_samples=unavailable imu_publish_count_valid=no`；开启后显示实际 IMU `publish()` 成功次数并标记 `yes`。它不是 DDS 接收确认；`published_samples` 始终是独立的 producer 健康计数。关闭诊断不额外读取时钟或更新 metrics。
- 正常 SIGINT/SIGTERM 由 owner 先停止 producer、接纳并排空停止前回调，再析构节点，最后关闭 ROS context；致命错误仍按失败处理。修复后四目30fps＋IMU1600Hz短窗 smoke 的 producer/sink/DDS 数量相等，不证明历史差1根因或持续完整性。此前60fps持续候选未达到请求帧率，尚无最终组成的关闭证据；non-ROS console 成功不能替代 ROS2 高档或保存验收，不承诺全组合零丢失或新组成8小时稳定。
- 源码、本地 install、板上正式 install 与正式发布分别识别；一次 install verifier 通过不代表目标板已升级。LPWM 捕获超窗仍显式停止，SHM 容量改善不等于历史初始化错误根因已关闭。统一限制见[发布与支持](https://4p-docs.readthedocs.io/zh-cn/latest/release-and-support.html)。

## 详细文档

- [产品介绍](https://4p-docs.readthedocs.io/zh-cn/latest/getting-started/Product_Introduction.html)
- [产品版本与兼容性](https://4p-docs.readthedocs.io/zh-cn/latest/getting-started/product-and-compatibility.html)
- [ROS2 Demo 使用](https://4p-docs.readthedocs.io/zh-cn/latest/usage/ros2-demo.html)
- [公开 Demo 源码编译](https://4p-docs.readthedocs.io/zh-cn/latest/development/open-source-build.html)
- [部署、升级与回滚](https://4p-docs.readthedocs.io/zh-cn/latest/development/deployment-and-upgrade.html)
- [数据合同](https://4p-docs.readthedocs.io/zh-cn/latest/development/data-contracts.html)
- [API 参考](https://4p-docs.readthedocs.io/zh-cn/latest/development/api-reference.html)
- [故障排查](https://4p-docs.readthedocs.io/zh-cn/latest/troubleshooting.html)
- [版本更新记录](https://4p-docs.readthedocs.io/zh-cn/latest/changelog.html)

许可证和第三方组件说明以本仓库的 `LICENSE` 及发布说明为准。
