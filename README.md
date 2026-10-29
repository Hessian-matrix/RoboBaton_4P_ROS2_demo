# RoboBaton_4P_ROS2_demo

RoboBaton 4P V2 的 ROS2 `ament_cmake` 示例包，发布 SC132 四目 NV12 图像和 ICM-42670-P IMU 数据。

> **最终用户说明以 [4P_doc](https://4p-docs.readthedocs.io/zh-cn/latest/index.html) 为准。** 本 README 只保留 ROS2 仓库入口、最小构建/运行方式和支持边界。

当前源码使用 `icm42670_*` / `icm42670_driver.h`，IMU ABI 3.0、SONAME `libicm42670.so.3`、real SO `libicm42670.so.3.0.0`，全部导出只使用 `ICM42670_X5_3.0`。须成套重建头文件、库、配置与 consumer，移除旧 `.2` 库，不建立跨 major 链接或旧符号别名；软件 VERSION 不变，manifest 才能区分组成。FIFO 顺序与实测 TMST 定义样本身份和时间，INT1 仅唤醒；`host_timestamp_ns` 为该包 SPI FIFO burst 成功完成时的 RAW 观测，经当次系统时钟桥接转换为 realtime，同 burst 可重复，FSYNC 延迟发布保留转换后的原值，不在发布时重新换算旧 RAW；随墙钟回拨，不是 IRQ 沿/样本/曝光时间。ROS2 继续用实测 TMST→RAW→realtime 的 `sample_timestamp_ns`，以 `sample_sequence` 判顺序；GPIO 通知缺口仅诊断、不单独失败。`200µs` admission、坏包/FIFO overflow/时钟歧义/无效 epoch 的失败边界不变。本架构不修复底层 GPIO/内核原因、不改 LPWM/内核、不承诺任意负载零丢失。

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
/robobaton/fsync                # 仅显式启用后发布
```

完整 topic、消息类型、QoS、相机映射、时间戳和数据语义见 [ROS2 Demo 使用](https://4p-docs.readthedocs.io/zh-cn/latest/usage/ros2-demo.html) 和 [数据合同](https://4p-docs.readthedocs.io/zh-cn/latest/development/data-contracts.html)。

`/robobaton/camN/exposure` 使用 `std_msgs/msg/String` 发布 `robobaton_exposure_v1` JSON，按相机和 `timestamp_ns` 对照 Image 的触发沿 stamp。`exposure.status=1` 是随帧冻结的最新 ISP 估计值，不证明该帧实际生效曝光；`0` 表示不可用。它不改变图像时间戳，也不提供真实曝光中点。

曝光 topic 为 Reliable KeepLast(8)；按相机及 Image stamp 关联并无损解析整数 JSON，不把 `source_timestamp` 当 realtime。FSYNC 默认关闭、Reliable KeepLast(256)，必须与实际收到的 Image/CameraInfo/IMU 样本核对；关联 JSON 不是 DDS 全部送达证明。

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
- IMU 原生支持 `25/50/100/200/400/800/1600Hz`，默认 `800Hz`，使用 sensor-timestamp FIFO；旧 `500/1000/2000Hz` 明确拒绝。
- 所有相机采集入口仅支持 `vin_lpwm`（VIN LPWM 硬件触发）。相机硬件触发沿与 IMU 样本经内部 RAW 映射统一为 `CLOCK_REALTIME`；Image/CameraInfo 与 IMU/Temperature 的 `header.stamp` 原样保留 producer 时间，不添加 offset。运行中跟随系统校时，允许回拨；不是仿真 `/clock`，`use_sim_time` 不转换这些 stamp。相机需要 X5 `/dev/mem` 只读访问；映射缺失/超窗即停止，不使用到帧或 IRQ 时间兜底。INT1 为 `34120000.gpio` offset `8`。同域和共用 LPWM IO 不保证样本/曝光关联；不提供 TF、外参、相机标定或可用的 IMU orientation。
- `CameraInfo` 只有当前帧宽高，标定字段为空；完整时间戳和 buffer 语义见数据合同。
- 设置 `imu.fsync_enabled:=true` 和 `imu.sample_rate_hz:=1600` 可开启 `/robobaton/fsync`（Reliable KeepLast(256)，`std_msgs/msg/String` JSON，默认关闭，仅支持 `800/1600Hz`）。仅 `status=1` 是唯一触发—样本匹配；它不改变 IMU 采样相位，不保证每个触发或曝光同步。
- IMU 发布线程使用 `SCHED_RR/3`，采集 owner 使用 `SCHED_FIFO/20`，相机 RAW observer 使用 `SCHED_FIFO/60`、继承允许 CPU 集合并每轮休眠 `50µs`，需实时调度权限。IMU 不再使用独立 INT1 时钟 observer 或包/沿身份匹配；GPIO 通知序号缺口仅诊断、不单独失败。跨优先级队列使用优先级继承锁；应用不自动修改 IRQ/debug 或实时配额，重构不修复底层 GPIO/kernel 根因。压测须核对 stop 后最终 `IMU_RUNTIME_HEALTH`、四路持续速率和实际 DDS IMU/图像载荷，不以 topic 存在或关联 JSON 单独证明送达。
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
