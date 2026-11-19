# RoboBaton_4P_ROS2_demo

RoboBaton 4P 的 ROS2 `ament_cmake` 示例包，发布 SC132 四目 NV12 图像和 ICM-42688 IMU 数据。

> **最终用户说明以 [4P_doc](https://4p-docs.readthedocs.io/zh-cn/latest/index.html) 为准。** 本 README 只保留 ROS2 仓库入口、最小构建/运行方式和支持边界。

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
/robobaton/imu/data
/robobaton/imu/temperature
```

完整 topic、消息类型、QoS、相机映射、时间戳和数据语义见 [ROS2 Demo 使用](https://4p-docs.readthedocs.io/zh-cn/latest/usage/ros2-demo.html) 和 [数据合同](https://4p-docs.readthedocs.io/zh-cn/latest/development/data-contracts.html)。

## 版本查询

安装后执行：

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_demo/install/robobaton_ros2_env.bash

/root/ros2_demo/install/lib/robobaton_4p_ros2_demo/robobaton_sensors_node --version
/root/ros2_demo/install/lib/robobaton_4p_ros2_demo/robobaton_imu_rate_monitor --version
```

版本查询不会初始化 ROS graph、相机或 IMU。产品版本、ABI 和发布组合见 [产品版本与兼容性](https://4p-docs.readthedocs.io/zh-cn/latest/getting-started/product-and-compatibility.html)、[API 参考](https://4p-docs.readthedocs.io/zh-cn/latest/development/api-reference.html) 和 [版本更新记录](https://4p-docs.readthedocs.io/zh-cn/latest/changelog.html)。

当前 `1.3.4` 候选构建使用 `libicm42688.so.2.1.2` 和 `libsc132.so.2.0.2`；SONAME 仍为 `libicm42688.so.2`、`libsc132.so.2`，ABI 不变。软件 `VERSION`、`package.xml` 与版本接口报告 `1.3.4`；请同时核验真实库文件名和 `abi_manifest.sha256`。正式发布状态见文档站的发布与支持页面。

源码 `lib/` 和 install 的 `lib/robobaton_4p_ros2_demo/` 每库只有一份真实 ELF，另有 `.so -> .so.2 -> 具体版本文件` 两级同目录相对链接。构建与 `script/verify_install.py` 严格检查链接类型和目标；`abi_manifest.sha256` 继续包含别名的内容哈希。使用 tar、`cp -a` 或 `rsync -a` 保留链接，不能只用哈希相等代替链接校验。

## 构建

构建前需要：

- 本仓库 `./lib/libicm42688.so` 和 `./lib/libsc132.so`；
- `x5_4cam_cross_toolchain_20261009` 工具包，包含目标侧 ROS2（含 image_transport/pluginlib）、媒体头库和固定的 Arm GNU AArch64 编译器；
- Ubuntu 22.04 x86_64 主机、ROS Humble 和 `python3-colcon-common-extensions`；按下方源码编译页配置官方软件源并安装，系统 Python 必须能导入 ament。

本仓库只构建 ROS2 consumer 和 compressed image_transport plugin，不从父仓库读取 producer 源码，也不在这里重新构建 producer 库。

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

环境脚本通过已安装的 ament hook 补充 `/usr/hobot/lib` 与 `/usr/hobot/lib/sensor`（目录存在时），用于解析 X5 媒体库的 `libalog.so.1` 等传递依赖；无需手工追加 `LD_LIBRARY_PATH`。FastDDS 继续使用安装包的 SHM profile，不切换到 UDP。

相机与 IMU 同时启用时，先完成相机初始化，再建立 IMU 采样时间域；正常停止时先停 IMU，再回收相机，避免相机初始化/销毁与 IMU 采样重叠。只启用一种传感器时不启动另一种。

默认 FastDDS SHM segment 为每 participant `256 MiB`，不是所有进程共享一个配额。节点、随包提供的 IMU monitor、订阅程序和 CLI/daemon 都可能各创建 participant；并发启动前用 `--check` 确认 `/dev/shm` 可用空间覆盖各 segment、端口和管理开销。四个 participant 仅 segment 合计约 `1 GiB`；不要在已有进程运行时清理 SHM 文件。

只运行 IMU：

```bash
ros2 run robobaton_4p_ros2_demo robobaton_sensors_node --ros-args \
  -p enable_camera:=false -p enable_imu:=true
```

检查 IMU 接收频率：

```bash
ros2 run robobaton_4p_ros2_demo robobaton_imu_rate_monitor
```

默认 `imu.sample_rate_hz=1000`，monitor 的稳定窗口应接近 `1000Hz`。相机帧率和 IMU 采样率相互独立。

## 支持边界摘要

- 相机支持 `25/30/40/50/60fps`，默认 `30fps`；只支持单路或完整四路，不支持 2/3 路组合。
- `camera.rotate_degrees=180` 只支持 `30fps`；其他公开帧率使用 `180` 会被拒绝。
- raw 图像为 NV12；compressed 使用 X5 硬件 JPEG；ROS2 路径不提供 RTSP。
- IMU 支持 `25/50/100/200/500/1000/2000Hz`，默认 `1000Hz`，使用 sensor-timestamp FIFO。
- `camera.trigger_mode` 只接受默认值 `software_gpio`，使用 GPIO417 软件触发；其他值在访问硬件前被拒绝，无别名或回退。不提供TF、外参、相机标定或可用的 IMU orientation。
- compressed JPEG 仅移除 X5 JPU 紧随 SOI 的私有 APP9 帧序号，保留其他 metadata、编码表和图像 scan 字节，兼容通用 JPEG 解码器。
- 每路 worker 复用自己的 raw Image 存储；`publish(const&)` 返回后才覆盖。JPEG 输入无行 padding 时整块拷贝，否则逐行拷贝有效像素，不改变 NV12 布局、QoS 或硬件 JPEG 质量。
- `CameraInfo` 只有当前帧宽高，标定字段为空；完整时间戳和 buffer 语义见数据合同。

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
