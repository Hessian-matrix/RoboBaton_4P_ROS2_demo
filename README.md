# RoboBaton_4P_ROS2_demo

RoboBaton 4P 的 ROS2 `ament_cmake` 示例包，发布 SC132 四目 NV12 图像和 ICM-42688 IMU 数据。

> **最终用户说明以 [4P_doc](https://4p-docs.readthedocs.io/en/latest/index.html) 为准。** 本 README 只保留 ROS2 仓库入口、最小构建/运行方式和支持边界。

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

完整 topic、消息类型、QoS、相机映射、时间戳和数据语义见 [ROS2 Demo 使用](https://4p-docs.readthedocs.io/en/latest/ros2-demo.html) 和 [数据合同](https://4p-docs.readthedocs.io/en/latest/data-contracts.html)。

## 版本查询

安装后执行：

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_demo/install/robobaton_ros2_env.bash

/root/ros2_demo/install/lib/robobaton_4p_ros2_demo/robobaton_sensors_node --version
/root/ros2_demo/install/lib/robobaton_4p_ros2_demo/robobaton_imu_rate_monitor --version
```

版本查询不会初始化 ROS graph、相机或 IMU。产品版本、ABI 和发布组合见 [产品版本与兼容性](https://4p-docs.readthedocs.io/en/latest/product-and-compatibility.html)、[API 参考](https://4p-docs.readthedocs.io/en/latest/api-reference.html) 和 [版本更新记录](https://4p-docs.readthedocs.io/en/latest/changelog.html)。

## 构建

构建前需要：

- 本仓库 `./lib/libicm42688.so` 和 `./lib/libsc132.so`；
- X5 交叉编译包及其目标侧 ROS2、媒体编码头文件、vendor 库和 aarch64 工具链；
- 主机 ROS Humble Python 环境，且 `/usr/bin/python3` 能导入 `ament_package`。

本仓库只构建 ROS2 consumer 和 compressed image_transport plugin，不从父仓库读取 producer 源码，也不在这里重新构建 producer 库。

```bash
export X5_CROSS_ROOT="/path/to/cross_compile/new"
set +u
source /opt/ros/humble/setup.bash
set -u
script/build_x5_ros2.sh --clean --cross-root "$X5_CROSS_ROOT"
```

脚本将产物放到本仓库的 `1.ros2_build/{build,install,log}`，默认生成 merged install。完整依赖、构建参数、install verifier 和发布包规则见 [公开 Demo 源码编译](https://4p-docs.readthedocs.io/en/latest/open-source-build.html)。

## 部署与运行

部署到 `/root/ros2_demo/install` 前，必须使用完整 archive checksum、解包后的 runtime `abi_manifest.sha256`、旧应用退出检查、旧目录备份、原子切换和 smoke 验证。失败时恢复最近备份。完整命令见 [部署、升级与回滚](https://4p-docs.readthedocs.io/en/latest/deployment-and-upgrade.html)。

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

默认 `imu.sample_rate_hz=1000`，monitor 的稳定窗口应接近 `1000Hz`。相机帧率和 IMU 采样率相互独立。

## 支持边界摘要

- 相机支持 `25/30/40/50/60fps`，默认 `30fps`；只支持单路或完整四路，不支持 2/3 路组合。
- `camera.rotate_degrees=180` 只支持 `30fps`；其他公开帧率使用 `180` 会被拒绝。
- raw 图像为 NV12；compressed 使用 X5 硬件 JPEG；ROS2 路径不提供 RTSP。
- IMU 支持 `25/50/100/200/500/1000/2000Hz`，默认 `1000Hz`，使用 sensor-timestamp FIFO。
- 默认 trigger 为 `software_gpio`；不提供TF、外参、相机标定或可用的 IMU orientation。
- `CameraInfo` 只有当前帧宽高，标定字段为空；完整时间戳和 buffer 语义见数据合同。

## 详细文档

- [产品介绍](https://4p-docs.readthedocs.io/en/latest/Product_Introduction.html)
- [产品版本与兼容性](https://4p-docs.readthedocs.io/en/latest/product-and-compatibility.html)
- [ROS2 Demo 使用](https://4p-docs.readthedocs.io/en/latest/ros2-demo.html)
- [公开 Demo 源码编译](https://4p-docs.readthedocs.io/en/latest/open-source-build.html)
- [部署、升级与回滚](https://4p-docs.readthedocs.io/en/latest/deployment-and-upgrade.html)
- [数据合同](https://4p-docs.readthedocs.io/en/latest/data-contracts.html)
- [API 参考](https://4p-docs.readthedocs.io/en/latest/api-reference.html)
- [故障排查](https://4p-docs.readthedocs.io/en/latest/troubleshooting.html)
- [版本更新记录](https://4p-docs.readthedocs.io/en/latest/changelog.html)

许可证和第三方组件说明以本仓库的 `LICENSE` 及发布说明为准。
