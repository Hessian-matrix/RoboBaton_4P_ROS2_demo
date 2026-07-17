# RoboBaton_4P_ROS2_demo

RoboBaton 4P 的 ROS2 `ament_cmake` 示例包，发布 SC132 四目 NV12 图像和 ICM-42688 IMU 数据。

## 1. 目标产物

- 节点：`robobaton_sensors_node`
- Launch：`launch/robobaton_sensors.launch.py`
- 默认配置：`config/robobaton_sensors.yaml`

## 2. Topics

```text
/robobaton/cam0/image_raw
/robobaton/cam0/camera_info
/robobaton/cam1/image_raw
/robobaton/cam1/camera_info
/robobaton/cam2/image_raw
/robobaton/cam2/camera_info
/robobaton/cam3/image_raw
/robobaton/cam3/camera_info
/robobaton/imu/data
/robobaton/imu/temperature
```

## 3. 构建

构建前需要准备：

- `./lib/libicm42688.so`
- `./lib/libsc132.so`
- X5 交叉编译包：目标侧 ROS2、`hb_mem_mgr.h`、`libhbmem.so` 和 aarch64 工具链。
- 宿主机 ROS2 Python 环境：`/usr/bin/python3` 必须能导入 `ament_package`；当前验证方式是先 source `/opt/ros/humble/setup.bash`。

ROS2 节点只链接上文列出的 ICM/SC 生产库；本仓不携带其他媒体协议库，也不要求运行非 ROS 构建脚本。两个前置 SO 的上游构建入口：

```bash
# 按实际安装位置设置 X5 交叉编译包根目录；脚本默认值可用 --cross-root 或 X5_CROSS_ROOT 覆盖。
export X5_CROSS_ROOT=<cross_compile/new>

# libicm42688.so：根工程 CMake target，X5 交叉构建时自动刷新本包 ./lib 和公共头。
cmake -S . -B build_x5 \
  -DCMAKE_TOOLCHAIN_FILE="${X5_CROSS_ROOT}/toolchain/aarch64_x5_host_toolchain.cmake" \
  -DROBOBATON_SYNC_BUILT_SHARED_LIBS=ON
cmake --build build_x5 --target icm42688_x5

# libsc132.so：构建完成后刷新本包 ./lib 和公共头。
bash scripts/build_sc132.sh
```

推荐入口是包内脚本；它会按 `--cross-root <path>` 或 `X5_CROSS_ROOT` 解析交叉编译包，并加载 `<cross-root>/scripts/setup_x5_cross_env.sh`。如环境脚本不在该位置，可用 `--cross-env <path>` 显式指定。脚本会显式传入 `CMAKE_TOOLCHAIN_FILE`，并把 colcon 产物固定到本包的 `1.ros2_build/{build,install,log}`。默认使用 `--merge-install`，生成常规 ROS2 install 目录：`1.ros2_build/install/{lib,share,local_setup.*,setup.*}`。

```bash
cd <4cam-workspace>
set +u
source /opt/ros/humble/setup.bash
set -u
sub_module/RoboBaton_4P_ROS2_demo/script/build_x5_ros2.sh --clean --cross-root "${X5_CROSS_ROOT}"
```

如果跳过宿主机 ROS 环境，当前交叉环境脚本只设置目标侧 CMake/ament 前缀，CMake 运行 `ament_cmake_core` 的 Python 模板时可能报 `ModuleNotFoundError: No module named 'ament_package'`。

常用覆盖项：

```bash
sub_module/RoboBaton_4P_ROS2_demo/script/build_x5_ros2.sh \
  --install-base install_x5 \
  --parallel-workers 1 \
  -- --event-handlers console_direct+
```

相对 `--build-base` / `--install-base` / `--log-base` 会按本包目录解析，不按调用脚本时的当前目录解析。安装后 `libicm42688.so` 和 `libsc132.so` 会放到 `<install-base>/lib/robobaton_4p_ros2_demo/`；`libhbmem.so` 仍来自板端 `/usr/hobot/lib` 或 `/usr/hobot/lib/sensor`。

## 4. 运行
部署到 X5 后，正式运行入口使用可自定位的 Bash setup；先加载板端 ROS2，再加载本包 overlay：

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_demo/install/setup.bash
```

不要直接 source 已搬迁 install 的 POSIX `setup.sh`：POSIX shell 无法获知被 source 文件自身路径。确需 `sh` 时必须显式提供前缀：

```sh
. /opt/ros/humble/setup.sh
COLCON_CURRENT_PREFIX=/root/ros2_demo/install \
  . /root/ros2_demo/install/setup.sh
```


默认同时启用相机和 IMU：

```bash
ros2 launch robobaton_4p_ros2_demo robobaton_sensors.launch.py
```

只跑 IMU：

```bash
ros2 run robobaton_4p_ros2_demo robobaton_sensors_node --ros-args \
  -p enable_camera:=false -p enable_imu:=true
```

只跑单颗相机 smoke：

```bash
ros2 run robobaton_4p_ros2_demo robobaton_sensors_node --ros-args \
  -p enable_camera:=true -p enable_imu:=false -p camera.camera_mask:=1
```

## 5. 参数边界

- `camera.camera_mask` 只支持单颗物理相机或完整四路；不支持 2/3 路组合。
- `camera.fps` 只支持 `30` 或 `60`。
- `camera.rotate_degrees` 只支持 `0/90/180/270`；当前源码对所有 camera mask 的 `60fps` 都拒绝对外 `180` 度旋转。
- 第一版不暴露 `camera.width` / `camera.height`。SC132 frame-set 初始化固定使用 sensor 原始尺寸 `kSensorInputWidth/kSensorInputHeight`，ROS `Image.width/height` 使用 libsc132 帧元数据。
- `camera.image_encoding` 第一版只支持 `nv12`。
- `timestamp_source` 第一版只支持 `ros_now`。
- `camera.trigger_mode` 当前在 ROS 参数层没有枚举校验；请只使用已验证的 `software_gpio`、`vin_lpwm` 或 `none`。

## 6. 数据语义与限制

- 图像是 NV12，不保证 `rqt_image_view`、`cv_bridge` 或常规 RGB/BGR 工具可直接显示。
- `sensor_msgs::msg::Image::step` 使用底层 DMA buffer 的 `stride`；`data.size()` 使用 `hb_mem_graphic_buf_t::size[0] + size[1]`，可能大于紧凑 `width * height * 3 / 2`。底层 `vstride` 代表垂直对齐，但标准 ROS Image 没有对应字段。
- 第一版不做相机和 IMU 硬同步，不发布 TF 外参，不加载相机标定；`CameraInfo` 只带当前帧宽高，畸变模型为空。
- `camera.queue_policy=drop_newest` 会按单路丢弃最新帧，ROS 发布层不再保证完整四帧组；需要保持组完整性时使用默认 `block`。
- SC132 frame-set 的 `group_id`、sensor timestamp 和 max skew 当前不写入 ROS 消息。
- 相机图像、CameraInfo、IMU 和温度消息的 `header.stamp` 都是 ROS 发布时间，不是 SC132 sensor timestamp，也不是 IMU `host_timestamp_ns`。
- IMU orientation 不可用，发布时设置 `orientation_covariance[0] = -1.0`；gyro/accel 协方差第一版不伪造。
- 当前 package 未注册 CTest/单元测试，构建脚本使用 `BUILD_TESTING=OFF`；交叉构建成功不能替代 X5 板端 topic/频率/NV12 布局验证。

## 7. 快速检查

```bash
ros2 topic hz /robobaton/imu/data
ros2 topic hz /robobaton/cam0/image_raw
ros2 topic echo /robobaton/cam0/camera_info --once
```
