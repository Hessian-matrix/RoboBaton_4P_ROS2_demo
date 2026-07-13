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

`libprrtsp.so` 会随上游库构建同步到 `./lib`，但不是 ROS2 节点构建前置。相关入口：

```bash
# 按实际安装位置设置 X5 交叉编译包根目录；脚本默认值可用 --cross-root 或 X5_CROSS_ROOT 覆盖。
export X5_CROSS_ROOT=<cross_compile/new>

# libicm42688.so：根工程 CMake target，X5 交叉构建时自动刷新本包 ./lib。
cmake -S . -B build_x5 \
  -DCMAKE_TOOLCHAIN_FILE="${X5_CROSS_ROOT}/toolchain/aarch64_x5_host_toolchain.cmake" \
  -DROBOBATON_SYNC_BUILT_SHARED_LIBS=ON
cmake --build build_x5 --target icm42688_x5

# libsc132.so：构建完成后刷新本包 ./lib/libsc132.so。
bash scripts/build_sc132.sh

# libprrtsp.so：刷新本包 ./lib/libprrtsp.so；不是 ROS2 包 build_x5_ros2.sh 的前置依赖。
bash scripts/build_rtsp_so_mp4.sh
```

推荐入口是包内脚本；它会按 `--cross-root <path>` 或 `X5_CROSS_ROOT` 解析交叉编译包，并加载 `<cross-root>/scripts/setup_x5_cross_env.sh`。如环境脚本不在该位置，可用 `--cross-env <path>` 显式指定。脚本会显式传入 `CMAKE_TOOLCHAIN_FILE`，并把 colcon 产物固定到本包的 `1.ros2_build/{build,install,log}`。默认使用 `--merge-install`，生成常规 ROS2 install 目录：`1.ros2_build/install/{lib,share,local_setup.*,setup.*}`。

```bash
cd <4cam-workspace>
sub_module/RoboBaton_4P_ROS2_demo/script/build_x5_ros2.sh --clean --cross-root "${X5_CROSS_ROOT}"
```

常用覆盖项：

```bash
sub_module/RoboBaton_4P_ROS2_demo/script/build_x5_ros2.sh \
  --install-base install_x5 \
  --parallel-workers 1 \
  -- --event-handlers console_direct+
```

相对 `--build-base` / `--install-base` / `--log-base` 会按本包目录解析，不按调用脚本时的当前目录解析。安装后 `libicm42688.so` 和 `libsc132.so` 会放到 `<install-base>/lib/robobaton_4p_ros2_demo/`；`libhbmem.so` 仍来自板端 `/usr/hobot/lib` 或 `/usr/hobot/lib/sensor`。

## 4. 运行

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
- `camera.rotate_degrees` 只支持 `0/90/180/270`；四路 `60fps` 不支持对外 `180` 度旋转。
- 第一版不暴露 `camera.width` / `camera.height`。SC132 frame-set 初始化固定使用 sensor 原始尺寸 `kSensorInputWidth/kSensorInputHeight`，ROS `Image.width/height` 使用 libsc132 帧元数据。
- `camera.image_encoding` 第一版只支持 `nv12`。
- `timestamp_source` 第一版只支持 `ros_now`。

## 6. 数据语义与限制

- 图像是 NV12，不保证 `rqt_image_view`、`cv_bridge` 或常规 RGB/BGR 工具可直接显示。
- `sensor_msgs::msg::Image::step` 使用底层 DMA buffer 的 `stride`；`data.size()` 使用 `hb_mem_graphic_buf_t::size[0] + size[1]`，可能大于紧凑 `width * height * 3 / 2`。底层 `vstride` 代表垂直对齐，但标准 ROS Image 没有对应字段。
- 第一版不做相机和 IMU 硬同步，不发布 TF 外参，不加载相机标定；`CameraInfo` 只带当前帧宽高，畸变模型为空。
- 相机图像、CameraInfo、IMU 和温度消息的 `header.stamp` 都是 ROS 发布时间，不是 SC132 sensor timestamp，也不是 IMU `host_timestamp_ns`。
- IMU orientation 不可用，发布时设置 `orientation_covariance[0] = -1.0`；gyro/accel 协方差第一版不伪造。

## 7. 快速检查

```bash
ros2 topic hz /robobaton/imu/data
ros2 topic hz /robobaton/cam0/image_raw
ros2 topic echo /robobaton/cam0/camera_info --once
```
