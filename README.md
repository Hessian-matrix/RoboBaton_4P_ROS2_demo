# RoboBaton_4P_ROS2_demo

RoboBaton 4P 的 ROS2 `ament_cmake` 示例包，发布 SC132 四目 NV12 图像和 ICM-42688 IMU 数据。

## 1. 目标产物

- 节点：`robobaton_sensors_node`
- IMU 频率检查工具：`robobaton_imu_rate_monitor`
- Launch：`launch/robobaton_sensors.launch.py`
- 默认配置：`config/robobaton_sensors.yaml`

## 版本查询

仓库`VERSION`、`package.xml`和install中的`share/robobaton_4p_ros2_demo/VERSION`使用同一产品SemVer。部署后先source ROS2和本包overlay，再直接调用可执行文件；版本路径不会初始化ROS graph或相机/IMU：

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_demo/install/setup.bash

/root/ros2_demo/install/lib/robobaton_4p_ros2_demo/robobaton_sensors_node --version
/root/ros2_demo/install/lib/robobaton_4p_ros2_demo/robobaton_imu_rate_monitor --version
```

`robobaton_sensors_node`同时输出实际加载的`libicm42688`和`libsc132`版本。compressed image_transport插件SO导出`robobaton_nv12_compressed_image_transport_get_version()` C符号。产品SemVer与SO的SONAME/ABI版本相互独立。功能新增、问题修复和已知限制见[公开版本更新记录](https://github.com/Hessian-matrix/4P_doc/blob/main/source/changelog.md)。

## 2. Topics

```text
/robobaton/cam0/image_raw
/robobaton/cam0/image_raw/compressed
/robobaton/cam0/camera_info
/robobaton/cam1/image_raw
/robobaton/cam1/image_raw/compressed
/robobaton/cam1/camera_info
/robobaton/cam2/image_raw
/robobaton/cam2/image_raw/compressed
/robobaton/cam2/camera_info
/robobaton/cam3/image_raw
/robobaton/cam3/image_raw/compressed
/robobaton/cam3/camera_info
/robobaton/imu/data
/robobaton/imu/temperature
```

## 3. 构建

构建前需要准备：

- `./lib/libicm42688.so`
- `./lib/libsc132.so`
- X5 交叉编译包：目标侧 ROS2、`hb_media_codec.h`、`libmultimedia.so.1` / `libhbmem.so.1` / `libalog.so.1` 和 aarch64 工具链。
- 宿主机 ROS2 Python 环境：`/usr/bin/python3` 必须能导入 `ament_package`；当前验证方式是先 source `/opt/ros/humble/setup.bash`。

ROS2 节点只链接上文列出的 ICM/SC 生产库；NV12 compressed image_transport 插件链接 X5 `libmultimedia.so.1`、`libhbmem.so.1` 和 `libalog.so.1`，将 ROS `Image` 的有效 NV12 行复制到 media-codec 内部输入 buffer 后以 `MEDIA_CODEC_ID_JPEG` 硬件编码，不拆分 I420，也不使用 CPU TurboJPEG。本仓不携带其他媒体协议库，也不要求运行非 ROS 构建脚本。两个前置 SO 的上游构建入口：

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

相对 `--build-base` / `--install-base` / `--log-base` 会按本包目录解析，不按调用脚本时的当前目录解析。安装后 `libicm42688.so` 和 `libsc132.so` 会放到 `<install-base>/lib/robobaton_4p_ros2_demo/`；X5 JPEG codec使用板端`/usr/hobot/lib`中的vendor库，不复制到install。ROS2 运行时转发依赖由安装包内的 `libconsole_bridge`、`libspdlog`、`libfmt`、`libtinyxml2`、`libssl`、`libcrypto`、`libblas`、`liblapack`、`libgfortran` 闭包提供，环境钩子也会补入 X5 板端 `/usr/lib` 与 `/usr/lib/sensor` 以解析 ISP/相机库的传递依赖。

## 4. 运行
部署到 X5 后，install 根目录会包含 `robobaton_ros2_env.bash`。推荐用它一次性加载 ROS2 underlay、本包 overlay、FastDDS SHM profile 和日志缓冲设置：

```bash
source /root/ros2_demo/install/robobaton_ros2_env.bash
```

需要保持当前 shell 环境时用 `source`；只想运行一个命令时直接执行脚本：

```bash
/root/ros2_demo/install/robobaton_ros2_env.bash ros2 topic list --no-daemon --include-hidden-topics
```

脚本默认加载 `/opt/ros/humble/setup.bash`。板端 ROS2 路径不同时，先设置 `ROBOBATON_ROS_UNDERLAY=/path/to/setup.bash`；确需跳过 underlay 时可设置为空。overlay setup 会把 `FASTDDS_DEFAULT_PROFILES_FILE` 和兼容变量 `FASTRTPS_DEFAULT_PROFILES_FILE` 设置为 install 内的 `config/fastdds/robobaton_shm.xml`。该 profile 强制使用 FastDDS SHM、`512 MiB` segment 和 `1024` port queue；板端必须保留可写的 `/dev/shm`。相机 raw/compressed publisher 使用 Reliable QoS 与 KeepLast(8)，用于四路高帧率的大消息完整传输。

每个用于启动、查看话题或运行监控工具的终端都必须先加载同一个 `robobaton_ros2_env.bash`。`ros2 daemon` 会保留启动时的 DDS 环境；如果 daemon 在未加载本包 overlay 或 `/dev/shm` 异常时启动，后续 `ros2 topic list` 可能只看到 `/parameter_events` 和 `/rosout`。排查 graph 时优先使用 `--no-daemon`，或在加载脚本后重启 daemon。

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

环境脚本默认导出 `RCUTILS_LOGGING_BUFFERED_STREAM=0`。需要观察连续日志时，仍建议让 launch 进程按行刷新：

```bash
stdbuf -oL -eL ros2 launch robobaton_4p_ros2_demo robobaton_sensors.launch.py
```

如果终端长时间不显示、随后集中刷出多行日志，先用日志内 `ROB2_RATE` 的 `interval_*`、`source`、`published`、`dropped` 和 `publish_failures` 判断真实发布窗口；终端显示时间不等于 sensor 发布时间。

只跑 IMU：

```bash
ros2 run robobaton_4p_ros2_demo robobaton_sensors_node --ros-args \
  -p enable_camera:=false -p enable_imu:=true
```

另开一个终端验证 IMU 接收频率，使用 C++ 订阅器而不是 Python CLI：

```bash
ros2 run robobaton_4p_ros2_demo robobaton_imu_rate_monitor
```

默认订阅 `/robobaton/imu/data`，每秒输出一行：

```text
ROB2_IMU_RATE topic=/robobaton/imu/data hz=30.000 samples=30 window_s=1.000000 total=30
```

启动后的第一行可能包含 DDS 匹配和半个统计窗口，判断稳定频率时看后续连续多行。

常用参数：

```bash
ros2 run robobaton_4p_ros2_demo robobaton_imu_rate_monitor --ros-args \
  -p topic:=/robobaton/imu/data -p report_period_ms:=1000 -p qos_depth:=100
```

`robobaton_imu_rate_monitor`使用C++订阅并按固定窗口计数，用于发布门中的ROS2发布、DDS传输和C++接收链路验证。`ros2 topic hz`仍可用于交互诊断，但正式证据统一使用包内monitor，避免工具口径漂移。

查看话题列表时，可直接用环境脚本运行一次性命令，绕过可能过期的 daemon：

```bash
/root/ros2_demo/install/robobaton_ros2_env.bash ros2 topic list --no-daemon --include-hidden-topics
/root/ros2_demo/install/robobaton_ros2_env.bash ros2 node list --no-daemon
```

如果必须使用普通 `ros2 topic list`，在加载环境后重启 daemon：

```bash
/root/ros2_demo/install/robobaton_ros2_env.bash --restart-daemon
/root/ros2_demo/install/robobaton_ros2_env.bash ros2 topic list --include-hidden-topics
```

FastDDS SHM profile 会为每个 ROS2 参与进程创建较大的 `/dev/shm/fastrtps_*` segment；节点、daemon、监控工具和 CLI 同时运行时会消耗共享内存。启动前可检查：

```bash
/root/ros2_demo/install/robobaton_ros2_env.bash --check
```

只有在已停止 launch、`ros2 daemon stop` 且确认没有 `robobaton_sensors_node`、`ros2 launch`、`ros2 run` 进程时，才允许清理遗留 FastDDS SHM 文件：

```bash
/root/ros2_demo/install/robobaton_ros2_env.bash --clean-shm
```

不要在节点运行时清理 `/dev/shm/fastrtps_*`，否则会破坏正在运行的 DDS participant。

默认四相机加 IMU 成功后应能看到：

```text
/robobaton/imu/data
/robobaton/imu/temperature
/robobaton/cam0/image_raw
/robobaton/cam0/image_raw/compressed
/robobaton/cam0/camera_info
...
/robobaton/cam3/image_raw
/robobaton/cam3/image_raw/compressed
/robobaton/cam3/camera_info
```

只跑单颗相机 smoke：

```bash
ros2 run robobaton_4p_ros2_demo robobaton_sensors_node --ros-args \
  -p enable_camera:=true -p enable_imu:=false -p camera.camera_mask:=1
```

## 5. 参数边界

- `camera.camera_mask` 只支持单颗物理相机或完整四路；不支持 2/3 路组合。
- `camera.fps` 默认 `30`，仅支持`25`或`30`；其他值在启动相机前拒绝。
- `camera.rotate_degrees` 只支持 `0/90/180/270`；当前源码对所有 camera mask 的`25fps`都拒绝对外`180`度旋转。
- 发布图像和压缩流的画布随对外旋转角变化：`0/180 => 1280x1088`，`90/270 => 1088x1280`；ROS2 raw/compressed 的 `width`/`height` 必须使用对应画布。
- 第一版不暴露 `camera.width` / `camera.height`。SC132 frame-set 初始化固定使用 sensor 原始尺寸 `kSensorInputWidth/kSensorInputHeight`，ROS `Image.width/height` 使用 libsc132 帧元数据。
- `camera.image_encoding` 第一版只支持 `nv12`。
- `camera.publish_compressed_image` 默认启用，通过 `image_transport` 只注册 raw 与 compressed 发布插件；有 `/image_raw/compressed` 订阅者时才把有效NV12行复制到X5 media-codec内部buffer，并以`MEDIA_CODEC_ID_JPEG`执行硬件单帧压缩。
- `camera.compressed_jpeg_quality` 会写入每路 `image_transport` compressed 插件的 `jpeg_quality` 参数，取值 `1..100`，默认 `80`。
- `camera.trigger_mode` 默认 `software_gpio`，也是 V1 唯一已验证的稳定模式；`vin_lpwm` 和 `none` 为实验性 / 未验收参数，不属于 V1 稳定合同。
- `imu.read_mode` 只支持 `sensor_timestamp_fifo`；`imu.fifo_watermark_samples` 固定为 `1`，匹配 ICM-42688 sensor timestamp FIFO 映射。

## 6. 数据语义与限制

- 图像主 topic 是 NV12，不保证 `rqt_image_view`、`cv_bridge` 或常规 RGB/BGR 工具可直接显示；需要通用可视化时通过 `image_transport` 订阅 compressed transport。
- `sensor_msgs::msg::Image::step` 使用底层 DMA buffer 的 `stride`；`data.size()` 使用 `hb_mem_graphic_buf_t::size[0] + size[1]`，可能大于紧凑 `width * height * 3 / 2`。compressed 插件按 NV12 总字节比例反推垂直对齐，只压缩有效宽高区域。
- 第一版不做相机和 IMU 硬同步，不发布 TF 外参，不加载相机标定；`CameraInfo` 只带当前帧宽高，畸变模型为空。
- `camera.queue_policy=drop_newest` 会按单路丢弃最新帧，ROS 发布层不再保证完整四帧组；需要保持组完整性时使用默认 `block`。
- SC132 frame-set 的 `group_id` 和 max skew 当前不写入 ROS 消息；`Image.header.stamp` 使用 SC132 `timestamp_ns` 映射到系统实时时间，默认 `software_gpio` 下该字段来自 `CLOCK_MONOTONIC_RAW` 域。
- IMU 和温度消息复用同一 `header.stamp`；时间戳使用 ICM-42688 `sample_timestamp_ns` 映射到系统实时时间，不使用回调入队时的 `host_timestamp_ns`。
- IMU orientation 不可用，发布时设置 `orientation_covariance[0] = -1.0`；gyro/accel 协方差第一版不伪造。
- 当前 package 未注册 CTest/单元测试，构建脚本使用 `BUILD_TESTING=OFF`；交叉构建成功不能替代 X5 板端 topic/频率/NV12 布局验证。

## 7. 快速检查

```bash
source /root/ros2_demo/install/robobaton_ros2_env.bash
ros2 daemon stop
ros2 topic list --no-daemon --include-hidden-topics
ros2 run robobaton_4p_ros2_demo robobaton_imu_rate_monitor
ros2 topic hz /robobaton/cam0/image_raw
ros2 topic echo /robobaton/cam0/camera_info --once
ros2 topic hz /robobaton/cam0/image_raw/compressed
```
