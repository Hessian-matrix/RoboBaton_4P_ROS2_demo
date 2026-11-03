# RoboBaton_4P_ROS2_demo

[中文使用文档](https://4p-docs.readthedocs.io/zh-cn/latest/usage/ros2-demo.html) · [English documentation](https://4p-doc-en.readthedocs.io/en/latest/usage/ros2-demo.html)

面向 X5 / ROS2 Humble 的 `ament_cmake` 示例包，发布 SC132 四目图像和 ICM-42670-P IMU 数据。接线、部署、参数、topics、数据语义和排障说明统一维护在公开文档仓库 [4P_doc](https://github.com/Hessian-matrix/4P_doc)。

## 仓库内容

- `src/`、`include/`、`lib/`：节点、compressed image_transport 插件、公开头文件与匹配的预编译库。
- `config/`、`launch/`：默认配置与启动入口。
- `script/`：交叉构建及 install 校验工具。
- 主要程序：`robobaton_sensors_node`、`robobaton_imu_rate_monitor`。

本仓库不编译底层相机/IMU 运行库。头文件、库、配置与程序须成套匹配；部署完整 install 树并保留符号链接，不与 non-ROS 运行包混用。

## 开始使用

先按[源码编译](https://4p-docs.readthedocs.io/zh-cn/latest/development/open-source-build.html)准备 install 包，并按[部署、升级与回滚](https://4p-docs.readthedocs.io/zh-cn/latest/development/deployment-and-upgrade.html)完成部署，再在 X5 上执行：

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_demo/install/robobaton_ros2_env.bash
ros2 launch robobaton_4p_ros2_demo robobaton_sensors.launch.py
```

按 `Ctrl+C` 正常停止。保持 `cam-service` 运行，不与其他相机应用并发。依赖、SHM 配置、参数和订阅方法见 [ROS2 Demo 使用](https://4p-docs.readthedocs.io/zh-cn/latest/usage/ros2-demo.html)；ROS2 路径不提供 RTSP。

## 文档入口

- [首次上电](https://4p-docs.readthedocs.io/zh-cn/latest/getting-started/first-boot.html) · [硬件连接与安全](https://4p-docs.readthedocs.io/zh-cn/latest/getting-started/hardware-and-safety.html)
- [数据合同](https://4p-docs.readthedocs.io/zh-cn/latest/development/data-contracts.html) · [API 参考](https://4p-docs.readthedocs.io/zh-cn/latest/development/api-reference.html)
- [故障排查](https://4p-docs.readthedocs.io/zh-cn/latest/troubleshooting.html)
- [版本、已知限制与支持](https://4p-docs.readthedocs.io/zh-cn/latest/release-and-support.html)

许可证及第三方组件说明见本仓库 [LICENSE](LICENSE) 和公开发布说明。
