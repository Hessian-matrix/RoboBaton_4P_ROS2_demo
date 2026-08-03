# ROS2 Release Subrepository Rules

本子仓受主仓`/root/x5/4cam/AGENTS.md`约束。Agent从本目录启动时也必须先读取主仓规则。

## 强制规则

1. 所有测试相关内容放在主仓，目标位置是`/root/x5/4cam/tests/robobaton_4p_ros2_demo/`；本子仓不得新增`tests/`、perf matrix、内部probe、板端validation runner或测试证据。
2. 只保留面向用户的ROS2 node源码、公开头、运行库、config/launch、构建/install verifier和README。
3. 代码只能写当前行为的解释性注释；禁止日期和修改原因式注释，禁止`修改原因`、`修改说明`、`新增原因`和changelog式注释。
4. DDS probe、topic-rate runner、matrix、parser、classifier和内部测试合同全部归主仓。
5. 修改后必须在主仓运行`python3 tests/repository_policy_test.py`、ROS2 host测试、fresh构建和install verifier。
6. 默认不stage、不commit、不push、不tag；禁止无边界reset/clean/restore。
