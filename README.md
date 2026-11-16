# AirHUST 2026 梯队培训

亲爱的小朋友们和大朋友们你们好呀，欢迎来到**嘉豪哥哥奇妙的无人机小课堂**。
下面，你将探索奇妙的无人机小世界，由于大部分内容都是我们手写的，我们希望你们在前三关的时候仅仅将AI作为提问的工具，而不是让AI直接帮你操作哦。
你能做到吗？
你一定可以的。

## 五关分别做什么，怎么选

五关是一条逐步深入的学习路线，不是五个任选其一的任务。**第一、第二、第五关是共同任务；第三、第四关按方向开展。**

1. **第一关：把环境和 ROS 通信弄明白。** 熟悉工作空间、包、节点和编译，完成基础通信任务，为后续读代码、运行仿真做好准备。
2. **第二关：看懂无人机怎样执行任务。** 阅读 template，理解回调、参数读取、控制目标与任务推进，再自己实现绕圈。
3. **第三关：选择一个方向，补全关键代码。** 先理解原理，再通过代码补全练习掌握这个方向的基本方法。
4. **第四关：沿用第三关的方向，在小场景中完成任务。** 把补全的方法放进仿真，通过实际表现检查、调试和改进。
5. **第五关：组队完成融合项目。** 将多个方向结合起来，分工完成一个完整任务，具体项目后续公布。

先按顺序完成第一、第二关，再从下面四个方向中选一个作为第三、第四关的主方向。有余力可以继续学习其他方向，不要求先把所有方向都做完。

| 方向 | 主要学习什么 | 当前材料 |
| --- | --- | --- |
| 避障 | 根据障碍物信息生成运动目标，学习人工势场法并调试避障效果 | 已收录 |
| 视觉 | 从图像中提取信息，学习颜色、二维码、AprilTag、文字和目标识别 | 已收录 |
| 点云 | 处理三维空间数据，学习读写、滤波、邻域搜索和聚类 | 已收录 |

按自己感兴趣的问题选择即可：想研究“怎么绕开障碍”选避障，想研究“图像里有什么”选视觉，想研究“怎么对准并穿过去”选穿门，想研究“三维环境数据怎么处理”选点云。穿门材料尚未提供，准备选择这一方向的同学可以先完成共同关卡。

## 开发环境

**培训任务统一在提供的镜像环境中开发。** 先下载并解压 [第一关环境镜像：2026_tutorial_iso.zip](http://202.114.212.116:54321/directlink/1/2026_tutorial_iso.zip)，按第一关的环境操作文档准备好系统，再在该系统内克隆仓库、编译和完成练习。

第一关提供的环境以 Ubuntu 20.04、ROS Noetic、PX4 和 Gazebo 为基础。第一关基础通信任务使用简化后端；第二关及后续仿真任务再使用飞控与仿真组件。各任务要求补全的代码仍需自己完成，提供镜像不代表练习源码已补全。

## 关卡目录

| 关卡 | 内容 | 当前状态 |
| --- | --- | --- |
| [第 1 关](1关/README.md) | 环境搭建与基础 ROS 通信 | 完整 |
| [第 2 关](2关/README.md) | 阅读 template、理解控制流并实现绕圈 | 完整 |
| [第 3 关](3关/) | 分方向补齐关键代码 | 点云、视觉、避障 |
| [第 4 关](4关/) | 分方向完成小场景任务 | 视觉、避障、与点云 |
| [第 5 关](5关/) | 组队完成多方向融合项目 | 完整 |

## 克隆后直接进入关卡

在镜像环境的 Linux 终端中克隆一次总仓库：

```zsh
cd ~
git clone https://github.com/HarryGit-HUST/airhust_tutorial2026_ws.git
cd ~/airhust_tutorial2026_ws
```

各任务已经保留独立工作空间结构。直接进入下表对应的目录编译、运行和修改代码，不需要再复制目录，也不要把总仓库放进其他工作空间的 `src`。

| 任务 | 工作空间（以下路径相对总仓库根目录） | 题目与讲义 |
| --- | --- | --- |
| 第一关 | `1关/airhust_tutorial26_ros_basics` | [基础通信任务](1关/airhust_tutorial26_ros_basics/README.md) |
| 第二关 | `2关/airhust_tutorial26_template` | [template 入门](2关/airhust_tutorial26_template/第二关-template入门.md)、[练习与绕圈任务](2关/airhust_tutorial26_template/第二关-练习与绕圈任务.md) |
| 点云方向 | `3关/airhust_tutorial26_pcl_ws` | [PCL 讲义](3关/airhust_tutorial26_pcl_ws/docs/index.md) |
| 视觉方向 | `3关/airhust_tutorial26_vision_lab` | [第三关](<3关/airhust_tutorial26_vision_lab/src/板块3 函数原型与静态调用/README.md>)、[第四关](<3关/airhust_tutorial26_vision_lab/src/板块4 仿真单元测试/README.md>) |
| 避障方向 | `3关/airhust_tutorial26_collision_avoidance` | [算法补全与仿真任务](3关/airhust_tutorial26_collision_avoidance/collision_avoidance.md) |

镜像已经配置好环境，终端统一使用 zsh，直接进入对应工作空间操作。

第一关：

```zsh
cd ~/airhust_tutorial2026_ws/1关/airhust_tutorial26_ros_basics
catkin_make
roslaunch drone_bringup mission.launch
```

先按题目修复编译与通信问题，再运行任务。

第二关：

```zsh
cd ~/airhust_tutorial2026_ws/2关/airhust_tutorial26_template
catkin_make
rosparam load src/template/config/template.yaml
rosrun template template
```

运行第二关任务前，按镜像中的方式启动仿真、飞控与 MAVROS。修改 C++ 后重新编译。运行说明以这里为准，收录的原讲义保持原文。

第三、第四关共用对应方向的工作空间，第四关直接继续运行该方向的仿真任务，不再复制一份。各工作空间分别编译，不在总仓库根目录执行 `catkin_make`。

<!-- 收录来源与固定版本，原始收录版本，已调整目录及文档路径：
第一关：https://github.com/consefact/tutor_26_misson1/tree/185f156e3566f2eba56168737e4cb652fdc6c26f
点云：https://github.com/RgerColdline/airhust_tutorial26_pcl_ws/tree/9cf903f916d30dba3e807ee398f124a7bfa5d038
视觉：https://github.com/dunantedage1/airhust_tutorial26_vision_lab/tree/b8c3286636602fb095630d06bc5b3acedddb306f
避障：https://github.com/yan233966/airhust_tutorial26_collision_avoidance/tree/b1b8bb52ab9327f8a82d456e3db9428fa309da81
-->
