`# airhust_tutorial26_vision_lab`

2026 级梯队培训 · 视觉方向（板块三 视觉基础、板块四 仿真单元测试）

**环境**：Ubuntu 20.04 + ROS Noetic + PX4 SITL + Gazebo；

**先把仓库放在 `~/airhust_tutorial26_vision_lab`,第三关和第四关都在这个仓库下**

```bash
mv ~/airhust_tutorial2026_ws/3关/airhust_tutorial26_vision_lab ~/
```

**板块三（离线，五个完形填空）**

```bash
cd ~/airhust_tutorial26_vision_lab/src/lab_vision_sim_task3
pip install -r requirements.txt
python scripts/01_color_detect.py     # 01~05 依次做
```

说明看 `src/板块3 函数原型与静态调用/README.md`

**板块四（仿真，五关）**

```bash
cd ~/airhust_tutorial26_vision_lab
catkin_make
roslaunch lab_vision_sim_task4 task1.launch   # task1~task5
```

说明看 `src/板块4 仿真单元测试/README.md`；

**目录**

```
src/
├── lab_vision_sim_task3/   视觉代码：scripts、assets、models/best.pt
├── lab_vision_sim_task4/   ROS 包：launch、worlds、四张靶标
├── 板块3 函数原型与静态调用/README.md
└── 板块4 仿真单元测试/README.md
```
