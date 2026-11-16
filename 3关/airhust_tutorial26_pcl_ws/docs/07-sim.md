<details>
<summary>目录</summary>

- [README](../README.md)
  - [阶段三：先学会用人家的函数](index.md)
    - [3.1 点云在内存里怎么存](01-io.md)
    - [3.2 用 rviz 看](02-rviz.md)
    - [3.3 滤波](03-filter.md)
    - [3.4 找邻居](04-kdtree.md)
    - [3.5 聚簇](05-cluster.md)
    - [3.6 点云哪来的](06-cloud-register.md)
    - **3.7 进仿真 ←**
    - [3.8 扩展](08-extend.md)

</details>

[← 3.6 点云哪来的](06-cloud-register.md)　·　[目录](index.md)　·　[3.8 扩展 →](08-extend.md)

# 3.7 进仿真

## 目标

把前面几步串成一条在线流水线：订阅 `/cloud_registered`，每收到一帧就累积、裁区域、降采样、去噪、聚类，然后把染好色的结果发布到 `/pipeline_result`。程序一直跑，点云一直发，rviz 里能实时看到分簇。

这一节提供程序架子，四个环节要自己补。

## 程序结构

[`07_pipeline.cpp`](../src/tutorial_pcl_basic/src/07_pipeline.cpp) 里，点云和参数放在文件顶部，`cloudCallback` 是 `/cloud_registered` 的订阅回调：每来一帧，依次调用下面四个函数，染色后发布到 `/pipeline_result`。`main` 里创建发布器、订阅话题，最后调用 `ros::spin()`。

`crop` 用 `CropBox` 按盒子裁掉地面和范围外的点，盒子范围在文件顶部，更换仿真场景时按点云坐标调整这里。要补的四个函数在回调前面：

```cpp
void accumulate(const Cloud::ConstPtr& frame, Cloud& map);                     // TODO 1
void downsample(const Cloud::ConstPtr& cloud, Cloud& output, float leaf);      // TODO 2
void denoise(const Cloud::ConstPtr& cloud, Cloud& output, int mean_k, double stddev);  // TODO 3
void cluster(const Cloud::ConstPtr& cloud, std::vector<pcl::PointIndices>& clusters,
             double tolerance, int min_size);                                  // TODO 4
```

处理参数在源码顶部设置，修改后重新编译：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `kLeaf` | 0.10 | 体素边长，米 |
| `kMeanK` / `kStddev` | 20 / 1.0 | 统计去噪 |
| `kTolerance` | 0.20 | 聚类容差，米 |
| `kMinCluster` | 50 | 每簇最少点数 |

参数没有通用值。换一张地图、换一个场景，体素、去噪和聚类容差都要重新试：先看各步点数的变化，再看该保留的物体还在不在。

## 任务

1. 补全四个函数。
2. 先启动 `roscore`，用包里的假发布者验证链路：
   ```shell
   # 新终端先加载本工作空间环境。
   source ~/airhust_tutorial26_pcl_ws/devel/setup.bash
   rosrun tutorial_pcl_basic publish_demo.py
   ```
   假发布者只支持 `x/y/z/intensity` 四个 float32 字段的未压缩 binary PCD，默认房间数据符合这个格式；其他格式会明确报错。

   另开终端在本工作空间根目录执行 `./run 7`，再在 rviz 中添加 PointCloud2，订阅 `/pipeline_result`，Fixed Frame 设为 `world`。

   分色结果可以检查聚类效果，另外几步要分别查看：累计前后点数是否变化，降采样是否合并了密集点，去噪后离群点是否减少。假发布者反复发送同一份点云，主要用于验证消息和处理流程；验证累计时，还需要用仿真中不同位置的观测，检查先前看到的区域是否保留。
3. 确认各步处理结果后，停止假发布者，再启动仿真。

## 启动仿真

```shell
roslaunch simulation sim.launch
```

`sim.launch` 里可以改地图和飞机模型。启动后先检查 `/cloud_registered` 的发布节点、消息类型和坐标系，再运行处理程序。

## 运行

```shell
./run 7
```

程序每 10 帧打印一次点数、簇数和前 10 个簇的中心。“累计”是追加当前帧后、裁剪前的点数。处理完成后，用裁剪并降采样的结果更新地图，下一帧继续在这份地图上追加点，限制保留地图的范围和密度。

每步的结果都会单独发出来，方便看到底哪一步不对：

| 话题 | 内容 |
| --- | --- |
| `/pipeline_accumulated` | 累积后的点云 |
| `/pipeline_cropped` | 裁区域后 |
| `/pipeline_filtered` | 降采样后 |
| `/pipeline_denoised` | 去噪后 |
| `/pipeline_result` | 聚类、染色后的结果 |

多发布几份会略微增加开销，调试完只留 `/pipeline_result` 就行。

## 运行结果

更换仿真场景或参数后，点数和簇数都会变；飞行路径和收帧时机不同，每次跑的结果也会有出入。下面是用假发布者反复发送 `demo_room.pcd`、采用默认参数时的一组输出：

```text
第 10 帧: 累计 128129 点, 降采样 15729, 去噪 15472, 4 个簇
簇 0: 13487 点, 中心 (4.00566, 3.00347, 1.42541)
簇 1: 758 点, 中心 (2.61454, 3.11586, 0.739606)
簇 2: 746 点, 中心 (5.49821, 4.30116, 1.52209)
簇 3: 481 点, 中心 (5.49723, 1.50275, 1.49127)
第 20 帧: 累计 128129 点, 降采样 15729, 去噪 15472, 4 个簇
...
```

## 扩展

改参数（体素、去噪、聚类容差）各跑一次，比较簇的数量和大小。

---

[← 3.6 点云哪来的](06-cloud-register.md)　·　[目录](index.md)　·　[3.8 扩展 →](08-extend.md)
