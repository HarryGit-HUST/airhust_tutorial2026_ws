<details>
<summary>目录</summary>

- [README](../README.md)
  - [阶段三：先学会用人家的函数](index.md)
    - [3.1 点云在内存里怎么存](01-io.md)
    - [3.2 用 rviz 看](02-rviz.md)
    - **3.3 滤波 ←**
    - [3.4 找邻居](04-kdtree.md)
    - [3.5 聚簇](05-cluster.md)
    - [3.6 点云哪来的](06-cloud-register.md)
    - [3.7 进仿真](07-sim.md)
    - [3.8 扩展](08-extend.md)

</details>

[目录](index.md)　·　[3.4 找邻居 →](04-kdtree.md)

# 3.3 滤波

## 问题

`data/demo_room.pcd` 是脚本造的，每个面都是均匀网格，没有多余的点。

真实雷达扫出来是另一回事，常见两个问题：

- **冗余**：飞机悬停时，同一块墙面被扫了几百遍，点挤在一起。一平米几千个点，信息没有增加，但后面每一步都要逐点算邻居、算距离，点数增加也会增加计算量。
- **噪声**：激光打到灰尘、桨叶、玻璃，或者打到远处又反射回来，会产生不属于任何物体的孤立点。它们会干扰平面拟合和聚类。

`data/demo_room_noisy.pcd` 就是在 `demo_room.pcd` 上加了这两样：每个点复制三份、各加 2mm 抖动，再撒 4000 个随机点和 800 个从雷达位置往外散的离群点。

用 `pcl_viewer` 对比（在仓库根目录执行）：

```shell
pcl_viewer data/demo_room.pcd
pcl_viewer data/demo_room_noisy.pcd
pcl_viewer data/demo_room.pcd data/demo_room_noisy.pcd   # 叠着看，两色
```

## 怎么解决

### 冗余：每个格子用重心表示

同一块墙面被反复扫，点彼此只差几毫米，测出来是同一个位置，这就是冗余。按空间切成格子，每格计算一个重心作为代表点，就把这些重复点合成一个。

格子边长越大，输出点通常越少，小零件和边缘也越容易被合并；边长越小，细节保留得越多，计算量和内存占用也会增加。可以先用 5cm，再试试 1~10cm。这个范围只适用于这里的米制房间数据，实际取值要结合目标尺寸和点间距。

这就是 `pcl::VoxelGrid`（[查看官网 API](https://pointclouds.org/documentation/classpcl_1_1_voxel_grid.html)），一个模板类，尖括号里填点类型：

```cpp
// pcl 命名空间内的类声明，省略实现
template <typename PointT>
class VoxelGrid : public Filter<PointT>;

// 本节用到的构造函数和成员函数
VoxelGrid ();                                      // 构造函数
void setInputCloud (const PointCloudConstPtr &cloud);
void setLeafSize (float lx, float ly, float lz);   // 格子边长，米
void filter (PointCloud &output);
```

下面用三维坐标点演示调用顺序，`sample` 是输入点云指针，`reduced` 是已有的输出容器：

```cpp
pcl::VoxelGrid<pcl::PointXYZ> grid;
grid.setInputCloud(sample);
grid.setLeafSize(0.1f, 0.1f, 0.1f);
grid.filter(reduced);
```

`Filter<PointT>` 提供 `setInputCloud` 和 `filter`；`FilterIndices<PointT>` 继承它，另外提供 `setIndices`、`setNegative` 这类按下标处理的接口。所以四个滤波类的调用方式一样。

后面的滤波类也按“设置输入、设置参数、计算输出”的顺序调用。

它把空间切成 lx×ly×lz 的格子，输出每格内所有点的重心。同一表面反复扫，点仍落进原来的格子，总点数不再增加；只有新的噪声点和新的观测区域会占用新格子。

### 噪声：看邻居的距离

离群点周围没有别的点，它到最近 k 个邻居的平均距离会明显大于表面上的点。统计每个点的这个距离，超过阈值的删掉。

`pcl::StatisticalOutlierRemoval`（[查看官网 API](https://pointclouds.org/documentation/classpcl_1_1_statistical_outlier_removal.html)）：

```cpp
// pcl 命名空间内的类声明，省略实现
template <typename PointT>
class StatisticalOutlierRemoval : public FilterIndices<PointT>;

// 本节用到的构造函数和成员函数
StatisticalOutlierRemoval ();            // 构造函数
void setInputCloud (const PointCloudConstPtr &cloud);
void setMeanK (int mean_k);              // 看多少个邻居
void setStddevMulThresh (double stddev); // 超过均值多少个标准差算异常
void filter (PointCloud &output);
```

参数的设置方式如下，输入和输出沿用前面的滤波调用方式：

```cpp
pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
sor.setMeanK(neighbor_count);
sor.setStddevMulThresh(multiplier);
```

`meanK` 是算平均距离时参考几个邻居：取小了，每个点的平均距离本身波动大，容易误判；取大了算得慢。一般 10~50，这里用 20。

`stddev` 决定离群阈值比平均距离宽几个标准差：取小了删得多，取大了删得少，一般 0.5~3.0，这里用 1.0。

### 无关区域：按范围切掉

如果任务只处理桌子附近的物体，就可以先排除地面、天花板和远处墙面。需要保留哪些区域由任务决定；地面和墙在定位或避障中仍可能有用。

`pcl::PassThrough`（[查看官网 API](https://pointclouds.org/documentation/classpcl_1_1_pass_through.html)）留某个字段在范围内的点：

```cpp
// pcl 命名空间内的类声明，省略实现
template <typename PointT>
class PassThrough : public FilterIndices<PointT>;

// 本节用到的构造函数和成员函数
PassThrough ();                          // 构造函数
void setInputCloud (const PointCloudConstPtr &cloud);
void setFilterFieldName (const std::string &field_name);  // "z"
void setFilterLimits (float min, float max);
void filter (PointCloud &output);
```

用的时候：

```cpp
pcl::PassThrough<pcl::PointXYZI> pass;
pass.setInputCloud(cloud);
pass.setFilterFieldName("z");
pass.setFilterLimits(0.2, 2.4);
pass.filter(output);
```

`pcl::CropBox`（[查看官网 API](https://pointclouds.org/documentation/classpcl_1_1_crop_box.html)）留一个盒子里的点：

```cpp
// pcl 命名空间内的类声明，省略实现
template <typename PointT>
class CropBox : public FilterIndices<PointT>;

// 本节用到的构造函数和成员函数
CropBox ();                              // 构造函数
void setInputCloud (const PointCloudConstPtr &cloud);
void setMin (const Eigen::Vector4f &min_pt);
void setMax (const Eigen::Vector4f &max_pt);
void filter (PointCloud &output);
```

用两个向量设置盒子的最小、最大边界：

```cpp
pcl::CropBox<pcl::PointXYZI> crop;
crop.setMin(min_corner);
crop.setMax(max_corner);
```

`min_corner` 和 `max_corner` 是 `Eigen::Vector4f`，前三个分量分别是 x、y、z 边界。

只按 x、y、z 三个方向各切一刀时，`CropBox` 和 `PassThrough` 连做三次结果一样，区别是 `CropBox` 一趟扫完、还能带旋转（`setTransform`）。但两者并不等价：`PassThrough` 能按任意字段过滤（比如 `intensity`），`CropBox` 只能按位置。

本节 demo 数据的范围是按房间坐标系给出的。实际雷达话题可能在雷达系，也可能已转换到世界系，轴方向需要查看驱动或 TF 定义后再确定。`Vector4f` 第四个分量固定填 `1.0`（齐次坐标），不是长度。

## 封装成函数

三个处理函数都接收输入点云，把结果写入输出容器：

```cpp
using Cloud = pcl::PointCloud<pcl::PointXYZI>;

void readCloud(const std::string& path, Cloud& output);
void downsample(const Cloud::ConstPtr& cloud, Cloud& output, float leaf);
void denoise(const Cloud::ConstPtr& cloud, Cloud& output, int mean_k, double stddev);
void region(const Cloud::ConstPtr& cloud, Cloud& output);
void saveCloud(const Cloud& cloud, const std::string& file);
```

`Cloud::ConstPtr` 是只读点云指针，方便直接传给 PCL 的 `setInputCloud`；`Cloud& output` 是要写入的点云。传指针或引用都不会在传参时复制整张点云。输入和输出使用不同的容器。

先创建容器，再按流程调用：

```cpp
auto raw = pcl::make_shared<Cloud>();
auto sampled = pcl::make_shared<Cloud>();
auto denoised = pcl::make_shared<Cloud>();
auto objects = pcl::make_shared<Cloud>();

readCloud(path, *raw);
downsample(raw, *sampled, 0.05f);
denoise(sampled, *denoised, 20, 1.0);
region(denoised, *objects);
saveCloud(*objects, "03_region.pcd");
```

处理函数内部设置输入和参数，然后把结果写进 `output`，例如 `filter(output)`。三个 `TODO` 分别在三个处理函数里，不用自己遍历点。`region` 可以用 `CropBox`，也可以依次在 x、y、z 上做 `PassThrough`。

### 连续处理时

本节先处理一个文件。实际接收雷达数据时，容器在接收循环之前创建，每来一帧就重复处理：

```cpp
// 接收下一帧，写入 output；停止接收时返回 false。这里只给签名。
bool receiveCloud(Cloud& output);

// raw、sampled、denoised、objects 已在循环外创建。
while (receiveCloud(*raw)) {
  downsample(raw, *sampled, 0.05f);
  denoise(sampled, *denoised, 20, 1.0);
  region(denoised, *objects);
  // 把 objects 交给后续处理或发布函数。
}
```

每次更新容器中的点，不累加上一帧。点数增加时，容器仍可能重新分配内存。在 ROS 中，这几步通常放在订阅回调里；上面的 `receiveCloud` 是流程示例，本节不用实现。

## 任务

[`src/03_filter.cpp`](../src/tutorial_pcl_basic/src/03_filter.cpp) 的 `main` 已经按“读点云 → 降采样 → 去噪 → 取关心的区域 → 存点云”排好，中间三步各是一个函数，三个 `TODO` 都在函数体里。

参数可以自己调整。`main` 中的 `0.05`、`20`、`1.0` 和区域范围供参考，下面的结果对应这组参数，点数不必一样。

运行并查看结果：

```shell
./run 3
pcl_viewer data/demo_room_noisy.pcd out/03_denoised.pcd   # 叠着看处理前后
```

## 运行结果

下面是 leaf 0.05、meanK 20、stddev 1.0 时的结果。参数不同，点数也会不同。未补全的代码会把输入原样拷贝到输出，各步点数不变。输出格式随意，能看出各步点数就行。

```
原始点数        : 342000
体素降采样后    : 91064
统计离群点去除后: 90311
区域内点数      : 3987
```

`pcl_viewer` 叠着看：远处散在空中的点没了，墙面和桌面还在。

`out/03_region.pcd` 是框出来的桌子和桌上的零件。

换成原始数据 `./run 3 data/demo_room.pcd`，同一组参数下是 112400 → 70003 → 69733 → 3846。

换一份数据后，需要重新调参数：先看点数变化，再看需要保留的物体是否还在。

## 扩展

多试几组参数，对比处理后的点云。

再想一个问题：把体素边长从 5cm 改成 10cm 或 2cm，合适的 `meanK`、`stddev` 还是同一组吗？

---

[← 3.2 rviz](02-rviz.md)　·　[目录](index.md)　·　[3.4 找邻居 →](04-kdtree.md)
