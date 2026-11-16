<details>
<summary>目录</summary>

- [README](../README.md)
  - [阶段三：先学会用人家的函数](index.md)
    - [3.1 点云在内存里怎么存](01-io.md)
    - [3.2 用 rviz 看](02-rviz.md)
    - [3.3 滤波](03-filter.md)
    - [3.4 找邻居](04-kdtree.md)
    - **3.5 聚簇 ←**
    - [3.6 点云哪来的](06-cloud-register.md)
    - [3.7 进仿真](07-sim.md)
    - [3.8 扩展](08-extend.md)

</details>

[← 3.4 找邻居](04-kdtree.md)　·　[目录](index.md)　·　[3.6 点云哪来的 →](06-cloud-register.md)

# 3.5 聚簇

## 目标

先看一眼现在这份点云：

```shell
pcl_viewer data/demo_room.pcd
```

整张点云使用同一种颜色，桌子、柱子和墙面的点都显示在一起。

这一节按空间连接关系给点分组，每组染上一种颜色，再保存成彩色点云，方便看清各组包含哪些点。

## 哪些点属于同一个物体

地面、天花板、外墙会把本来无关的东西连成一片：桌腿和地面的点挨得很近，聚类时会被并进同一堆。实际做的时候，往往先用 `CropBox` 把地面和关心范围外的点裁掉，少掉这些干扰。

裁完之后，桌子、柱子和桌上的零件仍可能混在一起。欧式聚类按空间的连接关系把它们分组：从一个点出发，找距离小于容差的邻居，再继续找邻居的邻居，直到没有新的点。

因此，一簇中的首尾两点可以相距很远，只要中间有点逐步相连。桌子和桌上的零件接触时，也可能被分到同一簇。算法判断的是连接关系。

## 裁掉地面

本节用 3.3 的 `CropBox` 裁一块：`x∈[1.6,6.6]、y∈[1.0,5.0]、z∈[0.2,2.4]`，单位为米。`z` 的下限切掉地面，上限切掉天花板和横梁。范围缩小可能截断目标，范围扩大可能重新包含墙面；换数据时应先检查包围盒和坐标系，再调整范围。

## 调用聚类类

`pcl::EuclideanClusterExtraction` 是欧式聚类的类，[查看官网 API](https://pointclouds.org/documentation/classpcl_1_1_euclidean_cluster_extraction.html)。头文件是 `<pcl/segmentation/extract_clusters.h>`。

类声明简写：

```cpp
// pcl 命名空间内，省略实现
template <typename PointT>
class EuclideanClusterExtraction : public PCLBase<PointT>;
```

`PointT` 是输入点类型。

主要成员函数如下：

```cpp
void setInputCloud(const Cloud::ConstPtr& cloud);
void setSearchMethod(const Search::Ptr& tree);   // 搜索方法
void setClusterTolerance(double tolerance);
void setMinClusterSize(int min_size);
void setMaxClusterSize(int max_size);
void extract(std::vector<pcl::PointIndices>& clusters);
```

`pcl::search` 命名空间里提供了几种搜索方法，各自适合什么数据，[查看官网 API](https://pointclouds.org/documentation/namespacepcl_1_1search.html)。

头文件里参数写作 `const KdTreePtr&`，实际是 `pcl::search::Search<PointT>::Ptr`，旧名字而已。传 `pcl::search::KdTree`、`OrganizedNeighbor`、`BruteForce` 这类继承 `Search` 的就行，`pcl::KdTreeFLANN` 属于老体系，不能直接传。头文件可以一次 include `<pcl/search/pcl_search.h>`，它带进 `Search`、`KdTree`、`Octree`、`OrganizedNeighbor`；`BruteForce` 这些还是要单独 include。

怎么建、怎么查邻居，见 [3.4 找邻居](04-kdtree.md)。

先构造聚类对象，最后调用 `extract`：

```cpp
pcl::EuclideanClusterExtraction<pcl::PointXYZI> extraction;
extraction.setInputCloud(cloud);
// 设置搜索方法、容差与点数范围后：
extraction.extract(clusters);
```

搜索方法用的点云要和输入点云是同一份。输出的每个 `PointIndices` 是一簇点的下标，后面会专门讲。

下面的参数供参考，可以自己调整：

| 参数 | 本节取值 | 调整时观察什么 |
| --- | --- | --- |
| 连接容差 | 0.12 m | 可比较 0.02、0.12、0.5 m；过小会使稀疏表面分裂，过大会连接相邻物体 |
| 最少点数 | 50 | 调高会排除小簇，也可能排除远处目标；调低会保留更多噪声簇 |
| 最多点数 | 10000 | 降采样后最大的一堆是 3974 点，留 2 倍左右余量；调小可能排除完整的大物体 |

点数阈值依赖数据密度和降采样设置，没有跨数据集通用的点数范围。超过最大点数的簇会被排除，不会被自动拆成小簇。

## 下标

`pcl::PointIndices` 就是一组整数，等价于 `std::vector<int>`。它存的是每个点在 `cloud->points` 里的索引。

```cpp
pcl::PointIndices one = clusters[0];
for (int idx : one.indices) {
  const auto& p = cloud->points[idx];   // 按下标取点
  std::cout << p.x << ' ' << p.y << ' ' << p.z << '\n';
}
```

想按下标拼出一张子点云，可以直接用现成的：

```cpp
pcl::PointCloud<pcl::PointXYZI> sub;
pcl::copyPointCloud(*cloud, clusters[0].indices, sub);
```

也可以按下标逐点 `push_back`。本节的 `clusterCenter` 直接按下标算中心，`describeClusters` 负责打印。3.8 扩展里找平面用的 `pcl::ExtractIndices` 也是按下标把内点、外点分开。

很多算法还提供 `setIndices`，用来指定待处理的点。比如只判断第一堆中的点是否为离群点：

```cpp
pcl::PointIndices::Ptr subset(new pcl::PointIndices);
subset->indices = clusters[0].indices;

pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
sor.setInputCloud(cloud);
sor.setIndices(subset);
sor.filter(out);
```

这里省掉了拷贝子点云的步骤。不过 SOR 查邻居时仍使用完整的输入点云，其他簇的点也可能成为邻居；如果只想在这一簇内部找邻居，先提取子点云，再把子点云设为输入。

存下标而不是直接拷贝点，省内存，也保证子集和原点云一一对应。但下标的参照系是**传进去的那张点云**：这里传的是降采样、裁剪之后的 `cropped`，拿这些下标去索引原始 `demo_room.pcd` 会错位。

## 算中心

一组点的质心是它们坐标的平均值。PCL 的 `compute3DCentroid` 可以接收整张点云，也可以通过下标指定参与计算的点。

本节使用的重载签名如下，头文件是 `<pcl/common/centroid.h>`：

```cpp
template <typename PointT, typename Scalar>
unsigned int compute3DCentroid(const pcl::PointCloud<PointT>& cloud,
                              const std::vector<int>& indices,
                              Eigen::Matrix<Scalar, 4, 1>& centroid);
```

结果写入四维向量：前三个分量是 x、y、z，第四个为 1。返回值是实际参与计算的有效点数。

需要补全的 `clusterCenter` 接收一簇点的下标，将中心写入 `centroid` 并返回点数。打印由 `describeClusters` 负责。

## 封装成函数

[`05_cluster.cpp`](../src/tutorial_pcl_basic/src/05_cluster.cpp) 的流程是：读点云 → 降采样 → 裁区域 → 聚类 → 染色保存。`downsample` 和 `region` 已经写好，作为 3.3 中滤波接口的基础用法示例；本节需要补全的是 `cluster` 和 `clusterCenter`。

```cpp
void downsample(const Cloud::ConstPtr& cloud, Cloud& output, float leaf);
void region(const Cloud::ConstPtr& cloud, Cloud& output);
void cluster(const Cloud::ConstPtr& cloud, std::vector<pcl::PointIndices>& clusters);
std::size_t clusterCenter(const Cloud::ConstPtr& cloud, const pcl::PointIndices& one,
                          Eigen::Vector4f& centroid);
void describeClusters(const Cloud::ConstPtr& cloud,
                      const std::vector<pcl::PointIndices>& clusters);
void colorClusters(const Cloud::ConstPtr& cloud,
                   const std::vector<pcl::PointIndices>& clusters,
                   pcl::PointCloud<pcl::PointXYZRGB>& colored);
```

`main` 里先降采样再裁区域：点少几倍，聚类时每次找邻居也快几倍。

```cpp
auto cloud = pcl::make_shared<Cloud>();
auto sampled = pcl::make_shared<Cloud>();
auto cropped = pcl::make_shared<Cloud>();

readCloud(path, *cloud);
downsample(cloud, *sampled, 0.05f);
region(sampled, *cropped);

std::vector<pcl::PointIndices> clusters;
cluster(cropped, clusters);
describeClusters(cropped, clusters);

pcl::PointCloud<pcl::PointXYZRGB> colored;
colorClusters(cropped, clusters, colored);
savePCDFileBinary(outPath("05_clusters.pcd"), colored);
```

## 任务

补全聚类函数和中心计算，运行：

```shell
./run 5
pcl_viewer out/05_clusters.pcd
```

程序会打印聚成几堆、每堆多少点、中心坐标（最多列 10 堆），并把每堆染上不同颜色的点云存到 `out/05_clusters.pcd`。没进任何一堆的点保持白色。

再改连接容差和体素边长各跑几次，看堆的数量和大小怎么变。

## 运行结果

使用原始房间数据时，一组输出示例如下：

```text
输入点数: 112400
降采样后点数: 70003
裁剪后点数: 7833
分出 3 堆
堆 0: 3974 点, 中心 (2.61153, 3.11056, 0.731429)
堆 1: 2397 点, 中心 (5.50085, 4.30166, 1.30253)
堆 2: 1364 点, 中心 (5.49915, 1.50049, 1.30763)
存到 out/05_clusters.pcd
```

`out/05_clusters.pcd` 里每堆一种颜色，桌子是红的、两根柱子是绿的和蓝的，没分进堆的点是白色。

### 怎么检查

检查桌子和柱子是否各自形成连续的簇，颜色是否按物体分开。中心坐标是这堆点的平均位置。

所有簇点数之和可以小于裁剪后的点数，因为不满足点数范围的簇被排除了。

## 思考题

- 为什么桌面上的零件可能与桌子属于同一簇？
- 体素尺寸增大后，最少点数是否还适合保持不变？

---

[← 3.4 找邻居](04-kdtree.md)　·　[目录](index.md)　·　[3.6 点云哪来的 →](06-cloud-register.md)
