# ego

[English documentation](README.md)

`ego` 是从原 `minco_trajectory/rfs_ego` 拆出的纯 C++14 轨迹优化库。库内没有 ROS、PCL、消息、参数服务器或可视化依赖；核心库唯一的外部依赖是 Eigen3。

## 依赖

- CMake 3.16 或更高版本
- 支持 C++14 的编译器
- Eigen3 3.3 或更高版本
- Python 可视化脚本额外需要 Matplotlib

## 构建

```bash
cmake -S . -B build \
  -DEGO_BUILD_EXAMPLES=ON \
  -DEGO_BUILD_TESTS=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

## 地图接口

规划器只通过世界坐标占用查询访问地图：

```cpp
using OccupancyQuery = std::function<bool(const Eigen::Vector3d &position)>;
```

返回 `true` 表示占用，`false` 表示空闲。普通函数指针可直接传入；接 ROS 地图时通常传入捕获地图对象的 lambda：

```cpp
ego::Config config;
config.dimension = 3;
config.map_resolution = map_resolution;
config.map_origin = map_origin;

ego::Ego planner(config, [&map](const Eigen::Vector3d &point) {
    return map.isOccupied_global(point);
});
```

回调应把地图范围外的点视为占用，并在规划调用期间保证所捕获地图对象有效。若地图会被其他线程更新，锁或快照由接入层负责。

二维规划时，查询点的 `z` 始终为 `config.planning_height`。内部 A* 使用 `map_resolution` 和 `map_origin` 离散化世界坐标，因此这两个参数必须与实际地图一致。

## 配置与有效权重

`ego::Config` 当前包含以下实际参与计算的权重：

| 参数 | 作用 |
| --- | --- |
| `weight_time` | 总轨迹时间代价 |
| `weight_distance` | 采样点间距均匀性代价 |
| `weight_collision` | 硬碰撞惩罚 |
| `weight_soft_collision` | 障碍物软距离惩罚 |
| `weight_velocity` | 超速惩罚 |
| `weight_acceleration` | 超加速度惩罚 |
| `weight_jerk` | 超 jerk 惩罚 |

`obstacle_clearance` 和 `soft_obstacle_clearance` 分别配置硬、软障碍物安全距离，默认值为 `0.15` m 和 `0.5` m。

其他重要参数包括 `dimension`、`piece_length`、`map_resolution`、`map_origin`、`astar_max_time_ms`、`planning_height`、动力学上限和 `samples_per_piece`。

## ROS 与可视化适配

`setDebugPointsCallback` 输出名称和 `std::vector<Eigen::Vector3d>`，接入方可以在回调中转换成 `sensor_msgs::PointCloud2` 或 `visualization_msgs::Marker`：

```cpp
planner.setDebugPointsCallback(
    [&publisher](const std::string &name,
                 const std::vector<Eigen::Vector3d> &points) {
        publisher.publish(toRosPointCloud(name, points));
    });
```

ROS 订阅、参数读取、轨迹消息发布和 RViz 展示均应放在库外的适配节点中。

## 输出轨迹 CSV

优化成功后可以直接输出最终连续轨迹：

```cpp
planner.saveTrajectoryCsv("trajectory.csv", 0.005);
```

第二个参数是采样时间间隔，单位为秒。二维和三维使用统一列格式：

```text
t,x,y,z,vx,vy,vz,ax,ay,az,jx,jy,jz,occupied
```

二维轨迹的 `z` 为 `planning_height`，z 方向速度、加速度和 jerk 为零。`occupied` 来自用户提供的地图查询回调，可用于在可视化中标记碰撞点。

## 示例

- `examples/callback_example.cpp`：最小地图函数指针和调试回调接入。
- `examples/ego_2d_example.cpp`：二维圆形障碍绕障、连续碰撞检查和 CSV 输出。
- `examples/ego_3d_example.cpp`：三维球形障碍绕障、连续碰撞检查和 CSV 输出。

```bash
./build/ego_callback_example
./build/ego_2d_example
./build/ego_3d_example

# 指定 CSV 输出位置
./build/ego_2d_example /tmp/ego_trajectory_2d.csv
./build/ego_3d_example /tmp/ego_trajectory_3d.csv
```

## Python 可视化

交互式显示：

```bash
python3 scripts/plot_trajectory.py build/ego_trajectory.csv
python3 scripts/plot_trajectory.py build/ego_trajectory_3d.csv
```

无图形界面时保存图片：

```bash
python3 scripts/plot_trajectory.py build/ego_trajectory.csv \
  --save build/trajectory_2d.png --no-show

python3 scripts/plot_trajectory.py build/ego_trajectory_3d.csv \
  --save build/trajectory_3d.png --no-show
```

## 安装与引用

```bash
cmake --install build --prefix /your/install/prefix
```

其他 CMake 工程可以使用：

```cmake
find_package(ego REQUIRED)
target_link_libraries(your_target PRIVATE ego::ego)
```

安装后，Python 可视化脚本可作为命令运行：

```bash
ego_plot_trajectory trajectory.csv
```
