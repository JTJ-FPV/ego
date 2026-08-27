# ego

[中文文档](README.zh-CN.md)

`ego` is a standalone C++14 trajectory optimization library extracted from the original `minco_trajectory/rfs_ego` implementation. The library has no dependency on ROS, PCL, ROS messages, the parameter server, or visualization frameworks. Eigen3 is its only external core dependency.

## Requirements

- CMake 3.16 or newer
- A C++14 compiler
- Eigen3 3.3 or newer
- Matplotlib for the optional Python visualization script

## Build

```bash
cmake -S . -B build \
  -DEGO_BUILD_EXAMPLES=ON \
  -DEGO_BUILD_TESTS=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

## Map interface

The planner accesses the map exclusively through a world-coordinate occupancy query:

```cpp
using OccupancyQuery = std::function<bool(const Eigen::Vector3d &position)>;
```

The callback returns `true` for occupied space and `false` for free space. A plain function pointer can be passed directly. A ROS integration typically uses a lambda that captures the map object:

```cpp
ego::Config config;
config.dimension = 3;
config.map_resolution = map_resolution;
config.map_origin = map_origin;

ego::Ego planner(config, [&map](const Eigen::Vector3d &point) {
    return map.isOccupied_global(point);
});
```

The callback should treat positions outside the map as occupied and keep captured objects alive while planning. If another thread updates the map, synchronization or snapshotting is the responsibility of the adapter.

For 2D planning, every query uses `config.planning_height` as its z coordinate. Internal A* discretization uses `map_resolution` and `map_origin`; both values must match the supplied map.

## Configuration and active weights

The following `ego::Config` weights participate in an actual cost or gradient calculation:

| Parameter | Purpose |
| --- | --- |
| `weight_time` | Total trajectory duration cost |
| `weight_distance` | Sample-spacing regularization |
| `weight_collision` | Hard collision penalty |
| `weight_soft_collision` | Soft obstacle-clearance penalty |
| `weight_velocity` | Velocity-limit violation penalty |
| `weight_acceleration` | Acceleration-limit violation penalty |
| `weight_jerk` | Jerk-limit violation penalty |

The legacy vertical-parallax, horizontal-parallax, and maximum-yaw weights were never used by a cost or gradient and have been removed from both the public configuration and the implementation.

Other important fields include `dimension`, `piece_length`, `map_resolution`, `map_origin`, `astar_max_time_ms`, `planning_height`, the dynamic limits, and `samples_per_piece`.

## ROS and visualization adapters

`setDebugPointsCallback` exposes a name and a `std::vector<Eigen::Vector3d>`. An adapter can convert these values to `sensor_msgs::PointCloud2` or `visualization_msgs::Marker`:

```cpp
planner.setDebugPointsCallback(
    [&publisher](const std::string &name,
                 const std::vector<Eigen::Vector3d> &points) {
        publisher.publish(toRosPointCloud(name, points));
    });
```

Keep ROS subscriptions, parameter loading, trajectory message publication, and RViz visualization outside this library.

## Export a trajectory to CSV

After successful optimization, sample and export the final continuous trajectory with:

```cpp
planner.saveTrajectoryCsv("trajectory.csv", 0.005);
```

The second argument is the sampling interval in seconds. Both 2D and 3D trajectories use the same columns:

```text
t,x,y,z,vx,vy,vz,ax,ay,az,jx,jy,jz,occupied
```

For a 2D trajectory, `z` equals `planning_height`, while z velocity, acceleration, and jerk are zero. `occupied` is evaluated through the user-provided map callback and can highlight collision samples during visualization.

## Examples

- `examples/callback_example.cpp`: minimal function-pointer and debug-callback integration.
- `examples/ego_2d_example.cpp`: 2D circular-obstacle avoidance, continuous collision checking, and CSV output.
- `examples/ego_3d_example.cpp`: 3D spherical-obstacle avoidance, continuous collision checking, and CSV output.

```bash
./build/ego_callback_example
./build/ego_2d_example
./build/ego_3d_example

# Select an output CSV path.
./build/ego_2d_example /tmp/ego_trajectory_2d.csv
./build/ego_3d_example /tmp/ego_trajectory_3d.csv
```

## Python visualization

Open an interactive plot:

```bash
python3 scripts/plot_trajectory.py build/ego_trajectory.csv
python3 scripts/plot_trajectory.py build/ego_trajectory_3d.csv
```

Save a figure without opening a display:

```bash
python3 scripts/plot_trajectory.py build/ego_trajectory.csv \
  --save build/trajectory_2d.png --no-show

python3 scripts/plot_trajectory.py build/ego_trajectory_3d.csv \
  --save build/trajectory_3d.png --no-show
```

## Install and consume

```bash
cmake --install build --prefix /your/install/prefix
```

Use the installed package from another CMake project:

```cmake
find_package(ego REQUIRED)
target_link_libraries(your_target PRIVATE ego::ego)
```

The installed visualization command is:

```bash
ego_plot_trajectory trajectory.csv
```
