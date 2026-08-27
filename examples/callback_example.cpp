#include "ego/ego.hpp"

#include <Eigen/Core>

#include <iostream>
#include <vector>

namespace
{

bool queryDemoMap(const Eigen::Vector3d &position)
{
    // Replace this body with a query against a grid map, ESDF, or ROS map object.
    const bool outside = position.x() < -5.0 || position.x() > 5.0 ||
                         position.y() < -5.0 || position.y() > 5.0;
    const bool in_obstacle = position.head<2>().squaredNorm() < 0.5 * 0.5;
    return outside || in_obstacle;
}

}  // namespace

int main()
{
    ego::Config config;
    config.dimension = 2;
    config.map_resolution = 0.1;
    config.planning_height = 1.0;

    // A plain function pointer is implicitly converted to std::function.
    ego::Ego planner(config, &queryDemoMap);

    // This callback exposes Eigen points only. A ROS adapter can publish markers here.
    planner.setDebugPointsCallback(
        [](const std::string &name, const std::vector<Eigen::Vector3d> &points) {
            std::cout << name << ": " << points.size() << " points\n";
        });

    std::cout << "ego configured without ROS\n";
    return 0;
}
