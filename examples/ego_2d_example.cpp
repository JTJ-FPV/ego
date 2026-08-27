#include "ego/ego.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

int main(int argc, char **argv)
{
    const std::string output_path = argc > 1 ? argv[1] : "ego_trajectory.csv";
    constexpr double obstacle_radius = 0.3;

    // World-coordinate occupancy query. A ROS map can be called inside this lambda.
    const auto occupancy_query = [](const Eigen::Vector3d &point) {
        const bool outside_map = std::abs(point.x()) > 5.0 ||
                                 std::abs(point.y()) > 5.0;
        const bool inside_circle = point.head<2>().norm() < obstacle_radius;
        return outside_map || inside_circle;
    };

    ego::Config config;
    config.dimension = 2;
    config.map_resolution = 0.1;
    config.map_origin = Eigen::Vector3d::Zero();
    config.planning_height = 1.0;
    config.astar_max_time_ms = 100.0;
    config.weight_collision = 10000.0;
    config.weight_soft_collision = 5000.0;
    config.obstacle_clearance = 0.15;
    config.soft_obstacle_clearance = 0.5;
    config.samples_per_piece = 5;
    config.max_velocity = 3.0;
    config.max_acceleration = 3.0;
    config.max_jerk = 10.0;

    ego::Ego planner(config, occupancy_query);
    planner.setDebugPointsCallback(
        [](const std::string &name, const std::vector<Eigen::Vector3d> &points) {
            std::cout << "debug " << name << ": " << points.size() << " points\n";
        });

    // State columns are position, velocity, and acceleration.
    Eigen::MatrixXd start_state = Eigen::MatrixXd::Zero(2, 3);
    Eigen::MatrixXd target_state = Eigen::MatrixXd::Zero(2, 3);
    start_state.col(0) << -1.0, 0.0;
    target_state.col(0) << 1.0, 0.0;

    // The initial waypoints cross the circle to exercise A* and EGO avoidance.
    Eigen::MatrixXd initial_waypoints(2, 3);
    initial_waypoints << -0.5, 0.0, 0.5,
                          0.0, 0.0, 0.0;
    Eigen::VectorXd initial_times = Eigen::VectorXd::Constant(4, 0.5);

    Eigen::MatrixXd optimal_waypoints;
    Eigen::VectorXd optimal_times;
    double final_cost = 0.0;
    const bool success = planner.optimize(
        start_state, target_state, initial_waypoints, initial_times,
        optimal_waypoints, optimal_times, final_cost);
    if (!success)
    {
        std::cerr << "EGO optimization failed\n";
        return 1;
    }

    constexpr double sample_dt = 0.005;
    try
    {
        planner.saveTrajectoryCsv(output_path, sample_dt);
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 2;
    }

    const auto &trajectory = planner.trajectory();
    const double duration = trajectory.getTotalDuration();
    double minimum_clearance = std::numeric_limits<double>::infinity();
    bool collision_free = true;

    for (double t = 0.0; t < duration + 0.5 * sample_dt; t += sample_dt)
    {
        const double sample_time = std::min(t, duration);
        const Eigen::VectorXd position = trajectory.getState(sample_time, 0);
        const Eigen::Vector3d world_position(
            position.x(), position.y(), config.planning_height);
        const bool occupied = occupancy_query(world_position);
        collision_free = collision_free && !occupied;
        minimum_clearance = std::min(minimum_clearance, position.norm());

    }

    std::cout << "optimization succeeded\n"
              << "cost: " << final_cost << '\n'
              << "duration: " << duration << " s\n"
              << "minimum obstacle-center distance: " << minimum_clearance << " m\n"
              << "continuous collision check: "
              << (collision_free ? "PASS" : "FAIL") << '\n'
              << "trajectory CSV: " << output_path << '\n'
              << "optimal waypoints:\n" << optimal_waypoints << '\n'
              << "optimal piece times:\n" << optimal_times.transpose() << '\n';

    return collision_free ? 0 : 3;
}
