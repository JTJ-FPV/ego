#include "ego/a_star.hpp"
#include "ego/ego.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

int query_count = 0;

bool freeMap(const Eigen::Vector3d &position)
{
    ++query_count;
    return std::abs(position.x()) > 10.0 || std::abs(position.y()) > 10.0;
}

void require(bool condition, const std::string &message)
{
    if (!condition)
        throw std::runtime_error(message);
}

}  // namespace

int main()
{
    try
    {
    ego::AStar::Config astar_config;
    astar_config.dimension = 2;
    astar_config.resolution = 0.25;
    astar_config.height = 1.25;
    astar_config.max_search_time_ms = 100.0;

    ego::AStar astar(astar_config, &freeMap);
    double search_time_ms = 0.0;
    const auto result = astar.search(Eigen::Vector3d(-1.0, -1.0, 1.25),
                                     Eigen::Vector3d(1.0, 1.0, 1.25),
                                     search_time_ms);
    require(result == ego::AStarResult::Success, "A* function-pointer query failed");
    require(!astar.path().empty(), "A* returned an empty path");
    require(query_count > 0, "the map callback was not invoked");
    for (const auto &point : astar.path())
        require(std::abs(point.z() - 1.25) < 1e-12, "2D planning height changed");
    std::cout << "[PASS] function pointer map query: path_nodes=" << astar.path().size()
              << ", queries=" << query_count
              << ", search_ms=" << search_time_ms << '\n';

    ego::Config ego_config;
    ego_config.dimension = 2;
    ego_config.map_resolution = 0.25;
    ego_config.planning_height = 1.25;
    ego::Ego planner(ego_config, &freeMap);
    require(planner.trajectory().getDimensions() == 2, "wrong trajectory dimension");

    Eigen::MatrixXd start_state = Eigen::MatrixXd::Zero(2, 3);
    Eigen::MatrixXd target_state = Eigen::MatrixXd::Zero(2, 3);
    start_state.col(0) << -1.0, -1.0;
    target_state.col(0) << 1.0, 1.0;
    Eigen::MatrixXd way_points(2, 1);
    way_points.col(0) << 0.0, 0.0;
    Eigen::VectorXd times(2);
    times << 1.0, 1.0;
    Eigen::MatrixXd optimal_points;
    Eigen::VectorXd optimal_times;
    double final_cost = 0.0;
    const bool optimized = planner.optimize(start_state, target_state, way_points, times,
                                            optimal_points, optimal_times, final_cost);
    require(optimized, "free-space optimization failed");
    require(optimal_points.rows() == 2 && optimal_points.cols() == 1,
            "free-space waypoint output has the wrong shape");
    require(optimal_times.size() == 2, "free-space time output has the wrong size");
    require(std::isfinite(final_cost), "free-space cost is not finite");
    const std::string csv_path = "ego_test_trajectory.csv";
    planner.saveTrajectoryCsv(csv_path, 0.01);
    std::ifstream csv(csv_path);
    std::string csv_header;
    std::string csv_first_sample;
    require(static_cast<bool>(std::getline(csv, csv_header)), "CSV header is missing");
    require(csv_header == "t,x,y,z,vx,vy,vz,ax,ay,az,jx,jy,jz,occupied",
            "CSV header is incorrect");
    require(static_cast<bool>(std::getline(csv, csv_first_sample)),
            "CSV trajectory samples are missing");
    std::cout << "[PASS] free-space EGO optimization: cost=" << final_cost
              << ", duration=" << planner.trajectory().getTotalDuration()
              << " s, csv=" << csv_path << '\n';

    ego::Config obstacle_config = ego_config;
    obstacle_config.map_resolution = 0.1;
    obstacle_config.astar_max_time_ms = 100.0;
    obstacle_config.weight_collision = 10000.0;
    obstacle_config.weight_soft_collision = 5000.0;
    obstacle_config.samples_per_piece = 5;
    obstacle_config.max_velocity = 3.0;
    obstacle_config.max_acceleration = 3.0;
    obstacle_config.max_jerk = 10.0;
    const double obstacle_radius = 0.3;
    const auto obstacle_query = [obstacle_radius](const Eigen::Vector3d &point) {
        const bool outside = std::abs(point.x()) > 5.0 || std::abs(point.y()) > 5.0;
        return outside || point.head<2>().norm() < obstacle_radius;
    };
    ego::Ego obstacle_planner(obstacle_config, obstacle_query);
    std::size_t debug_astar_points = 0;
    obstacle_planner.setDebugPointsCallback(
        [&debug_astar_points](const std::string &name,
                              const std::vector<Eigen::Vector3d> &points) {
            if (name == "a_star_path")
                debug_astar_points += points.size();
        });

    start_state.col(0) << -1.0, 0.0;
    target_state.col(0) << 1.0, 0.0;
    way_points.resize(2, 3);
    way_points << -0.5, 0.0, 0.5,
                   0.0, 0.0, 0.0;
    times.resize(4);
    times.setConstant(0.5);
    const bool avoided_obstacle = obstacle_planner.optimize(
        start_state, target_state, way_points, times,
        optimal_points, optimal_times, final_cost);
    require(avoided_obstacle, "obstacle-avoidance optimization failed");
    require(std::isfinite(final_cost), "obstacle-avoidance cost is not finite");
    require(debug_astar_points > 0, "A* debug callback did not receive any points");

    double minimum_clearance = std::numeric_limits<double>::infinity();
    const double duration = obstacle_planner.trajectory().getTotalDuration();
    for (double t = 0.0; t <= duration; t += 0.005)
    {
        const Eigen::VectorXd state = obstacle_planner.trajectory().getState(t);
        const Eigen::Vector3d point(state.x(), state.y(), obstacle_config.planning_height);
        minimum_clearance = std::min(minimum_clearance, point.head<2>().norm());
        require(!obstacle_query(point), "optimized continuous trajectory still intersects the map");
    }
    std::cout << "[PASS] captured-lambda obstacle query: cost=" << final_cost
              << ", minimum_clearance=" << minimum_clearance
              << " m, debug_a_star_points=" << debug_astar_points << '\n';
    std::cout << "All standalone ego tests passed.\n";
    return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "[FAIL] " << error.what() << '\n';
        return 1;
    }
}
