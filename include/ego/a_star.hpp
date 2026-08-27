#pragma once

#include <Eigen/Core>

#include <chrono>
#include <functional>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>

namespace ego
{

using OccupancyQuery = std::function<bool(const Eigen::Vector3d &)>;

enum class AStarResult
{
    Success,
    InvalidStartOrGoal,
    Timeout,
    NoPath
};

class AStar
{
public:
    struct Config
    {
        int dimension = 3;
        double resolution = 0.1;
        Eigen::Vector3d origin = Eigen::Vector3d::Zero();
        double height = 0.0;
        double max_search_time_ms = 20.0;
    };

    AStar() = default;
    AStar(const Config &config, OccupancyQuery occupancy_query);

    void configure(const Config &config, OccupancyQuery occupancy_query);
    void setHeight(double height);
    double height() const noexcept { return config_.height; }

    AStarResult search(const Eigen::Vector3d &start,
                       const Eigen::Vector3d &goal,
                       double &cost_time_ms);

    const std::vector<Eigen::Vector3d> &path() const noexcept { return path_; }

private:
    struct Index
    {
        int x = 0;
        int y = 0;
        int z = 0;

        bool operator==(const Index &rhs) const noexcept
        {
            return x == rhs.x && y == rhs.y && z == rhs.z;
        }
    };

    struct IndexHash
    {
        std::size_t operator()(const Index &index) const noexcept;
    };

    struct Node
    {
        Index index;
        double g = 0.0;
        double f = 0.0;
        bool closed = false;
        std::shared_ptr<Node> parent;
    };

    struct QueueEntry
    {
        double f = 0.0;
        std::size_t sequence = 0;
        std::shared_ptr<Node> node;
    };

    struct QueueEntryGreater
    {
        bool operator()(const QueueEntry &lhs, const QueueEntry &rhs) const noexcept
        {
            if (lhs.f == rhs.f)
                return lhs.sequence > rhs.sequence;
            return lhs.f > rhs.f;
        }
    };

    Index positionToIndex(const Eigen::Vector3d &position) const;
    Eigen::Vector3d indexToPosition(const Index &index) const;
    double heuristic(const Index &from, const Index &to) const;
    bool occupied(const Index &index) const;
    void buildPath(const std::shared_ptr<Node> &goal_node);

    Config config_;
    OccupancyQuery occupancy_query_;
    std::vector<Eigen::Vector3d> path_;
};

using AStarPtr = std::shared_ptr<AStar>;

}  // namespace ego
