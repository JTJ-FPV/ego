#include "ego/a_star.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ego
{

AStar::AStar(const Config &config, OccupancyQuery occupancy_query)
{
    configure(config, std::move(occupancy_query));
}

void AStar::configure(const Config &config, OccupancyQuery occupancy_query)
{
    if (config.dimension != 2 && config.dimension != 3)
        throw std::invalid_argument("ego: A* dimension must be 2 or 3");
    if (!std::isfinite(config.resolution) || config.resolution <= 0.0)
        throw std::invalid_argument("ego: map resolution must be positive");
    if (!std::isfinite(config.max_search_time_ms) || config.max_search_time_ms <= 0.0)
        throw std::invalid_argument("ego: A* time limit must be positive");
    if (!occupancy_query)
        throw std::invalid_argument("ego: occupancy query is empty");

    config_ = config;
    occupancy_query_ = std::move(occupancy_query);
    path_.clear();
}

void AStar::setHeight(double height)
{
    if (!std::isfinite(height))
        throw std::invalid_argument("ego: 2D planning height must be finite");
    config_.height = height;
}

std::size_t AStar::IndexHash::operator()(const Index &index) const noexcept
{
    std::size_t seed = std::hash<int>{}(index.x);
    seed ^= std::hash<int>{}(index.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<int>{}(index.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
}

AStar::Index AStar::positionToIndex(const Eigen::Vector3d &position) const
{
    const Eigen::Array3d scaled = (position - config_.origin).array() / config_.resolution;
    return {static_cast<int>(std::floor(scaled.x())),
            static_cast<int>(std::floor(scaled.y())),
            static_cast<int>(std::floor(scaled.z()))};
}

Eigen::Vector3d AStar::indexToPosition(const Index &index) const
{
    Eigen::Vector3d position =
        config_.origin +
        config_.resolution * Eigen::Vector3d(index.x + 0.5, index.y + 0.5, index.z + 0.5);
    if (config_.dimension == 2)
        position.z() = config_.height;
    return position;
}

double AStar::heuristic(const Index &from, const Index &to) const
{
    const double dx = std::abs(from.x - to.x);
    const double dy = std::abs(from.y - to.y);
    const double dz = config_.dimension == 2 ? 0.0 : std::abs(from.z - to.z);
    return 1.001 * std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool AStar::occupied(const Index &index) const
{
    return occupancy_query_(indexToPosition(index));
}

void AStar::buildPath(const std::shared_ptr<Node> &goal_node)
{
    path_.clear();
    for (auto node = goal_node; node; node = node->parent)
        path_.push_back(indexToPosition(node->index));
    std::reverse(path_.begin(), path_.end());
}

AStarResult AStar::search(const Eigen::Vector3d &start,
                          const Eigen::Vector3d &goal,
                          double &cost_time_ms)
{
    using Clock = std::chrono::steady_clock;
    const auto started_at = Clock::now();
    const auto elapsedMs = [&started_at]() {
        return std::chrono::duration<double, std::milli>(Clock::now() - started_at).count();
    };

    path_.clear();
    Index start_index = positionToIndex(start);
    Index goal_index = positionToIndex(goal);
    if (config_.dimension == 2)
    {
        const int height_index = positionToIndex(
            Eigen::Vector3d(config_.origin.x(), config_.origin.y(), config_.height)).z;
        start_index.z = height_index;
        goal_index.z = height_index;
    }

    if (occupied(start_index) || occupied(goal_index))
    {
        cost_time_ms = elapsedMs();
        return AStarResult::InvalidStartOrGoal;
    }

    std::unordered_map<Index, std::shared_ptr<Node>, IndexHash> nodes;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryGreater> open;
    std::size_t sequence = 0;

    auto start_node = std::make_shared<Node>();
    start_node->index = start_index;
    start_node->f = heuristic(start_index, goal_index);
    nodes.emplace(start_index, start_node);
    open.push({start_node->f, sequence++, start_node});

    while (!open.empty())
    {
        if (elapsedMs() > config_.max_search_time_ms)
        {
            cost_time_ms = elapsedMs();
            return AStarResult::Timeout;
        }

        const QueueEntry entry = open.top();
        open.pop();
        const auto current = entry.node;
        if (current->closed || std::abs(entry.f - current->f) > 1e-12)
            continue;

        if (current->index == goal_index)
        {
            buildPath(current);
            cost_time_ms = elapsedMs();
            return AStarResult::Success;
        }
        current->closed = true;

        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                const int z_min = config_.dimension == 2 ? 0 : -1;
                const int z_max = config_.dimension == 2 ? 0 : 1;
                for (int dz = z_min; dz <= z_max; ++dz)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;
                    const Index next{current->index.x + dx,
                                     current->index.y + dy,
                                     current->index.z + dz};
                    if (occupied(next))
                        continue;

                    const double tentative_g =
                        current->g + std::sqrt(double(dx * dx + dy * dy + dz * dz));
                    auto found = nodes.find(next);
                    std::shared_ptr<Node> neighbor;
                    if (found == nodes.end())
                    {
                        neighbor = std::make_shared<Node>();
                        neighbor->index = next;
                        neighbor->g = std::numeric_limits<double>::infinity();
                        nodes.emplace(next, neighbor);
                    }
                    else
                    {
                        neighbor = found->second;
                    }

                    if (neighbor->closed || tentative_g >= neighbor->g)
                        continue;
                    neighbor->g = tentative_g;
                    neighbor->f = tentative_g + heuristic(next, goal_index);
                    neighbor->parent = current;
                    open.push({neighbor->f, sequence++, neighbor});
                }
            }
        }
    }

    cost_time_ms = elapsedMs();
    return AStarResult::NoPath;
}

}  // namespace ego
