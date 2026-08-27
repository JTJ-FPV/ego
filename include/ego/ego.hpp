#pragma once

#include "ego/a_star.hpp"
#include "ego/polynomial_spline.hpp"

#include <Eigen/Geometry>

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace ego
{

struct Config
{
    int dimension = 3;
    double piece_length = 1.0;

    double map_resolution = 0.1;
    Eigen::Vector3d map_origin = Eigen::Vector3d::Zero();
    double astar_max_time_ms = 20.0;
    double planning_height = 0.0;

    double weight_time = 1.0;
    double weight_distance = 0.0;
    double weight_collision = 1.0;
    double weight_soft_collision = 0.0;
    double weight_velocity = 1.0;
    double weight_acceleration = 1.0;
    double weight_jerk = 1.0;

    double max_velocity = 3.0;
    double max_acceleration = 3.0;
    double max_jerk = 6.0;
    int samples_per_piece = 10;
};

using DebugPointsCallback =
    std::function<void(const std::string &, const std::vector<Eigen::Vector3d> &)>;

class Ego
{
public:
    enum class CheckResult
    {
        ObstacleFree,
        Error,
        ConstraintsCreated
    };

    Ego() = default;
    Ego(const Config &config, OccupancyQuery occupancy_query);

    void configure(const Config &config, OccupancyQuery occupancy_query);
    void setOccupancyQuery(OccupancyQuery occupancy_query);
    void setDebugPointsCallback(DebugPointsCallback callback);
    void setHeight(double height);

    bool optimize(const Eigen::MatrixXd &start_state,
                  const Eigen::MatrixXd &target_state,
                  const Eigen::MatrixXd &way_points,
                  const Eigen::VectorXd &real_time_vector,
                  Eigen::MatrixXd &optimal_points,
                  Eigen::VectorXd &optimal_times,
                  double &final_cost);

    const Spline::PolynomialSpline &trajectory() const;
    Spline::PolynomialSpline &trajectory();

    /**
     * Sample the current optimized trajectory and write it as CSV.
     * Columns: t, position, velocity, acceleration, jerk and occupied.
     */
    void saveTrajectoryCsv(const std::string &path,
                           double sample_interval = 0.01) const;

    void compute3DPose(
        std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> &pose,
        double dt);
    void compute2DPose(
        std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> &pose);

private:
    static double costFunctionCallback(void *ptr, const double *x, double *grad, int n);
    static int earlyExitCallback(void *func_data, const double *x, const double *g,
                                 double fx, double xnorm, double gnorm,
                                 double step, int n, int k, int ls);

    void gradientFeasibility(double *gradient_c, double *gradient_time, double &cost);
    bool feasibilityGradCostV(const Eigen::VectorXd &velocity,
                              Eigen::VectorXd &gradient, double &cost);
    bool feasibilityGradCostA(const Eigen::VectorXd &acceleration,
                              Eigen::VectorXd &gradient, double &cost);
    bool feasibilityGradCostJ(const Eigen::VectorXd &jerk,
                              Eigen::VectorXd &gradient, double &cost);
    bool obstacleGradCostP(int id, const Eigen::VectorXd &position,
                           Eigen::VectorXd &gradient, double &cost);

    CheckResult finelyCheckAndSetConstraintPoints(
        std::vector<std::pair<int, int>> &segments);
    bool roughlyCheckConstraintPoints();
    bool allowRebound(const Eigen::MatrixXd &sample_points) const;
    bool computePointsToCheck(
        int end_id, std::vector<std::vector<Eigen::Vector3d>> &points_to_check) const;

    bool isOccupied(const Eigen::VectorXd &position) const;
    Eigen::Vector3d toWorldPosition(const Eigen::VectorXd &position) const;
    void emitDebugPoints(const std::string &name,
                         const std::vector<Eigen::Vector3d> &points) const;
    void ensureConfigured() const;

    Config config_;
    OccupancyQuery occupancy_query_;
    DebugPointsCallback debug_points_callback_;
    std::unique_ptr<Spline::PolynomialSpline> minco_ptr_;
    AStarPtr a_star_;

    int variable_num_ = 0;
    int K_ = 10;
    double weight_time_ = 1.0;
    double weight_dis_ = 0.0;
    double weight_c_ = 1.0;
    double weight_c_soft_ = 0.0;
    double weight_v_ = 1.0;
    double weight_a_ = 1.0;
    double weight_j_ = 1.0;
    double v_max_ = 3.0;
    double acc_max_ = 3.0;
    double jerk_max_ = 6.0;

    std::vector<std::vector<Eigen::Vector3d>> base_point_;
    std::vector<std::vector<Eigen::Vector3d>> direction_;
    std::vector<bool> flag_temp_;
    int iter_num_ = 0;

    enum class ForceStopType
    {
        DontStop,
        StopForRebound,
        StopForError
    };
    ForceStopType force_stop_type_ = ForceStopType::DontStop;
};

using EgoPtr = std::shared_ptr<Ego>;

}  // namespace ego
