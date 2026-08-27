#include "ego/ego.hpp"
#include "lbfgs.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace ego
{

Ego::Ego(const Config &config, OccupancyQuery occupancy_query)
{
    configure(config, std::move(occupancy_query));
}

void Ego::configure(const Config &config, OccupancyQuery occupancy_query)
{
    if (config.dimension != 2 && config.dimension != 3)
        throw std::invalid_argument("ego: trajectory dimension must be 2 or 3");
    if (!std::isfinite(config.piece_length) || config.piece_length <= 0.0)
        throw std::invalid_argument("ego: piece length must be positive");
    if (!std::isfinite(config.map_resolution) || config.map_resolution <= 0.0)
        throw std::invalid_argument("ego: map resolution must be positive");
    if (config.samples_per_piece <= 0)
        throw std::invalid_argument("ego: samples_per_piece must be positive");
    if (!std::isfinite(config.obstacle_clearance) || config.obstacle_clearance < 0.0 ||
        !std::isfinite(config.soft_obstacle_clearance) ||
        config.soft_obstacle_clearance < 0.0)
        throw std::invalid_argument("ego: obstacle clearances must be finite and non-negative");
    if (config.max_velocity <= 0.0 || config.max_acceleration <= 0.0 ||
        config.max_jerk <= 0.0)
        throw std::invalid_argument("ego: dynamic limits must be positive");
    if (!occupancy_query)
        throw std::invalid_argument("ego: occupancy query is empty");

    config_ = config;
    occupancy_query_ = std::move(occupancy_query);
    minco_ptr_ = std::make_unique<Spline::PolynomialSpline>(
        config.dimension, config.piece_length);

    weight_time_ = config.weight_time;
    weight_dis_ = config.weight_distance;
    weight_c_ = config.weight_collision;
    weight_c_soft_ = config.weight_soft_collision;
    weight_v_ = config.weight_velocity;
    weight_a_ = config.weight_acceleration;
    weight_j_ = config.weight_jerk;
    obstacle_clearance_ = config.obstacle_clearance;
    soft_obstacle_clearance_ = config.soft_obstacle_clearance;
    K_ = config.samples_per_piece;
    v_max_ = config.max_velocity;
    acc_max_ = config.max_acceleration;
    jerk_max_ = config.max_jerk;

    AStar::Config astar_config;
    astar_config.dimension = config.dimension;
    astar_config.resolution = config.map_resolution;
    astar_config.origin = config.map_origin;
    astar_config.height = config.planning_height;
    astar_config.max_search_time_ms = config.astar_max_time_ms;
    a_star_ = std::make_shared<AStar>(astar_config, occupancy_query_);
}

void Ego::setOccupancyQuery(OccupancyQuery occupancy_query)
{
    if (!occupancy_query)
        throw std::invalid_argument("ego: occupancy query is empty");
    occupancy_query_ = std::move(occupancy_query);
    if (a_star_)
    {
        AStar::Config astar_config;
        astar_config.dimension = config_.dimension;
        astar_config.resolution = config_.map_resolution;
        astar_config.origin = config_.map_origin;
        astar_config.height = config_.planning_height;
        astar_config.max_search_time_ms = config_.astar_max_time_ms;
        a_star_->configure(astar_config, occupancy_query_);
    }
}

void Ego::setDebugPointsCallback(DebugPointsCallback callback)
{
    debug_points_callback_ = std::move(callback);
}

void Ego::setHeight(double height)
{
    if (config_.dimension != 2)
        return;
    config_.planning_height = height;
    ensureConfigured();
    a_star_->setHeight(height);
}

const Spline::PolynomialSpline &Ego::trajectory() const
{
    ensureConfigured();
    return *minco_ptr_;
}

Spline::PolynomialSpline &Ego::trajectory()
{
    ensureConfigured();
    return *minco_ptr_;
}

void Ego::saveTrajectoryCsv(const std::string &path, double sample_interval) const
{
    ensureConfigured();
    if (!std::isfinite(sample_interval) || sample_interval <= 0.0)
        throw std::invalid_argument("ego: CSV sample interval must be positive");
    if (minco_ptr_->getPieceNum() <= 0 || minco_ptr_->getTotalDuration() <= 0.0)
        throw std::logic_error("ego: no generated trajectory is available");

    std::ofstream csv(path);
    if (!csv)
        throw std::runtime_error("ego: cannot open trajectory CSV: " + path);

    csv << "t,x,y,z,vx,vy,vz,ax,ay,az,jx,jy,jz,occupied\n";
    csv << std::setprecision(12);

    const double duration = minco_ptr_->getTotalDuration();
    const auto write_sample = [this, &csv](double t) {
        const Eigen::VectorXd position = minco_ptr_->getState(t, 0);
        const Eigen::VectorXd velocity = minco_ptr_->getState(t, 1);
        const Eigen::VectorXd acceleration = minco_ptr_->getState(t, 2);
        const Eigen::VectorXd jerk = minco_ptr_->getState(t, 3);

        const Eigen::Vector3d world_position = toWorldPosition(position);
        Eigen::Vector3d world_velocity = Eigen::Vector3d::Zero();
        Eigen::Vector3d world_acceleration = Eigen::Vector3d::Zero();
        Eigen::Vector3d world_jerk = Eigen::Vector3d::Zero();
        world_velocity.head(config_.dimension) = velocity.head(config_.dimension);
        world_acceleration.head(config_.dimension) = acceleration.head(config_.dimension);
        world_jerk.head(config_.dimension) = jerk.head(config_.dimension);

        csv << t << ','
            << world_position.x() << ',' << world_position.y() << ',' << world_position.z() << ','
            << world_velocity.x() << ',' << world_velocity.y() << ',' << world_velocity.z() << ','
            << world_acceleration.x() << ',' << world_acceleration.y() << ',' << world_acceleration.z() << ','
            << world_jerk.x() << ',' << world_jerk.y() << ',' << world_jerk.z() << ','
            << (occupancy_query_(world_position) ? 1 : 0) << '\n';
    };

    const std::size_t complete_steps =
        static_cast<std::size_t>(std::floor(duration / sample_interval));
    for (std::size_t i = 0; i <= complete_steps; ++i)
        write_sample(std::min(duration, static_cast<double>(i) * sample_interval));

    const double last_regular_time = static_cast<double>(complete_steps) * sample_interval;
    if (duration - last_regular_time > 1e-12)
        write_sample(duration);

    if (!csv)
        throw std::runtime_error("ego: failed while writing trajectory CSV: " + path);
}

void Ego::ensureConfigured() const
{
    if (!minco_ptr_ || !a_star_ || !occupancy_query_)
        throw std::logic_error("ego: configure() must be called before planning");
}

Eigen::Vector3d Ego::toWorldPosition(const Eigen::VectorXd &position) const
{
    Eigen::Vector3d world = Eigen::Vector3d::Zero();
    world.head(config_.dimension) = position.head(config_.dimension);
    if (config_.dimension == 2)
        world.z() = config_.planning_height;
    return world;
}

bool Ego::isOccupied(const Eigen::VectorXd &position) const
{
    return occupancy_query_(toWorldPosition(position));
}

void Ego::emitDebugPoints(const std::string &name,
                          const std::vector<Eigen::Vector3d> &points) const
{
    if (debug_points_callback_)
        debug_points_callback_(name, points);
}



void Ego::gradientFeasibility(double *gradientC, double *gradientRT, double &costF)
{

    Eigen::Map<Eigen::MatrixXd> gradientKC(gradientC, minco_ptr_->getOrder() * minco_ptr_->getPieceNum(), minco_ptr_->getDimensions());
     
    int K = K_ + 1;
    double step;
    double s1, s2, s3, s4, s5;
    double omg, alpha;
    int i_dp = 0;
    Eigen::VectorXd pos, vel, acc, jer, sna;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
    Eigen::MatrixXd gradViolaPc, gradViolaVc, gradViolaAc, gradViolaJc;
    double gradViolaPt, gradViolaVt, gradViolaAt, gradViolaJt;
    Eigen::VectorXd gradp, gradv, grada, gradj;
    double costp, costV, costA, costJ;
    for(int i = 0; i < minco_ptr_->getPieceNum(); ++i)
    {
        Eigen::MatrixXd c = minco_ptr_->getCoeffs().block(minco_ptr_->getOrder() * i, 0, minco_ptr_->getOrder(), minco_ptr_->getDimensions());
        step = minco_ptr_->getRealTimeVec()(i) / K;
        // time
        s1 = 0.0;
        for(int j = 0; j <= K; ++j)
        {
            // time 
            s2 = s1 * s1;
            s3 = s2 * s1;
            s4 = s2 * s2;
            s5 = s4 * s1;
            beta0 << 1.0, s1, s2, s3, s4, s5;
            beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
            beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
            beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
            beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1;
            alpha = 1.0 / K * j;
            pos = c.transpose() * beta0;
            vel = c.transpose() * beta1;
            acc = c.transpose() * beta2;
            jer = c.transpose() * beta3;
            sna = c.transpose() * beta4;

            omg = (j == 0 || j == K) ? 0.5 : 1.0;
            if (obstacleGradCostP(i_dp, pos, gradp, costp))
            {
                gradViolaPc = beta0 * gradp.transpose();
                gradViolaPt = alpha * gradp.transpose() * vel;
                gradientKC.block(minco_ptr_->getOrder() * i, 0, minco_ptr_->getOrder(), minco_ptr_->getDimensions()) 
                                += omg * step * gradViolaPc;
                gradientRT[i] += omg * (costp / K + step * gradViolaPt);
                costF += omg * step * costp;
            }
            // Velocity-limit penalty.
            if (feasibilityGradCostV(vel, gradv, costV))
            {
                gradViolaVc = beta1 * gradv.transpose();
                gradViolaVt = alpha * gradv.transpose() * acc;
                gradientKC.block(minco_ptr_->getOrder() * i, 0, minco_ptr_->getOrder(), minco_ptr_->getDimensions()) 
                                += omg * step * gradViolaVc;
                gradientRT[i] += omg * (costV / K + step * gradViolaVt);
                costF += omg * step * costV;
            }
            if(feasibilityGradCostA(acc, grada, costA))
            {
                gradViolaAc = beta2 * grada.transpose();
                gradViolaAt = alpha * grada.transpose() * jer;
                gradientKC.block(minco_ptr_->getOrder() * i, 0, minco_ptr_->getOrder(), minco_ptr_->getDimensions()) 
                                += omg * step * gradViolaAc;
                gradientRT[i] += omg * (costA / K + step * gradViolaAt);
                costF += omg * step * costA;
            }
            if(feasibilityGradCostJ(jer, gradj, costJ))
            {
                gradViolaJc = beta3 * gradj.transpose();
                gradViolaJt = alpha * gradj.transpose() * sna;
                gradientKC.block(minco_ptr_->getOrder() * i, 0, minco_ptr_->getOrder(), minco_ptr_->getDimensions()) 
                                += omg * step * gradViolaJc;
                gradientRT[i] += omg * (costJ / K + step * gradViolaJt);
                costF += omg * step * costJ;
            }
            s1 += step;
            if (j != K || (j == K && i == minco_ptr_->getPieceNum() - 1))
            {
                ++i_dp;
            }
        }
    }
}


bool Ego::feasibilityGradCostV(const Eigen::VectorXd &v, Eigen::VectorXd &gradv, double &costv)
{
    double vpen = v.squaredNorm() - v_max_ * v_max_;
    if (vpen > 0)
    {
        gradv = weight_v_ * 6 * vpen * vpen * v;
        costv = weight_v_ * vpen * vpen * vpen;
        return true;
    }
    return false;
}

bool Ego::feasibilityGradCostA(const Eigen::VectorXd &a, Eigen::VectorXd &grada, double &costa)
{
    double apen = a.squaredNorm() - acc_max_ * acc_max_;
    if (apen > 0)
    {
        grada = weight_a_ * 6 * apen * apen * a;
        costa = weight_a_ * apen * apen * apen;
        return true;
    }
    return false;
}

bool Ego::feasibilityGradCostJ(const Eigen::VectorXd &j, Eigen::VectorXd &gradj, double &costj)
{
    double Jpen = j.squaredNorm() - jerk_max_ * jerk_max_;
    if (Jpen > 0)
    {
        gradj = weight_j_ * 6 * Jpen * Jpen * j;
        costj = weight_j_ * Jpen * Jpen * Jpen;
        return true;
    }
    return false;
}

bool Ego::obstacleGradCostP(const int id, const Eigen::VectorXd &pos, Eigen::VectorXd &gradp, double &costp)
{
    if(id <= 0 || static_cast<std::size_t>(id) >= direction_.size() ||
       direction_[id].empty())
        return false;
    
    bool ret = false;

    if(2 == minco_ptr_->getDimensions())
        gradp.resize(2);
    else
        gradp.resize(3);
    
    gradp.setZero();
    costp = 0;

    Eigen::Vector3d pose;
    if(2 == minco_ptr_->getDimensions())
        pose << pos.x(), pos.y(), a_star_->height();
    else
        pose = pos;

    for(size_t j = 0; j < direction_[id].size(); ++j)
    {
        const double dist = (pose - base_point_[id][j]).dot(direction_[id][j]);
        const double dist_err = obstacle_clearance_ - dist;
        const double dist_err_soft = soft_obstacle_clearance_ - dist;
        const Eigen::Vector3d &dist_grad = direction_[id][j];

        if(dist_err > 0)
        {
            ret = true;
            costp += weight_c_ * std::pow(dist_err, 3);
            if(2 == minco_ptr_->getDimensions())
                gradp += -weight_c_ * 3.0 * dist_err * dist_err * dist_grad.head(2);
            else
                gradp += -weight_c_ * 3.0 * dist_err * dist_err * dist_grad;
        }

        if(dist_err_soft > 0)
        {
            ret = true;
            constexpr double r = 0.05;
            constexpr double rsqr = r * r;
            const double term = std::sqrt(1.0 + dist_err_soft * dist_err_soft / rsqr);
            costp += weight_c_soft_ * rsqr * (term - 1.0);
            if(2 == minco_ptr_->getDimensions())
                gradp += -weight_c_soft_ * dist_err_soft / term * dist_grad.head(2);
            else
                gradp += -weight_c_soft_ * dist_err_soft / term * dist_grad;
        }
    }

    return ret;
}

bool Ego::optimize(const Eigen::MatrixXd &startState, const Eigen::MatrixXd &targetState, 
                              const Eigen::MatrixXd &way_points, const Eigen::VectorXd &real_time_vector, 
                              Eigen::MatrixXd &optimal_points, Eigen::VectorXd &optimal_T,
                              double &final_cost)
{
    ensureConfigured();
    if (startState.rows() != config_.dimension || startState.cols() != 3 ||
        targetState.rows() != config_.dimension || targetState.cols() != 3)
        throw std::invalid_argument("ego: start and target states must be dimension x 3");
    if (real_time_vector.size() <= 0 ||
        way_points.rows() != config_.dimension ||
        way_points.cols() != real_time_vector.size() - 1)
        throw std::invalid_argument("ego: waypoint/time dimensions are inconsistent");
    if ((real_time_vector.array() <= 0.0).any())
        throw std::invalid_argument("ego: every piece duration must be positive");

    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    int restart_nums = 0, rebound_times = 0;
    bool flag_force_return, flag_still_unsafe, flag_success;
    
    minco_ptr_->SetParam(startState, targetState, real_time_vector.rows());
    minco_ptr_->generate(way_points, real_time_vector);

    std::vector<std::vector<Eigen::Vector3d>>().swap(base_point_);
    base_point_.resize(minco_ptr_->getPieceNum() * K_ + minco_ptr_->getAllPoint().cols());
    std::vector<std::vector<Eigen::Vector3d>>().swap(direction_);
    direction_.resize(minco_ptr_->getPieceNum() * K_ + minco_ptr_->getAllPoint().cols());
    std::vector<bool>().swap(flag_temp_);
    flag_temp_.resize(minco_ptr_->getPieceNum() * K_ + minco_ptr_->getAllPoint().cols());

    std::vector<std::pair<int, int>> segments;
    if(finelyCheckAndSetConstraintPoints(segments) == CheckResult::Error)
    {
        return false;
    }

    // set initial value
    variable_num_ = minco_ptr_->getDimensions() * (minco_ptr_->getPieceNum() - 1) + minco_ptr_->getPieceNum();
    std::vector<double> x_init(static_cast<std::size_t>(variable_num_));
    // way points
    std::memcpy(x_init.data(), way_points.data(),
                static_cast<std::size_t>(way_points.size()) * sizeof(double));
    // time vecotr
    Eigen::Map<Eigen::VectorXd> VT(x_init.data() + way_points.size(), minco_ptr_->getPieceNum());

    // convert real time vector to virtual time vector
    minco_ptr_->RealT2VirtualT(real_time_vector.data(), VT.data());

    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;
    lbfgs_params.max_iterations = 200;
    lbfgs_params.min_step = 1e-32;
    lbfgs_params.past = 3;
    lbfgs_params.delta = 1.0e-2;

    do{
        iter_num_ = 0;
        flag_force_return = false;
        force_stop_type_ = ForceStopType::DontStop;
        flag_still_unsafe = false;
        flag_success = false;

        // Run one optimization attempt.
        const auto t1 = Clock::now();
        int result = lbfgs::lbfgs_optimize(
            variable_num_,
            x_init.data(),
            &final_cost,
            Ego::costFunctionCallback,
            nullptr,
            Ego::earlyExitCallback,
            this,
            &lbfgs_params);
        const auto t2 = Clock::now();

        const double time_ms =
            std::chrono::duration<double, std::milli>(t2 - t1).count();
        const double total_time_ms =
            std::chrono::duration<double, std::milli>(t2 - t0).count();

        if(result == lbfgs::LBFGS_CONVERGENCE ||
           result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
           result == lbfgs::LBFGS_ALREADY_MINIMIZED||
           result == lbfgs::LBFGS_STOP)
        {
            flag_force_return = false;
            std::vector<std::pair<int, int>> segments;
            if(finelyCheckAndSetConstraintPoints(segments) == CheckResult::ObstacleFree)
            {
                flag_success = true;
                (void)time_ms;
                (void)total_time_ms;
            }
            else    // The optimized trajectory is still in collision.
            {
                flag_still_unsafe = true;
                restart_nums++;
            }
        }
        else if(result == lbfgs::LBFGSERR_CANCELED) // New constraints require a rebound.
        {
            flag_force_return = true;
            rebound_times++;
            // std::cout << "reBound occur" << std::endl;
        }
    }while((flag_still_unsafe && restart_nums < 3) ||
           (flag_force_return && force_stop_type_ == ForceStopType::StopForRebound && rebound_times <= 20));
    
    Eigen::VectorXd RT(minco_ptr_->getPieceNum());
    minco_ptr_->VirtualT2RealT(VT.data(), RT.data());
    Eigen::Map<Eigen::MatrixXd> wps(x_init.data(), way_points.rows(), way_points.cols());
    minco_ptr_->generate(wps, RT);
    optimal_points = wps;
    optimal_T = RT;
    return flag_success;
}

double Ego::costFunctionCallback(void * ptr, const double *x, double *grad, const int n)
{
    (void)n;
    Ego &obj = *(Ego *)ptr;
    // Intermediate waypoints.
    Eigen::Map<const Eigen::MatrixXd> wps(x, obj.minco_ptr_->getDimensions(), obj.minco_ptr_->getPieceNum() - 1);
    // Unconstrained virtual durations.
    Eigen::Map<const Eigen::VectorXd> VT(x + wps.size(), obj.minco_ptr_->getPieceNum());
    // Gradient with respect to waypoints.
    Eigen::Map<Eigen::MatrixXd> grad_J_P(grad, obj.minco_ptr_->getDimensions(), obj.minco_ptr_->getPieceNum() - 1);
    // Gradient with respect to virtual durations.
    Eigen::Map<Eigen::VectorXd> grad_J_VT(grad + obj.minco_ptr_->getDimensions() * (obj.minco_ptr_->getPieceNum() - 1), obj.minco_ptr_->getPieceNum());
    // Convert virtual durations to positive physical durations.
    Eigen::VectorXd RT(obj.minco_ptr_->getPieceNum());
    obj.minco_ptr_->VirtualT2RealT(VT.data(), RT.data());
    
    // Generate the current spline.
    obj.minco_ptr_->generate(wps, RT);
    // Gradient with respect to polynomial coefficients.
    Eigen::MatrixXd gradientJ_c(obj.minco_ptr_->getOrder() * obj.minco_ptr_->getPieceNum(), obj.minco_ptr_->getDimensions());
    gradientJ_c.setZero();
    // Gradient with respect to physical durations.
    Eigen::VectorXd gradientJ_RT(obj.minco_ptr_->getPieceNum());
    gradientJ_RT.setZero();

    // Initialize the smoothness gradient and jerk cost.
    obj.minco_ptr_->gradientJ_E_RT(gradientJ_RT.data());
    obj.minco_ptr_->gradientJ_E_C(gradientJ_c.data());
    double costJerk = 0.0;
    costJerk = obj.minco_ptr_->getJerkCost();

    // Add the sample-spacing regularization gradient.
    double costDis = 0.0;
    Eigen::MatrixXd cps;
    cps = obj.minco_ptr_->getSampleStates(obj.K_, 0);
    obj.minco_ptr_->gradientDistanceC_RT(obj.weight_dis_, obj.K_, cps, gradientJ_c.data(), gradientJ_RT.data(), costDis);

    double costF = 0.0;
    obj.gradientFeasibility(gradientJ_c.data(), gradientJ_RT.data(), costF);

    // Detect newly introduced collisions and request a rebound if needed.
    if(obj.allowRebound(cps))
    {
        obj.roughlyCheckConstraintPoints();
    }

    // Propagate coefficient gradients to waypoint gradients.
    obj.minco_ptr_->grad2P_RT(gradientJ_c.data(), grad_J_P.data(), gradientJ_RT.data());
    // Propagate physical-duration gradients to virtual durations.
    double costTime = 0.0;
    obj.minco_ptr_->DiffphismVirtualTGrad(RT.data(), VT.data(), gradientJ_RT.data(), obj.weight_time_, grad_J_VT.data(), costTime);
    
    // Count completed optimizer iterations.
    obj.iter_num_++;

    return costJerk + costDis + costF + costTime;
}

int Ego::earlyExitCallback(void *func_data, const double *x, const double *g,
                                const double fx, const double xnorm, const double gnorm,
                                const double step, int n, int k, int ls)
{
    (void)x;
    (void)g;
    (void)fx;
    (void)xnorm;
    (void)gnorm;
    (void)step;
    (void)n;
    (void)k;
    (void)ls;
    Ego *ego_ptr = reinterpret_cast<Ego *>(func_data);
    return ego_ptr->force_stop_type_ == ForceStopType::StopForRebound ||
           ego_ptr->force_stop_type_ == ForceStopType::StopForError;
}

bool Ego::computePointsToCheck(
    int id_cps_end, std::vector<std::vector<Eigen::Vector3d>> &pts_check) const
{
    if (id_cps_end <= 0)
        return false;
    pts_check.clear();
    pts_check.resize(static_cast<std::size_t>(id_cps_end));

    const double resolution = config_.map_resolution;
    const Eigen::VectorXd durations = minco_ptr_->getRealTimeVec();

    const double DURATION = minco_ptr_->getTotalDuration();
    double t = 0.0, t_step = std::min(resolution / v_max_, durations.minCoeff() / std::max(K_ + 1, 1) / 1.5);

    double uniform_time = DURATION / (1.0 * id_cps_end);
    Eigen::VectorXd uniform_time_vector(id_cps_end);
    for(int i = 0; i < id_cps_end; ++i)
    {
        uniform_time_vector(i) =  (i + 1.0) * uniform_time;
    }

    int piece = 0;
    Eigen::VectorXd tmp;
    Eigen::Vector3d pos;
    while(t < DURATION)
    {
        
        tmp = minco_ptr_->getState(t);

        if(2 == minco_ptr_->getDimensions())
            pos << tmp.x(), tmp.y(), a_star_->height();
        else
            pos = tmp;
        
        while (piece < id_cps_end - 1 && t > uniform_time_vector(piece))
            piece++;

        pts_check[piece].emplace_back(pos);
        t += t_step;
    }
    return true;
}

Ego::CheckResult Ego::finelyCheckAndSetConstraintPoints(std::vector<std::pair<int, int>> &segments)
{
    // Sample K_ points from each trajectory piece.
    Eigen::MatrixXd init_points = minco_ptr_->getSampleStates(K_, 0);


    int in_id = -1, out_id = -1;
    std::vector<std::pair<int, int>> segment_ids;
    constexpr int ENOUGH_INTERVAL = 2;
    int same_occ_state_times = ENOUGH_INTERVAL + 1;
    bool occ, last_occ = false;
    bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
    int i_end = init_points.cols() - 1;

    const double resolution = config_.map_resolution;

    std::vector<std::vector<Eigen::Vector3d>> pts_check;

    if(!computePointsToCheck(i_end, pts_check))
    {
        return CheckResult::Error;
    }


    for(int i = 0; i < i_end; ++i)
    {
        for(size_t j = 0; j < pts_check[i].size(); ++j)
        {
            occ = isOccupied(pts_check[i][j]);
            if(occ && !last_occ) // Entering an occupied segment.
            {
                if(same_occ_state_times > ENOUGH_INTERVAL)
                {
                    if(i == 0)
                        in_id = 0;
                    else
                        in_id = i - 1;
                    flag_got_start = true;
                }
                same_occ_state_times = 0;
                flag_got_end_maybe = false;
            }
            else if(!occ && last_occ)
            {
                out_id = i + 1;
                flag_got_end_maybe = true;
                same_occ_state_times = 0;
            }
            else
            {
                ++same_occ_state_times;
            }

            if(flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || (i == i_end - 1)))
            // if(flag_got_end_maybe)
            {
                flag_got_end_maybe = false;
                flag_got_end = true;
            }

            last_occ = occ;

            if(flag_got_start && flag_got_end)
            {
                flag_got_end = false;
                flag_got_start = false;
                if(in_id < 0 || out_id < 0) // Defensive consistency check.
                {
                    return CheckResult::Error;
                }
                segment_ids.push_back(std::pair<int, int>(in_id, out_id)); // Store collision bounds.
            }
        }
    }


    if(0 == segment_ids.size()) // No collision was found.
        return CheckResult::ObstacleFree;
    

    // Compute non-overlapping adjustment bounds for each collision segment.
    int id_low_bound, id_up_bound;
    std::vector<std::pair<int, int>> bounds(segment_ids.size());
    for(size_t i = 0; i < segment_ids.size(); ++i)
    {
        if(0 == i)
        {
            id_low_bound = 1;
            if(segment_ids.size() > 1)
                id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0) / 2.0);
            else // Only one collision segment remains.
                id_up_bound = init_points.cols() - 2; // Stop at the penultimate point.
        }
        else if(i == segment_ids.size() - 1) // Last collision segment.
        {
            id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0) / 2.0);
            id_up_bound = init_points.cols() - 2;
        }
        else
        {
            id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0) / 2.0);
            id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0) / 2.0);
        }
        bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
    }

    // Adjust short collision segments without changing their ordering.
    std::vector<std::pair<int, int>> adjusted_segment_ids(segment_ids.size());
    int minimum_points = 0, num_points;
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      /*** Adjust segment length ***/
      num_points = segment_ids[i].second - segment_ids[i].first + 1;
      if (num_points < minimum_points)
      {
        double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);

        adjusted_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first
                                            ? segment_ids[i].first - add_points_each_side
                                            : bounds[i].first;

        adjusted_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second
                                             ? segment_ids[i].second + add_points_each_side
                                             : bounds[i].second;
      }
      else
      {
        adjusted_segment_ids[i].first = segment_ids[i].first;
        adjusted_segment_ids[i].second = segment_ids[i].second;
      }
    }

    // Split overlapping segment bounds at their midpoint.
    for(size_t i = 1; i < adjusted_segment_ids.size(); ++i)
    {
        if (adjusted_segment_ids[i - 1].second >= adjusted_segment_ids[i].first)
        {
            double middle = (double)(adjusted_segment_ids[i - 1].second + adjusted_segment_ids[i].first) / 2.0;
            adjusted_segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
            adjusted_segment_ids[i].first = static_cast<int>(middle + 1.1);
        }
    }


    // Run A* around every colliding trajectory segment.
    double costTime = 0.0;
    std::vector<std::vector<Eigen::Vector3d>> a_star_pathes;


    for(size_t i = 0; i < adjusted_segment_ids.size(); ++i)
    {
        Eigen::Vector3d in, out;
        if(2 == minco_ptr_->getDimensions())
        {
            in << init_points.col(adjusted_segment_ids[i].second).x(), init_points.col(adjusted_segment_ids[i].second).y(), a_star_->height();
            out << init_points.col(adjusted_segment_ids[i].first).x(), init_points.col(adjusted_segment_ids[i].first).y(), a_star_->height();
        }
        else if(3 == minco_ptr_->getDimensions())
            in = init_points.col(adjusted_segment_ids[i].second), out = init_points.col(adjusted_segment_ids[i].first);
        else
            return CheckResult::Error;


        AStarResult ret = a_star_->search(in, out, costTime);
        if(ret == AStarResult::Success)
        {
            auto path = a_star_->path();
            // EGO projects along the original trajectory direction (out -> in).
            std::reverse(path.begin(), path.end());
            a_star_pathes.push_back(path);
            emitDebugPoints("a_star_path", path);
        }
        else if(ret == AStarResult::Timeout && i + 1 < adjusted_segment_ids.size())
        {
            adjusted_segment_ids[i].second = adjusted_segment_ids[i + 1].second;
            adjusted_segment_ids.erase(adjusted_segment_ids.begin() + i + 1);
            --i;
        }
        else
        {
            adjusted_segment_ids.erase(adjusted_segment_ids.begin() + i); // Discard the failed segment.
        }
    }



    if(a_star_pathes.size() == 0)
    {
        return CheckResult::Error;
    }



    // Build obstacle base-point/direction constraints for every collision segment.
    std::vector<std::pair<int, int>> final_segment_ids;
    Eigen::Vector3d tmp;
    for(size_t i = 0; i < adjusted_segment_ids.size(); ++i)
    {
        // Track whether each sampled point received a valid constraint.
        for(int j = adjusted_segment_ids[i].first; j <= adjusted_segment_ids[i].second; ++j)
            flag_temp_[j] = false;
        
        int got_intersection_id = -1;
        for(int j = adjusted_segment_ids[i].first + 1; j < adjusted_segment_ids[i].second; ++j)
        {
            got_intersection_id = -1;
            if(i >= a_star_pathes.size())
                break;

            Eigen::Vector3d ctrl_pts_law;
            Eigen::Vector3d intersection_point; // Orthogonal path intersection.
            if(2 == minco_ptr_->getDimensions())
            {
                tmp.head(2) = init_points.col(j);
                tmp(2) = a_star_->height();
                ctrl_pts_law.head(2) = init_points.col(j + 1) - init_points.col(j - 1);
                ctrl_pts_law(2) = 0;
            }
            else
            {
                tmp = init_points.col(j);
                ctrl_pts_law = init_points.col(j + 1) - init_points.col(j - 1);
            }

            int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id;
            // Walk along the A* path until the projection dot product changes sign.
            double val = (a_star_pathes[i][Astar_id] - tmp).dot(ctrl_pts_law), init_val = val;

            while(true)
            {
                last_Astar_id = Astar_id;

                if(val >= 0) // Move backward until the dot product becomes negative.
                {
                    --Astar_id;
                    if(Astar_id < 0)
                    {
                        break;
                    }
                }
                else // Move forward until the dot product becomes positive.
                {
                    ++Astar_id;
                    if(Astar_id >= (int)a_star_pathes[i].size())
                    {
                        break;
                    }
                }
                // Update the projection sign.
                val = (a_star_pathes[i][Astar_id] - tmp).dot(ctrl_pts_law);
                // std::cout << "init_val : " << init_val << std::endl;
                // std::cout << "     val : " << val << std::endl;
                if(init_val * val <= 0 && (abs(val) > 0 || abs(init_val) > 0))
                {
                    // Interpolate the point where the A* path crosses the normal plane.
                    intersection_point =
                        a_star_pathes[i][Astar_id] +
                        ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                        (ctrl_pts_law.dot(tmp - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]))
                        );
                    got_intersection_id = j;
                    break;
                }
            }

            if(got_intersection_id >= 0)
            {
                // Measure the ray length used for occupancy interpolation.
                double length = (intersection_point - tmp).norm();
                if(length > 1e-5) // Reject numerically degenerate rays.
                {
                    flag_temp_[j] = true;
                    for(double a = length; a >= 0.0; a -= resolution)
                    {
                        bool occ = isOccupied((a / length) * intersection_point + (1 - a / length) * tmp);
                        // Stop at the obstacle boundary or at the final interpolation step.
                        if(occ || a < resolution)
                        {
                            if(occ)
                                a += resolution;
                            base_point_[j].push_back((a / length) * intersection_point + (1 - a / length) * tmp);
                            direction_[j].push_back((intersection_point - tmp).normalized());
                            break;
                        }
                    }
                }
                else
                {
                    got_intersection_id = -1; // Intersection construction failed.
                }
            }
        }
        
        // Handle a collision segment containing only two adjacent samples.
        if(adjusted_segment_ids[i].second - adjusted_segment_ids[i].first == 1)
        {
            got_intersection_id = -1;
            if(i >= a_star_pathes.size())
                break;
            
            Eigen::Vector3d ctrl_pts_law, intersection_point, middle_point;
            if(2 == minco_ptr_->getDimensions())
            {
                ctrl_pts_law.head(2) = init_points.col(adjusted_segment_ids[i].second) - init_points.col(adjusted_segment_ids[i].first);
                ctrl_pts_law(2) = 0;
                middle_point.head(2) = (init_points.col(adjusted_segment_ids[i].second) + init_points.col(adjusted_segment_ids[i].first)) / 2.0;
                middle_point(2) = a_star_->height();
            }
            else
            {
                ctrl_pts_law = init_points.col(adjusted_segment_ids[i].second) - init_points.col(adjusted_segment_ids[i].first);
                middle_point = (init_points.col(adjusted_segment_ids[i].second) + init_points.col(adjusted_segment_ids[i].first)) / 2.0;
            }
            int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
            double val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law), init_val = val;
            while (true)
            {

                last_Astar_id = Astar_id;

                if(val >= 0) // Move backward until the dot product changes sign.
                {
                    --Astar_id;
                    if(Astar_id < 0)
                    {
                        // std::cout << "Astar_id lower than zero" << std::endl;
                        break;
                    }
                }
                else // Move forward until the dot product changes sign.
                {
                    ++Astar_id;
                    if(Astar_id >= (int)a_star_pathes[i].size())
                    {
                        // std::cout << "Astar_id over size" << std::endl;
                        break;
                    }
                }

                val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law);

                if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) // val = init_val = 0.0 is not allowed
                {
                    intersection_point =
                        a_star_pathes[i][Astar_id] +
                        ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                        (ctrl_pts_law.dot(middle_point - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                        );

                    if ((intersection_point - middle_point).norm() > 0.01) // 1cm.
                    {
                        flag_temp_[adjusted_segment_ids[i].first] = true;
                        if(2 == minco_ptr_->getDimensions())
                        {
                            tmp.head(2) = init_points.col(adjusted_segment_ids[i].first);
                            tmp(2) = a_star_->height();
                        }
                        else
                            tmp = init_points.col(adjusted_segment_ids[i].first);
                        base_point_[adjusted_segment_ids[i].first].push_back(tmp);
                        direction_[adjusted_segment_ids[i].first].push_back((intersection_point - middle_point).normalized());

                        got_intersection_id = adjusted_segment_ids[i].first;
                    }
                    break;
                }
            }
        }


        // Fill missing constraints from the nearest successfully constrained sample.
        if(got_intersection_id >= 0)
        {
            for(int j = got_intersection_id + 1; j <= adjusted_segment_ids[i].second; ++j)
            {
                if(!flag_temp_[j])
                {
                    base_point_[j].push_back(base_point_[j - 1].back());
                    direction_[j].push_back(direction_[j - 1].back());
                }
            }
            for(int j = got_intersection_id - 1; j >= adjusted_segment_ids[i].first; --j)
            {
                if(!flag_temp_[j])
                {
                    base_point_[j].push_back(base_point_[j + 1].back());
                    direction_[j].push_back(direction_[j + 1].back());
                }
            }
            final_segment_ids.push_back(adjusted_segment_ids[i]);
        }
    }
    segments = final_segment_ids;
    return CheckResult::ConstraintsCreated;
}

bool Ego::roughlyCheckConstraintPoints(void)
{
    int in_id = -1, out_id = -1;
    std::vector<std::pair<int, int>> segment_ids;
    bool flag_new_obs_valid = false;
    Eigen::MatrixXd points = minco_ptr_->getSampleStates(K_, 0);
    // Index of the final sampled point.
    int i_end = points.cols() - 1;
    bool occ;
    const double resolution = config_.map_resolution;
    // Search for newly colliding samples.
    Eigen::Vector3d point;
    for(int i = 1; i <= i_end; ++i)
    {
        if(2 == minco_ptr_->getDimensions())
            point << points.col(i).x(), points.col(i).y(), a_star_->height(); 
        else
            point = points.col(i);

        occ = isOccupied(point);

        if(occ)
        {
            // Ignore collisions already represented by an existing constraint.
            for(size_t k = 0; k < direction_[i].size(); ++k)
            {
                if((point - base_point_[i][k]).dot(direction_[i][k]) < resolution)
                {
                    occ = false;
                    break;
                }
            }
        }

        // Create new constraints for an unrepresented collision.
        if(occ)
        {
            flag_new_obs_valid = true;

            // Find the preceding free sample.
            int j;
            for(j = i - 1; j >= 0; --j)
            {
                occ = isOccupied(points.col(j));
                if(!occ)
                {
                    in_id = j;
                    break;
                }
            }
            if(j < 0) // No preceding free sample was found.
            {
                in_id = 0;
            }
            // Find the following free sample.
            for(j = i + 1; j < points.cols(); ++j)
            {
                occ = isOccupied(points.col(j));

                if(!occ) // The first free sample closes the collision segment.
                {
                    out_id = j;
                    break;
                }
            }
            // No following free point means that the local target is in collision.
            if(j >= points.cols())
            {
                force_stop_type_ = ForceStopType::StopForError;
                return false;
            }

            i = j + 1;
            segment_ids.push_back(std::pair<int ,int>(in_id, out_id));
        }
    }

    // Rebuild constraints when a new obstacle invalidates the trajectory.
    double costTime = 0.0;
    if(flag_new_obs_valid)
    {
        std::vector<std::vector<Eigen::Vector3d>> a_star_pathes;
        for(size_t i = 0; i < segment_ids.size(); ++i)
        {
            // Search for a local detour with A*.
            Eigen::Vector3d in, out;
            if(2 == minco_ptr_->getDimensions())
            {
                in << points.col(segment_ids[i].second).x(), points.col(segment_ids[i].second).y(), a_star_->height();
                out << points.col(segment_ids[i].first).x(), points.col(segment_ids[i].first).y(), a_star_->height();
            }
            else
                in = points.col(segment_ids[i].second), out = points.col(segment_ids[i].first);
            
            // Eigen::Vector3d in(points.col(segment_ids.at(i).second)), out(points.col(segment_ids.at(i).first));

            AStarResult ret = a_star_->search(in, out, costTime);
            if(ret == AStarResult::Success)
            {
                auto path = a_star_->path();
                std::reverse(path.begin(), path.end());
                a_star_pathes.push_back(path);
                emitDebugPoints("a_star_path", path);
            }
            else if(ret == AStarResult::Timeout && i + 1 < segment_ids.size())
            {
                segment_ids[i].second = segment_ids[i + 1].second; // Merge with the next segment.
                segment_ids.erase(segment_ids.begin() + i + 1);
                i--;
            }
            else // The detour endpoints are invalid or no path exists.
            {
                segment_ids.erase(segment_ids.begin() + i);
            }
        }// a_star

        // Remove overlaps between consecutive segments.
        for(size_t i = 1; i < segment_ids.size(); ++i)
        {   
            // Split overlapping bounds at their midpoint.
            if(segment_ids[i - 1].second >= segment_ids[i].first)
            {
                double middle = (double)(segment_ids[i - 1].second + segment_ids[i].first) / 2.0;
                segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
                segment_ids[i].first = static_cast<int>(middle + 1.1);
            }
        }

        // Build obstacle base-point/direction constraints for each segment.
        Eigen::Vector3d tmp;
        for(size_t i = 0; i < segment_ids.size(); ++i)
        {
            // Reset per-sample constraint flags.
            for(int j = segment_ids[i].first; j < segment_ids[i].second ; ++j)
                flag_temp_[j] = false;

            // Project trajectory samples toward the A* detour.
            int got_intersection_id = -1;
            for(int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
            {
                got_intersection_id = -1;
                if(i >= a_star_pathes.size())
                    break;

                Eigen::Vector3d ctrl_pts_law;
                Eigen::Vector3d intersection_point; // Orthogonal path intersection.

                if(2 == minco_ptr_->getDimensions())
                {
                    tmp.head(2) = points.col(j);
                    tmp(2) = a_star_->height();
                    ctrl_pts_law.head(2) = points.col(j + 1) - points.col(j - 1);
                    ctrl_pts_law(2) = 0;
                }
                else
                {
                    tmp = points.col(j);
                    ctrl_pts_law = points.col(j + 1) - points.col(j - 1);
                }

                int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id;
                // Walk until the projection dot product changes sign.
                
                double val = (a_star_pathes[i][Astar_id] - tmp).dot(ctrl_pts_law), init_val = val;

                while(true)
                {
                    last_Astar_id = Astar_id;

                    if(val >= 0) // Move backward along the detour.
                    {
                        --Astar_id;
                        if(Astar_id < 0)
                        {
                            break;
                        }
                    }
                    else // Move forward along the detour.
                    {
                        ++Astar_id;
                        if(Astar_id >= (int)a_star_pathes[i].size())
                        {
                            break;
                        }
                    }
                    // Update the projection sign.
                    val = (a_star_pathes[i][Astar_id] - tmp).dot(ctrl_pts_law);

                    if(init_val * val <= 0 && (abs(val) > 0 || abs(init_val) > 0))
                    {
                        // Interpolate the normal-plane intersection.
                        intersection_point =
                            a_star_pathes[i][Astar_id] +
                            ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                            (ctrl_pts_law.dot(tmp - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]))
                            );
                        got_intersection_id = j;
                        break;
                    }
                }

                // Interpolate from the sample toward the detour to find the boundary.
                if(got_intersection_id >= 0)
                {
                    // Measure the ray length used for interpolation.
                    double length = (intersection_point - tmp).norm();
                    if(length > 1e-5) // Reject numerically degenerate rays.
                    {
                        flag_temp_[j] = true;
                        for(double a = length; a >= 0.0; a -= resolution)
                        {
                            bool occ = isOccupied((a / length) * intersection_point + (1 - a / length) * tmp);
                            // Stop at the obstacle boundary or the last interpolation step.
                            if(occ || a < resolution)
                            {
                                if(occ)
                                    a += resolution;
                                base_point_[j].push_back((a / length) * intersection_point + (1 - a / length) * tmp);
                                direction_[j].push_back ((intersection_point - tmp).normalized());
                                break;
                            }
                        }
                    }
                    else
                    {
                        got_intersection_id = -1; // Intersection construction failed.
                    }
                }
            }

            if(got_intersection_id >= 0)
            {
                for(int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)
                {
                    if(!flag_temp_[j])
                    {
                        base_point_[j].push_back(base_point_[j - 1].back());
                        direction_[j].push_back(direction_[j - 1].back());
                    }
                }
                for(int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)
                {
                    if(!flag_temp_[j])
                    {
                        base_point_[j].push_back(base_point_[j + 1].back());
                        direction_[j].push_back(direction_[j + 1].back());
                    }
                }
            }
        }

        force_stop_type_ = ForceStopType::StopForRebound;
        return true;
    }
    return false;
}

bool Ego::allowRebound(const Eigen::MatrixXd &samplePoint) const
{
    if(iter_num_ < 3)
        return false;
    
    double min_product = 1;
    for(int i = 3; i <= samplePoint.cols() - 4; ++i)
    {
        double product = 
        ((samplePoint.col(i) - samplePoint.col(i - 1)).normalized()).dot((samplePoint.col(i + 1) - samplePoint.col(i)).normalized());
        if(product < min_product)
            min_product = product;
    }
    if(min_product < 0.87) // about 30 degree
        return false;

    return true;
}

void Ego::compute3DPose(std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> &pose, const double dt)
{
    ensureConfigured();
    if (dt <= 0.0)
        throw std::invalid_argument("ego: pose sampling interval must be positive");
    pose.clear();
    for(double t = 0; t < minco_ptr_->getTotalDuration(); t += dt)
    {
        Eigen::Vector3d p = minco_ptr_->getState(t);
        Eigen::Quaterniond q = minco_ptr_->getAttitude(t);
        pose.emplace_back(p, q);
    }
}

void Ego::compute2DPose(std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> &pose)
{
    ensureConfigured();
    if (config_.dimension != 2)
        throw std::logic_error("ego: compute2DPose requires a 2D trajectory");
    // Sample positions and velocities at piece boundaries.
    Eigen::MatrixXd wps = minco_ptr_->getSampleStates(1, 0);
    Eigen::MatrixXd vel = minco_ptr_->getSampleStates(1, 1);
    pose.clear();
    pose.reserve(static_cast<std::size_t>(vel.cols()));
    for(int i = 0; i < vel.cols(); ++i)
    {
        const double yaw = std::atan2(vel(1, i), vel(0, i));
        const Eigen::Vector3d position(wps(0, i), wps(1, i), config_.planning_height);
        const Eigen::Quaterniond attitude(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        pose.emplace_back(position, attitude);
    }
}

}  // namespace ego
