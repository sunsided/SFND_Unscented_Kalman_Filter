#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// TODO: Re-enable
// #define MEANS_BY_ROWWISE_SUM

namespace {

    Eigen::VectorXd buildSigmaPointWeightsVector(std::size_t n, double lambda) {
        auto weights_ = VectorXd(2 * n + 1);

        const auto divisor = lambda + n;
        const auto weight_0 = lambda / divisor;
        const auto weight_n = 1 / (2 * divisor);

        // All weights are identical, except for the first one.
        weights_.fill(weight_n);
        weights_(0) = weight_0;

        return weights_;
    }

    double normalizeAngle(double radians) {
        static const auto pi2 = 2.0 * M_PI;
        while (radians > M_PI) radians -= pi2;
        while (radians < -M_PI) radians += pi2;
        return radians;
    }
}

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // initial state vector
    x_ = VectorXd(5);

    // initial covariance matrix
    P_ = MatrixXd(5, 5);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    const auto expected_a_max = 4; // m/s^2 (e.g. 6 m/s^2 for inner-city dynamic driving)
    std_a_ = 0.5 * expected_a_max; // m/s^2 acceleration noise

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = M_PI_4; // ±45°/s^2

    /**
     * DO NOT MODIFY measurement noise values below.
     * These are provided by the sensor manufacturer.
     */

    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;

    /**
     * End DO NOT MODIFY section for measurement noise values
     */

    /**
     * See ukf.h for other member properties.
     * Hint: one or more values initialized above might be wildly off...
     */

    is_initialized_ = false;

    // Number of states (px, py, v, ψ, ̇ψ)
    n_x_ = 5;

    // Number of states in augmented vector (px, py, v, ψ, ̇ψ, ν, ̇ν)
    n_aug_ = 7;

    // Identity matrix for the state.
    I_ = MatrixXd::Identity(n_x_, n_x_);

    // Initial state vector - will be initialized at first measurement.
    x_ = VectorXd(n_x_);

    // Initial state covariance guesstimate.
    P_ = MatrixXd(n_x_, n_x_);

    // We will initialize the covariance matrix according to some basic considerations:
    //
    // - The initially measured X and Y positions will be good regardless of the sensor.
    // - We will have a better chance at measuring X distance than at measuring distances
    //   to the side due to the nature of the Radar.
    // - The LiDAR will not give us a useful velocity estimate, but the Radar will.
    //   Since we're measuring relative velocity, the assumption of zero relative velocity
    //   is reasonable unless the ego car moves at a vastly different speed.
    // - LiDAR will not be able to provide direct orientation or turn rate estimates,
    //   and we don't have a good prior. We could assume that the targets move in the
    //   same direction as we do, but this throws off the initial y velocity estimate.
    //
    // We could, of course, also initialize the covariance matrix as soon as the first
    // measurement arrives, since we then have some higher trust in individual values.
    P_ << 1.00, 0.00, 0.00, 0.00, 0.00,
          0.00, 0.50, 0.00, 0.00, 0.00,
          0.00, 0.00, 1.00, 0.00, 0.00,
          0.00, 0.00, 0.00, 0.05, 0.00,
          0.00, 0.00, 0.00, 0.00, 0.02;

    // Sigma point spreading parameter design rule
    lambda_ = 3 - static_cast<double>(n_aug_);

    // The number of sigma points (including noise augmentation).
    n_sigma_points_ = 2 * n_aug_ + 1;

    // Sigma point weights vector.
    weights_ = buildSigmaPointWeightsVector(n_aug_, lambda_);

    // Predicted sigma points matrix 𝓧
    // For 5 predicted states and 7 augmented states (having 2x7+1 = 15 sigma points),
    // this needs to be a 5x15 matrix; i.e., it projects the augmented
    // sigma points into the 1 x 5 state vector.
    Xsig_pred_ = MatrixXd(n_x_, n_sigma_points_);

    // LiDAR measurement projection matrices.
    H_lidar_ = MatrixXd(2, n_x_);
    H_lidar_ << 1, 0, 0, 0, 0, // px
                0, 1, 0, 0, 0; // py
    Ht_lidar_ = H_lidar_.transpose();

    // LiDAR noise covariance matrix.
    R_laser_ = MatrixXd(2, 2);
    R_laser_ << std_laspx_ * std_laspx_, 0,
                0, std_laspy_ * std_laspy_;

    // Number of measurements for Radar (r, φ, ̇φ)
    n_z_radar_ = 3;

    // Radar noise covariance matrix.
    R_radar_ = MatrixXd(n_z_radar_, n_z_radar_);
    R_radar_ << std_radr_ * std_radr_, 0, 0,
                0, std_radphi_ * std_radphi_, 0,
                0, 0, std_radrd_ * std_radrd_;
}

UKF::~UKF() = default;

void UKF::Prediction(double delta_t) {
    // Generate sigma points from the current process state, augmented with the process noise model.
    const auto Xsig_aug = GenerateAugmentedSigmaPoints();

    // Use the augmented sigma points to predict (project!) the sigma points for the next time step.
    PredictSigmaPoints(Xsig_aug, delta_t);

    // From the predicted sigma points, obtain the new state estimate.
    PredictStateMeanAndCovarianceFromSigmaPoints();
}

MatrixXd UKF::GenerateAugmentedSigmaPoints() {
    // We're going to augment the state vectors with the process noise parameters.
    VectorXd x_aug(n_aug_);
    MatrixXd P_aug(n_aug_, n_aug_);

    // Extend the process state with the process noise.
    x_aug.head(5) = x_;
    x_aug(5) = 0; // acceleration noise has zero mean
    x_aug(6) = 0; // yaw acceleration noise has zero mean

    // Extend the process covariance with the noise covariance.
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5, 5) = P_;
    P_aug(5, 5) = std_a_ * std_a_;
    P_aug(6, 6) = std_yawdd_ * std_yawdd_;

    // Square root of state covariance matrix via cholesky decomposition
    // Small caveat: LL^T requires a positive definite matrix, but in general
    //               covariance matrices are only positive (i.e. non-negative) semi-definite.
    //               (Nice explanation why here: https://stats.stackexchange.com/a/144469/26843)
    MatrixXd A = P_aug.llt().matrixL();

    // Create augmented sigma points.
    MatrixXd Xsig_aug(n_aug_, n_sigma_points_);

    // Constant coefficient for all sigma points.
    // TODO: Extract constant value; may introduce a class member here.
    const auto coeff_ = std::sqrt(lambda_ + n_aug_);

    // The first sigma point is just the state mean and the remaining points
    // are symmetrically distributed around it.
    Xsig_aug.col(0) = x_aug;
    for (int i = 0; i < n_aug_; ++i) {
        // Note that both the coefficient and A independently
        // had their square root taken before.
        const auto distance = coeff_ * A.col(i);
        Xsig_aug.col(i + 1)          = x_aug + distance;
        Xsig_aug.col(i + 1 + n_aug_) = x_aug - distance;
    }

    return Xsig_aug;
}

void UKF::PredictSigmaPoints(const Eigen::MatrixXd& Xsig_aug, const double& delta_t) {
    // This method applies a constant turn rate and velocity (CTRV) model.
    const auto delta_t_sq = delta_t * delta_t;

    for (auto s = 0; s < n_sigma_points_; ++s) {
        // State values extracted for readability.
        const auto p_x      = Xsig_aug(0, s);
        const auto p_y      = Xsig_aug(1, s);
        const auto v        = Xsig_aug(2, s);
        const auto yaw      = Xsig_aug(3, s);
        const auto yawd     = Xsig_aug(4, s);
        const auto nu_a     = Xsig_aug(5, s);
        const auto nu_yawdd = Xsig_aug(6, s);

        // Predicted velocity and yaw rate.
        // Since we're using a CTRV model, these values are constant
        // over time and only affected by noise.
        auto v_p    = v;
        auto yawd_p = yawd;

        // Predicted position states; we're integrating,
        // so we initialize with the current value.
        auto px_p = p_x;
        auto py_p = p_y;

        // A target moving in a straight line has zero turn rate,
        // so we need to make sure to avoid a division by zero.
        const auto yaw_rate_epsilon = 0.001;
        if (std::abs(yawd) > yaw_rate_epsilon) {
            // Target is turning.
            px_p += v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
            py_p += v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
        } else {
            // Target moves in straight line.
            px_p += v * delta_t * cos(yaw);
            py_p += v * delta_t * sin(yaw);
        }

        // Predict yaw state.
        auto yaw_p = yaw + yawd * delta_t;

        // Add noise.
        px_p   += 0.5 * nu_a * delta_t_sq * cos(yaw);
        py_p   += 0.5 * nu_a * delta_t_sq * sin(yaw);
        v_p    += nu_a * delta_t;
        yaw_p  += 0.5 * nu_yawdd * delta_t * delta_t;
        yawd_p += nu_yawdd * delta_t;

        // Update the state vector.
        Xsig_pred_(0, s) = px_p;
        Xsig_pred_(1, s) = py_p;
        Xsig_pred_(2, s) = v_p;
        Xsig_pred_(3, s) = yaw_p;
        Xsig_pred_(4, s) = yawd_p;
    }
}

void UKF::PredictStateMeanAndCovarianceFromSigmaPoints() {
    // predicted state mean
#ifdef MEANS_BY_ROWWISE_SUM
    x_ = (Xsig_pred_ * weights_).rowwise().sum();
#else
    x_.fill(0.0);
    for (int i = 0; i < n_sigma_points_; ++i) {
        x_ += weights_(i) * Xsig_pred_.col(i);
    }
#endif

    // predicted state covariance matrix
    P_.fill(0.0);
    for (int i = 0; i < n_sigma_points_; ++i) {
        // State prediction residuals.
        VectorXd delta = Xsig_pred_.col(i) - x_;

        // Keep yaw angle within -π .. π.
        delta(3) = normalizeAngle(delta(3));

        P_ += weights_(i) * delta * delta.transpose();
    }
}

void UKF::ProcessMeasurement(const MeasurementPackage& meas_package) {
    if (!is_initialized_) {
        if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            // set the state with the initial location and zero velocity
            x_ << meas_package.raw_measurements_[0],
                  meas_package.raw_measurements_[1],
                  0.0,
                  0.0,
                  0.0;
        } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            const auto r     = meas_package.raw_measurements_[0];
            const auto phi   = meas_package.raw_measurements_[1];
            const auto r_dot = meas_package.raw_measurements_[2];
            x_ << r * cos(phi),
                  r * sin(phi),
                  r_dot,
                  phi,
                  0.0;

            // We might ...
            // P_(3, 3) = 1;
            // P_(4, 4) = 1;
        }

        time_us_ = meas_package.timestamp_;
        is_initialized_ = true;
        return;
    }

    // compute the time elapsed between the current and previous measurements
    // dt - expressed in seconds
    constexpr auto microseconds_per_second = 1000000.0;
    const auto dt = static_cast<double>(meas_package.timestamp_ - time_us_) / microseconds_per_second;

    // dt can be null if we find more than one measurement in the same timestamp
    // assert(dt > 0);

    time_us_ = meas_package.timestamp_;

    Prediction(dt);

    if (use_laser_ && (meas_package.sensor_type_ == MeasurementPackage::LASER)) {
        const auto nis = UpdateLidar(meas_package);
        nisScoresLidar.emplace_back(std::make_pair(meas_package.timestamp_, nis));

    } else if (use_radar_ && (meas_package.sensor_type_ == MeasurementPackage::RADAR)) {
        const auto nis = UpdateRadar(meas_package);
        nisScoresRadar.emplace_back(std::make_pair(meas_package.timestamp_, nis));
    }
}

double UKF::UpdateLidar(const MeasurementPackage& meas_package) {
    /**
     * Use LiDAR data to update the belief about the object's position.
     * Modify the state vector, x_, and covariance, P_.
     */

    // Predict the LiDAR measurement.
    const VectorXd z = meas_package.raw_measurements_;
    const VectorXd z_pred = H_lidar_ * x_;

    // Determine measurement error.
    const VectorXd y = z - z_pred;

    // Project system covariance into measurement space.
    const MatrixXd S = H_lidar_ * P_ * Ht_lidar_ + R_laser_;

    // Determine Kalman gain
    const MatrixXd K = P_ * Ht_lidar_ * S.inverse();

    // Apply Kalman filter magic sauce.
    x_ = x_ + (K * y);
    P_ = (I_ - K * H_lidar_) * P_;

    // Determine Normalized Innovation Squared (NIS) score.
    const auto nis = y.transpose() * S.inverse() * y;
    return nis;
}

double UKF::UpdateRadar(const MeasurementPackage& meas_package) {
    /**
     * Use Radar data to update the belief about the object's position.
     * Modify the state vector, x_, and covariance, P_.
     */

    // Radar updates are a bit more involved than the LiDAR updates,
    // since LiDAR is linear but Radar isn't. We thus use the same
    // UKF techniques to perform a nonlinear measurement update,
    // then incorporate it into the state update.

    // Measurement mean and covariance.
    VectorXd z_pred = VectorXd(n_z_radar_);
    MatrixXd S = MatrixXd(n_z_radar_, n_z_radar_);

    // Measurement sigma points 𝓩.
    MatrixXd Zsig = MatrixXd(n_z_radar_, n_sigma_points_);

    PredictRadarMeasurement(z_pred, S, Zsig);
    return UpdateStateFromRadar(meas_package, z_pred, S, Zsig);
}

void UKF::PredictRadarMeasurement(VectorXd &z_pred, MatrixXd &S, MatrixXd &Zsig) {
    // Transform sigma points into measurement space
    for (int s = 0; s < n_sigma_points_; ++s) {
        const auto p_x = Xsig_pred_(0, s);
        const auto p_y = Xsig_pred_(1, s);
        const auto v   = Xsig_pred_(2, s);
        const auto yaw = Xsig_pred_(3, s);

        const auto v1 = cos(yaw) * v;
        const auto v2 = sin(yaw) * v;

        // Apply measurement model.
        const auto distance = sqrt(p_x * p_x + p_y * p_y);
        Zsig(0, s) = distance;                          // r
        Zsig(1, s) = atan2(p_y, p_x);                   // φ (phi)
        Zsig(2, s) = (p_x * v1 + p_y * v2) / distance;  // ̇φ (phi_dot)
    }

    // mean predicted measurement
#ifdef MEANS_BY_ROWWISE_SUM
    z_pred = (Zsig * weights_).rowwise().sum();
#else
    z_pred.fill(0.0);
    for (int i = 0; i < n_sigma_points_; ++i) {
        z_pred += + weights_(i) * Zsig.col(i);
    }
#endif

    // Determine innovation covariance matrix S
    S.fill(0.0);
    for (int i = 0; i < n_sigma_points_; ++i) {
        // Measurement prediction residuals.
        VectorXd delta = Zsig.col(i) - z_pred;

        // Keep phi angle within -π .. π.
        delta(1) = normalizeAngle(delta(1));

        S += weights_(i) * delta * delta.transpose();
    }

    // Add measurement noise covariance.
    S += R_radar_;
}

double UKF::UpdateStateFromRadar(const MeasurementPackage& meas_package, const VectorXd &z_pred, const MatrixXd &S,
                               const MatrixXd &Zsig) {
    // Cross-correlation between sigma points in state space and measurement space,
    // usd to determine the Kalman gain.
    MatrixXd Tc(n_x_, n_z_radar_);

    // calculate cross correlation matrix
    Tc.fill(0.0);
    for (int s = 0; s < n_sigma_points_; ++s) {
        // State and measurement prediction residuals.
        VectorXd xs_diff = Xsig_pred_.col(s) - x_;
        VectorXd zs_diff = Zsig.col(s) - z_pred;

        // Keep yaw and phi angles within -π .. π.
        xs_diff(3) = normalizeAngle(xs_diff(3));
        zs_diff(1) = normalizeAngle(zs_diff(1));

        Tc += weights_(s) * xs_diff * zs_diff.transpose();
    }

    // Measurement error.
    VectorXd z_diff = meas_package.raw_measurements_ - z_pred;

    // Normalizing angles once again ...
    z_diff(1) = normalizeAngle(z_diff(1));

    // Determine Kalman gain.
    auto K = Tc * S.inverse();

    // Apply the Kalman filter magic sauce.
    x_ += K * z_diff;
    P_ -= K * S * K.transpose();

    // Determine Normalized Innovation Squared (NIS) score.
    const auto nis = z_diff.transpose() * S.inverse() * z_diff;
    return nis;
}

