#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"

class UKF {
public:
    /**
     * Constructor
     */
    UKF();

    /**
     * Destructor
     */
    virtual ~UKF();

    /**
     * ProcessMeasurement
     * @param meas_package The latest measurement data of either radar or laser
     */
    void ProcessMeasurement(const MeasurementPackage& meas_package);

    /**
     * Predicts sigma points, the state, and the state covariance
     * matrix
     * @param delta_t Time between k and k+1 in s
     */
    void Prediction(double delta_t);

    /**
     * Updates the state and the state covariance matrix using a laser measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateLidar(const MeasurementPackage& meas_package);

    /**
     * Updates the state and the state covariance matrix using a radar measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateRadar(const MeasurementPackage& meas_package);


    // initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;

    // if this is false, laser measurements will be ignored (except for init)
    bool use_laser_;

    // if this is false, radar measurements will be ignored (except for init)
    bool use_radar_;

    // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    Eigen::VectorXd x_;

    // state covariance matrix
    Eigen::MatrixXd P_;

    // State mean / covariance identity matrix.
    Eigen::MatrixXd I_;

    // predicted sigma points matrix 𝓧
    Eigen::MatrixXd Xsig_pred_;

    // LiDAR measurement projection matrix
    Eigen::MatrixXd H_lidar_;
    Eigen::MatrixXd Ht_lidar_;

    // LiDAR measurement noise covariance matrix
    Eigen::MatrixXd R_laser_;

    // Radar measurement noise covariance matrix
    Eigen::MatrixXd R_radar_;

    // time when the state is true, in us
    long long time_us_;

    // Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a_;

    // Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd_;

    // Laser measurement noise standard deviation position1 in m
    double std_laspx_;

    // Laser measurement noise standard deviation position2 in m
    double std_laspy_;

    // Radar measurement noise standard deviation radius in m
    double std_radr_;

    // Radar measurement noise standard deviation angle in rad
    double std_radphi_;

    // Radar measurement noise standard deviation radius change in m/s
    double std_radrd_;

    // Weights of sigma points
    Eigen::VectorXd weights_;

    // State dimension
    std::size_t n_x_;

    // Augmented state dimension
    std::size_t n_aug_;

    // The number of sigma points
    std::size_t n_sigma_points_;

    // Sigma point spreading parameter
    double lambda_;

    // Radar measurement dimension (r, phi, r_dot)
    std::size_t n_z_radar_;

private:

    void PredictRadarMeasurement(Eigen::VectorXd &z_pred, Eigen::MatrixXd &S, Eigen::MatrixXd &Zsig);

    void UpdateStateFromRadar(const MeasurementPackage& meas_package, const Eigen::VectorXd &z_pred,
                              const Eigen::MatrixXd &S, const Eigen::MatrixXd &Zsig);

    Eigen::MatrixXd GenerateAugmentedSigmaPoints();

    void PredictSigmaPoints(const Eigen::MatrixXd& Xsig_aug, const double& delta_t);

    void PredictStateMeanAndCovarianceFromSigmaPoints();
};

#endif  // UKF_H
