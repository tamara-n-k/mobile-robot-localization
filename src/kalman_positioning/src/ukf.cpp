#include "kalman_positioning/ukf.hpp"
#include <iostream>
#include <map>


/**
 * STUDENT ASSIGNMENT: Unscented Kalman Filter Implementation
 * 
 * This file contains placeholder implementations for the UKF class methods.
 * Students should implement each method according to the UKF algorithm.
 * 
 * Reference: Wan, E. A., & Van Der Merwe, R. (2000). 
 * "The Unscented Kalman Filter for Nonlinear Estimation"
 */

// ============================================================================
// CONSTRUCTOR
// ============================================================================

/**
 * @brief Initialize the Unscented Kalman Filter
 * 
 * STUDENT TODO:
 * 1. Initialize filter parameters (alpha, beta, kappa, lambda)
 * 2. Initialize state vector x_ with zeros
 * 3. Initialize state covariance matrix P_ 
 * 4. Set process noise covariance Q_
 * 5. Set measurement noise covariance R_
 * 6. Calculate sigma point weights for mean and covariance
 */
UKF::UKF(double process_noise_xy, double process_noise_theta,
         double measurement_noise_xy)//, int num_landmarks)
    : nx_(5), nz_(2) {
    
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================

    x_ = Eigen::VectorXd::Zero(nx_);
    //Q: uncertainty in motion model = diag(process_noise_xy, process_noise_xy, process_noise_theta, 0, 0)
    //R: sensor measurement noise = diag(measurement_noise_xy, measurement_noise_xy)
    //P: covariance matrix of state vector
    P_ = Eigen::MatrixXd::Identity(nx_, nx_);
    Q_ = Eigen::MatrixXd::Zero(nx_, nx_);
    Q_(0,0) = process_noise_xy;
    Q_(1,1) = process_noise_xy;
    Q_(2,2) = process_noise_theta;
    R_ = Eigen::MatrixXd::Zero(nz_, nz_);
    R_(0,0) = measurement_noise_xy;
    R_(1,1) = measurement_noise_xy;
    
    //calculate lambda, gamma and weights
    lambda_ = ALPHA*ALPHA*(nx_ + KAPPA) - nx_;
    gamma_ = std::sqrt(nx_ + lambda_);


    Wm_ = std::vector<double>(2*nx_ + 1);
    Wc_ = std::vector <double>(2*nx_+ 1);
    Wm_[0] = lambda_/(nx_ + lambda_);
    Wc_[0] = (1 - ALPHA*ALPHA + BETA) + Wm_[0];

    for (int i = 1; i < 2 * nx_ + 1; i++)
    {
    Wm_[i] =  1.0f / (2.0f * (nx_ + lambda_));
    Wc_[i] = Wm_[i];
    }


}


// ============================================================================
// SIGMA POINT GENERATION
// ============================================================================

/**
 * @brief Generate sigma points from mean and covariance
 * 
 * STUDENT TODO:
 * 1. Start with the mean as the first sigma point
 * 2. Compute Cholesky decomposition of covariance
 * 3. Generate 2*n symmetric sigma points around the mean
 */
std::vector<Eigen::VectorXd> UKF::generateSigmaPoints(const Eigen::VectorXd& mean,
                                                       const Eigen::MatrixXd& cov) {
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================
    const int n = mean.size();

    std::vector<Eigen::VectorXd> sigma_points(2 * n + 1);
    sigma_points[0] = mean;
    Eigen::MatrixXd L = cov.llt().matrixL();

    for (int i = 0; i < n; i++) {
    sigma_points[i + 1]     = mean + gamma_ * L.col(i);
    sigma_points[i + 1 + n] = mean - gamma_ * L.col(i);

    }
    return sigma_points;
}

// ============================================================================
// PROCESS MODEL
// ============================================================================

/**
 * @brief Apply motion model to a state vector
 * 
 * STUDENT TODO:
 * 1. Updates position: x' = x + dx, y' = y + dy
 * 2. Updates orientation: theta' = theta + dtheta (normalized)
 * 3. Updates velocities: vx' = dx/dt, vy' = dy/dt
 */
Eigen::VectorXd UKF::processModel(const Eigen::VectorXd& state, double dt,
                                  double dx, double dy, double dtheta) {
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================
    Eigen::VectorXd new_state = state;
    new_state[0] += dx;
    new_state[1] += dy;
    new_state[2] = normalizeAngle(new_state[2] + dtheta);
    new_state[3] = dx/dt;
    new_state[4] = dy/dt;
    
    
    return new_state;
}

// ============================================================================
// MEASUREMENT MODEL
// ============================================================================

/**
 * @brief Predict measurement given current state and landmark
 * 
 * STUDENT TODO:
 * 1. Calculate relative position: landmark - robot position
 * 2. Transform to robot frame using robot orientation
 * 3. Return relative position in robot frame
 */
Eigen::Vector2d UKF::measurementModel(const Eigen::VectorXd& state, int landmark_id) {
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================
    
    if (landmarks_.find(landmark_id) == landmarks_.end()) {
        std::cout << "no landmark" << std::endl;

        return Eigen::Vector2d::Zero();
    }
    std::pair landmark_pos = landmarks_[landmark_id];
    Eigen::Vector2d rel_pos(landmark_pos.first - state[0], landmark_pos.second - state[1]);

    double theta = state[2];
    Eigen::Matrix2d R;
    R << std::cos(theta), -std::sin(theta),
    std::sin(theta),  std::cos(theta);
    
    Eigen::Vector2d measurement = R.transpose() * rel_pos;

    return measurement;
}

// ============================================================================
// ANGLE NORMALIZATION
// ============================================================================

double UKF::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// ============================================================================
// PREDICTION STEP
// ============================================================================

/**
 * @brief Kalman Filter Prediction Step (Time Update)
 * 
 * STUDENT TODO:
 * 1. Generate sigma points from current state and covariance
 * 2. Propagate each sigma point through motion model
 * 3. Calculate mean and covariance of predicted sigma points
 * 4. Add process noise
 * 5. Update state and covariance estimates
 */
void UKF::predict(double dt, double dx, double dy, double dtheta) {
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================
    std::vector<Eigen::VectorXd> sigma_points = generateSigmaPoints(x_, P_);
    const int num_sigma = sigma_points.size();

    std::vector<Eigen::VectorXd> sigma_points_pred(num_sigma);
    for (int i = 0; i < num_sigma; i++) //generate sigma points through motion model
    {
        sigma_points_pred[i] = processModel(sigma_points[i], dt, dx, dy, dtheta);
        sigma_points_pred[i][2] = normalizeAngle(sigma_points_pred[i][2]);

    }
    //compute predicted mean
    Eigen::VectorXd x_pred = Eigen::VectorXd::Zero(nx_);

    double sin_sum = 0.0;
    double cos_sum = 0.0;

    for (int i = 0; i < num_sigma; ++i) {
        x_pred[0] += Wm_[i] * sigma_points_pred[i][0];
        x_pred[1] += Wm_[i] * sigma_points_pred[i][1];

        sin_sum += Wm_[i] * std::sin(sigma_points_pred[i][2]);
        cos_sum += Wm_[i] * std::cos(sigma_points_pred[i][2]);

        x_pred[3] += Wm_[i] * sigma_points_pred[i][3];
        x_pred[4] += Wm_[i] * sigma_points_pred[i][4];
    }

    x_pred[2] = std::atan2(sin_sum, cos_sum);  // circular mean angle

    //compute predicted covariance
    Eigen::MatrixXd P_pred = Eigen::MatrixXd::Zero(nx_, nx_);//cov
    for (int i = 0; i < 2*nx_ + 1; i++) 
    {
        Eigen::VectorXd diff = sigma_points_pred[i] - x_pred;
        diff[2] = normalizeAngle(diff[2]);
        P_pred += Wc_[i] * diff * diff.transpose();
    }

    P_pred += Q_;//add process noise

    //update state & cov estimates
    x_ = x_pred;
    x_[2] = normalizeAngle(x_[2]);  // ensure state angle is clean
    P_ = P_pred;

    std::cout << "Prediction step done." << std::endl;


}

// ============================================================================
// UPDATE STEP
// ============================================================================

/**
 * @brief Kalman Filter Update Step (Measurement Update)
 * 
 * STUDENT TODO:
 * 1. Generate sigma points
 * 2. Transform through measurement model
 * 3. Calculate predicted measurement mean
 * 4. Calculate measurement and cross-covariance
 * 5. Compute Kalman gain
 * 6. Update state with innovation
 * 7. Update covariance
 */
void UKF::update(const std::vector<std::tuple<int,double,double>>& landmark_observations)
{
    int N = landmark_observations.size();
    int dim_z = 2 * N;

    // 1. Generate sigma points
    std::vector<Eigen::VectorXd> X_sigma = generateSigmaPoints(x_, P_);
    int num_sigma = X_sigma.size();

    // 2. Predict measurement mean
    std::vector<Eigen::VectorXd> Z_sigma(num_sigma, Eigen::VectorXd(dim_z));
    Eigen::VectorXd z_pred = Eigen::VectorXd::Zero(dim_z);

    for (int i = 0; i < num_sigma; i++) {
        int row = 0;
        for (const auto& obs : landmark_observations) {
            int landmark_id = std::get<0>(obs);

            Eigen::Vector2d z_i = measurementModel(X_sigma[i], landmark_id);

            Z_sigma[i].segment<2>(row) = z_i;
            row += 2;
        }
        z_pred += Wm_[i] * Z_sigma[i];
    }
    // 3. Compute measurement covariance S and cross-covariance Tc
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(dim_z, dim_z);
    Eigen::MatrixXd Tc = Eigen::MatrixXd::Zero(nx_, dim_z);

    for (int i = 0; i < num_sigma; i++) {
        Eigen::VectorXd z_diff = Z_sigma[i] - z_pred;

        Eigen::VectorXd x_diff = X_sigma[i] - x_;
        x_diff(2) = normalizeAngle(x_diff(2));

        S  += Wc_[i] * z_diff * z_diff.transpose();
        Tc += Wc_[i] * x_diff * z_diff.transpose();
    }

    // Measurement noise block (dim_z × dim_z)
    Eigen::MatrixXd Rb = Eigen::MatrixXd::Zero(dim_z, dim_z);
    for (int i = 0; i < N; i++)
        Rb.block<2,2>(2*i, 2*i) = R_;      // your existing 2×2 noise

    S += Rb;

    // 4. Kalman gain
    Eigen::MatrixXd K = Tc * S.inverse();

    // 5. Construct measurement vector z
    Eigen::VectorXd z(dim_z);
    {
        int row = 0;
        for (const auto& obs : landmark_observations) {
            double meas_x = std::get<1>(obs);
            double meas_y = std::get<2>(obs);

            z(row + 0) = meas_x;
            z(row + 1) = meas_y;
            row += 2;
        }
    }

    // 6. Update state
    Eigen::VectorXd z_innov = z - z_pred;
    x_ = x_ + K * z_innov;

    // 7. Update covariance
    P_ = P_ - K * S * K.transpose();

    x_(2) = normalizeAngle(x_(2));
}

// ============================================================================
// LANDMARK MANAGEMENT
// ============================================================================

void UKF::setLandmarks(const std::map<int, std::pair<double, double>>& landmarks) {
    landmarks_ = landmarks;
}

bool UKF::hasLandmark(int id) const {
    return landmarks_.find(id) != landmarks_.end();
}
