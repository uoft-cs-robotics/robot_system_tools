#include "interpolator/interpolator.h"

QuinticInterpolator::QuinticInterpolator(double interpolation_timestep)
    : interpolation_timestep_(interpolation_timestep) {};

void QuinticInterpolator::setInterpolationTimeWindow(double interpolation_window) {
    interpolation_window_ = interpolation_window;

    // Update time matrix
    calcTimeMatrix();
}

// Set target endpoints.
void QuinticInterpolator::setTargetEndpoints(Eigen::VectorXd start_position,
        Eigen::VectorXd end_position,
        Eigen::VectorXd start_velocity,
        Eigen::VectorXd end_velocity,
        Eigen::VectorXd start_acceleration,
        Eigen::VectorXd end_acceleration) {
    // Reset interpolation time.
    time_ = interpolation_timestep_;

    // Set velocities and accelerations to 0 if not set.
    if (start_velocity.size() == 999) {
        start_velocity.setZero(start_position.size());
    }
    if (end_velocity.size() == 999) {
        start_velocity.setZero(start_position.size());
    }
    if (start_acceleration.size() == 999) {
        start_acceleration.setZero(start_position.size());
    }
    if (end_acceleration.size() == 999) {
        start_acceleration.setZero(start_position.size());
    }

    // Set endpoints.
    endpoints_.resize(start_position.size());
    for (int i=0; i < endpoints_.size(); i++) {
        endpoints_[i].resize(6);
        endpoints_[i][0] = start_position(i);
        endpoints_[i][1] = start_velocity(i);
        endpoints_[i][2] = start_acceleration(i);
        endpoints_[i][3] = end_position(i);
        endpoints_[i][4] = end_velocity(i);
        endpoints_[i][5] = end_acceleration(i);
    }

    // Calculate coefficients
    coefficients_.resize(start_position.size());
    calcPolynomialCoefficients();
}

void QuinticInterpolator::step(std::vector<double>& inter_position,
          std::vector<double>& inter_velocity,
          std::vector<double>& inter_acceleration) {
    // Resize
    inter_position.resize(inter_position.size());
    inter_velocity.resize(inter_position.size());
    inter_acceleration.resize(inter_position.size());

    for (int i = 0; i < inter_position.size(); i++) {
        // Update position
        inter_position[i] = coefficients_[i][0] +
                            coefficients_[i][1] * time_ +
                            coefficients_[i][2] * pow(time_,2) +
                            coefficients_[i][3] * pow(time_,3) +
                            coefficients_[i][4] * pow(time_,4) +
                            coefficients_[i][5] * pow(time_,5);

        // Update velocity
        inter_velocity[i] = coefficients_[i][1] +
                            2. * coefficients_[i][2] * time_ +
                            3. * coefficients_[i][3] * pow(time_,2) +
                            4. * coefficients_[i][4] * pow(time_,3) +
                            5. * coefficients_[i][5] * pow(time_,4);

        // Update acceleration
        inter_acceleration[i] = 2. * coefficients_[i][2] +
                                6. * coefficients_[i][3] * time_ +
                                12. * coefficients_[i][4] * pow(time_,2) +
                                20. * coefficients_[i][5] * pow(time_,3);
    }

    // Set time.
    time_ += interpolation_timestep_;
}

void QuinticInterpolator::calcPolynomialCoefficients() {
    for (int i=0; i < coefficients_.size(); i++) {
        coefficients_[i] = time_matrix_.inverse() * endpoints_[i];
    }
}

void QuinticInterpolator::calcTimeMatrix() {
    time_matrix_.setZero(6, 6); // dim=5 for quintic spline.
    time_matrix_(0,0) = 1.;
    time_matrix_(1,1) = 1.;
    time_matrix_(2,2) = 2.;

    time_matrix_(3,0) = 1.;
    time_matrix_(3,1) = interpolation_window_;
    time_matrix_(3,2) = pow(interpolation_window_, 2);
    time_matrix_(3,3) = pow(interpolation_window_, 3);
    time_matrix_(3,4) = pow(interpolation_window_, 4);
    time_matrix_(3,5) = pow(interpolation_window_, 5);

    time_matrix_(4,1) = 1.;
    time_matrix_(4,2) = 2. * interpolation_window_;
    time_matrix_(4,3) = 3. * pow(interpolation_window_, 2);
    time_matrix_(4,4) = 4. * pow(interpolation_window_, 3);
    time_matrix_(4,5) = 5. * pow(interpolation_window_, 4);

    time_matrix_(5,2) = 2.;
    time_matrix_(5,3) = 6. * pow(interpolation_window_, 1);
    time_matrix_(5,4) = 12. * pow(interpolation_window_, 2);
    time_matrix_(5,5) = 20. * pow(interpolation_window_, 3);
};

