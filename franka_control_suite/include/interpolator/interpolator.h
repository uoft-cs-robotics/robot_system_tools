#ifndef ROBOT_JOINT_INTERPOLATOR
#define ROBOT_JOINT_INTERPOLATOR

#include <iostream>
#include <vector>
#include <string>
#include <stdlib.h>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Dense>

// This class calculates the coefficients of a quintic polynomial
// in order to smoothly interpolate between two points with constraints
// on position, velocity, and acceleration at the end points.
class QuinticInterpolator {
    public:
        // Default constructor. Do not init.
        QuinticInterpolator() {};

        // Constructor. Sets interpolation timestep.
        QuinticInterpolator(double interpolation_timestep);

        // Sets the window of time that the interpolation will happen
        // over.
        void setInterpolationTimeWindow(double interpolation_window);

        // Set target endpoints. If velocity and acceleration are not
        // specified, they will be set to 0s.
        void setTargetEndpoints(Eigen::VectorXd start_position,
                           Eigen::VectorXd end_position,
                           Eigen::VectorXd start_velocity = Eigen::VectorXd::Zero(999),
                           Eigen::VectorXd end_velocity = Eigen::VectorXd::Zero(999),
                           Eigen::VectorXd start_acceleration = Eigen::VectorXd::Zero(999),
                           Eigen::VectorXd end_acceleration = Eigen::VectorXd::Zero(999));

        // Steps the interpolator and calculates next interpolated point.
        void step(std::vector<double>& inter_position,
                  std::vector<double>& inter_velocity,
                  std::vector<double>& inter_acceleration);

    private:
        // End points.
        std::vector<Eigen::VectorXd> endpoints_;
        // Calculates polynomial coefficients.
        void calcPolynomialCoefficients();

        // Calculate time matrix
        void calcTimeMatrix();

        // Polynomial coefficients.
        std::vector<Eigen::VectorXd> coefficients_;
        Eigen::MatrixXd time_matrix_;

        double time_, interpolation_timestep_, interpolation_window_;
};


#endif
