#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <stdlib.h>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Dense>


class MinJerkInterpolator {
public:
    MinJerkInterpolator() {};
    MinJerkInterpolator(double dt, double duration);

    void setTargetEndpoints(
        Eigen::VectorXd start,
        Eigen::VectorXd end
    );

    void step(
        std::vector<double>& interp,
        std::vector<double>& interpDot
    );

private:
    double dt_;
    double t_;
    double duration_;
    Eigen::VectorXd start_;
    Eigen::VectorXd end_;
};