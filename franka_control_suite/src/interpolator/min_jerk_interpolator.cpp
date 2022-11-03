#include "interpolator/min_jerk_interpolator.h"
#include <iostream>


MinJerkInterpolator::MinJerkInterpolator(double dt, double duration) 
    : dt_(dt), duration_(duration) {};

void MinJerkInterpolator::setTargetEndpoints(
        Eigen::VectorXd start,
        Eigen::VectorXd end
) {
    start_.setZero(7);
    end_.setZero(7);
    for(int i = 0; i < 7; i++) {
        start_[i] = start[i];
        end_[i] = end[i];
    }
    t_ = 0;
}

void MinJerkInterpolator::step(
    std::vector<double>& interp,
    std::vector<double>& interpDot
) {
    for(int i = 0; i < 7; i++) {
        interp[i] = start_[i] + (end_[i] - start_[i]) * (
            10 * pow(t_ / duration_, 3) -
            15 * pow(t_ / duration_, 4) + 
            6 * pow(t_ / duration_, 5)
        );
        std::cout << "in interpolator " << t_ << " " << duration_ << std::endl;

        interpDot[i] = 1 / duration_ * (end_[i] - start_[i]) * (
            30 * pow(t_ / duration_, 2) - 
            60 * pow(t_ / duration_, 3) + 
            30 * pow(t_ / duration_, 4)
        );
    }
    t_ += dt_;
}