# pragma once 

#include <eigen3/Eigen/Dense>
#include <type_traits>

#include <iostream>


namespace utils{
    // References for quaternion eigen operations 
    // https://www.cc.gatech.edu/classes/AY2015/cs4496_spring/Eigen.html
    // https://gist.github.com/SIRHAMY/9767ed75bddf0b87b929


    // can use just * with eigen::quaterniond for multiplication
    inline Eigen::Quaterniond quat_multiplication(const Eigen::Quaterniond &q1, 
                                            const Eigen::Quaterniond &q2) {
        Eigen::Quaterniond resultQ;
        resultQ.w() = q1.w() * q2.w() - q1.vec().dot(q2.vec());
        resultQ.vec() = q1.w() * q2.vec() + q2.w() * q1.vec() + q1.vec().cross(q2.vec());
        return resultQ;
    }
    
    inline Eigen::Quaterniond get_ori_error_quat(const Eigen::Quaterniond &target_quat,
                                        const Eigen::Quaterniond &current_quat){

        Eigen::Quaterniond conj = current_quat.conjugate();
        double current_EE_quat_norm = (quat_multiplication (current_quat, conj) ).w();
        Eigen::Quaterniond temp = conj;
        Eigen::Quaterniond current_EE_quat_inv(temp.coeffs()/current_EE_quat_norm);
        return utils::quat_multiplication(target_quat, current_EE_quat_inv);

    }

    inline Eigen::AngleAxisd get_ori_error_aa(const Eigen::Quaterniond &target_quat,
                                        const Eigen::Quaterniond &current_quat){
        Eigen::AngleAxisd output(utils::get_ori_error_quat(target_quat, current_quat)); 
        return output;
    }

    inline Eigen::Matrix<double, 3, 1>  get_ori_error_matrix(const Eigen::Matrix3d &desired_ori_matrix,
                                        const Eigen::Matrix3d &current_ori_matrix){
        Eigen::Vector3d rc1 = current_ori_matrix.col(0);
        Eigen::Vector3d rc2 = current_ori_matrix.col(1);
        Eigen::Vector3d rc3 = current_ori_matrix.col(2);

        Eigen::Vector3d rd1 = desired_ori_matrix.col(0);
        Eigen::Vector3d rd2 = desired_ori_matrix.col(1);
        Eigen::Vector3d rd3 = desired_ori_matrix.col(2); 
        
        Eigen::Matrix<double, 3, 1> error = 0.5 * (rc1.cross(rd1) + rc2.cross(rd2)+ rc3.cross(rd3));       

        return error;
    }

    inline Eigen::Matrix<double, 3, 1> get_ori_error_matrix(const Eigen::Quaterniond &desired_ori_quat,
                                        const Eigen::Quaterniond &current_ori_quat){
        return get_ori_error_matrix(desired_ori_quat.toRotationMatrix(), current_ori_quat.toRotationMatrix());
    }

}
