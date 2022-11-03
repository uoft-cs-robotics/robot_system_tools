#include "base_controller.h"
	
#include <eigen3/Eigen/Dense>
#include <franka/rate_limiting.h>

enum class IKType{
    JACOBIAN_TRANSPOSE,
    M_P_PSEUDO_INVERSE,
    DAMPED_LS, 
    ADAPTIVE_SVD,
};

class InverseKinematics: public ControllersBase{

public:
    InverseKinematics(int start, IKType type); 
    ~InverseKinematics();
    franka::JointVelocities operator()(const franka::RobotState&, franka::Duration);
private: 
    IKType type_; 
    //std::array<double, 6> pose_command = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}; 
    Eigen::VectorXd ee_goal_pose; 
    Eigen::Matrix<double, 7, 1> _get_delta_joint_angles(const Eigen::Matrix<double, 6, 7> &jacobian,
                                                    const Eigen::Vector3d &ee_position_error,
                                                    const Eigen::Vector3d &ee_ori_error);  
    std::array<double,7>  _compute_target_joint_angle_pos(const Eigen::Matrix<double, 7, 1> &q_now, 
                                                        const Eigen::Matrix<double, 6, 7> &jacobian,
                                                        const Eigen::Vector3d &ee_position_error,
                                                        const Eigen::AngleAxisd &ee_ori_error_aa);
                                                        
};

/*
Refernce: MR pseudo invere with Eigen
 https://gist.github.com/pshriwise/67c2ae78e5db3831da38390a8b2a209f */
// method for calculating the pseudo-Inverse as recommended by Eigen developers
template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
	Eigen::JacobiSVD< _Matrix_Type_ > svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
	double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
	return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

