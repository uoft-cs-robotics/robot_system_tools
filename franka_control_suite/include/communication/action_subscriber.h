#pragma once

#include <zmq.hpp>
#include <vector>
#include <map>
#include <thread>
#include <mutex>


enum class CommsDataType {
    JOINT_ANGLES,
    JOINT_ANGLES_VEL,
    DELTA_POSE,
    DELTA_POSE_NULL_POSE,
    POSE,
    JOINT_ANGLES_GRIPPER,
    JOINT_ANGLES_VEL_GRIPPER,
    DELTA_POSE_GRIPPER,
    POSE_GRIPPER
};

inline std::map<CommsDataType, int> typeLengths = {
    {CommsDataType::JOINT_ANGLES, 7},
    {CommsDataType::JOINT_ANGLES_VEL, 7 + 7},
    {CommsDataType::DELTA_POSE, 6},
    {CommsDataType::DELTA_POSE_NULL_POSE, 6 + 7},
    {CommsDataType::POSE, 7},//quaternion
    {CommsDataType::JOINT_ANGLES_GRIPPER, 7 + 2},
    {CommsDataType::JOINT_ANGLES_VEL_GRIPPER, 7 + 7 + 2},
    {CommsDataType::DELTA_POSE_GRIPPER, 6 + 2},
    {CommsDataType::POSE_GRIPPER, 6 + 2},
};

class ActionSubscriber {
private:
    std::mutex accessValuesMutex;

public:
    CommsDataType type;
    int action_space_dim_;
    ActionSubscriber(CommsDataType dataType, std::string portId);
    ActionSubscriber(const ActionSubscriber& ActionSubscriber);
    void readMessage();
    void readValues(std::vector<double>& output);
    double readGripperCommands();
    void setDataType(CommsDataType dataType);

    std::vector<double> values;
    zmq::context_t ctx;
    zmq::socket_t socket;
    std::string port;
};
