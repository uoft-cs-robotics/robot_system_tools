#pragma once

#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/model.h>

#include "communication/action_subscriber.h"
#include "communication/state_publisher.h"


namespace robotContext {
    extern franka::Robot *robot;
    extern franka::Gripper *gripper;
    extern franka::Model *model;
}

namespace Comms { 
    extern ActionSubscriber *actionSubscriber;
    extern StatePublisher *statePublisher;
}
