#pragma once

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/model.hpp"

#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "../../models/"
#endif

class PinocchioUtils
{
private:
  std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string("franka.urdf");
  pinocchio::Model modelPin;
  pinocchio::Data data;
  Eigen::VectorXd pinGrav;
public:
  PinocchioUtils(/* args */);
  ~PinocchioUtils();
};

PinocchioUtils::PinocchioUtils(/* args */)
{
  pinocchio::urdf::buildModel(urdf_filename, modelPin);
  data = pinocchio::Data(modelPin);
}

PinocchioUtils::~PinocchioUtils()
{

}

