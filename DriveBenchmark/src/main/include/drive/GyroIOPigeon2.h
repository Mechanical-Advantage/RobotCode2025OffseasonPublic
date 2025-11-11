#pragma once

#include <ctre/phoenix6/Pigeon2.hpp>

#include "DriveConstants.h"
#include "GyroIO.h"

class GyroIOPigeon2 : public GyroIO {
public:
  GyroIOPigeon2();

private:
  void updateInputs(GyroIOInputs &inputs) override;

  ctre::phoenix6::hardware::Pigeon2 pigeon{DriveConstants::gyroId,
                                           DriveConstants::canBus};
  ctre::phoenix6::StatusSignal<units::angle::degree_t> yaw = pigeon.GetYaw();
  ctre::phoenix6::StatusSignal<units::angle::degree_t> pitch =
      pigeon.GetPitch();
  ctre::phoenix6::StatusSignal<units::angle::degree_t> roll = pigeon.GetRoll();
  ctre::phoenix6::StatusSignal<units::angular_velocity::degrees_per_second_t>
      yawVelocity = pigeon.GetAngularVelocityZWorld();
  ctre::phoenix6::StatusSignal<units::angular_velocity::degrees_per_second_t>
      pitchVelocity = pigeon.GetAngularVelocityXWorld();
  ctre::phoenix6::StatusSignal<units::angular_velocity::degrees_per_second_t>
      rollVelocity = pigeon.GetAngularVelocityYWorld();
};
