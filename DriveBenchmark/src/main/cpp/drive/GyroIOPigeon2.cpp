#include "drive/GyroIOPigeon2.h"

#include <frc/geometry/Rotation2d.h>

#include "util/PhoenixUtil.h"

GyroIOPigeon2::GyroIOPigeon2() : GyroIO() {
  ctre::phoenix6::configs::Pigeon2Configuration config{};
  pigeon.GetConfigurator().Apply(config);
  ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
      50_Hz, yaw, pitch, roll, yawVelocity, pitchVelocity, rollVelocity);
  pigeon.OptimizeBusUtilization();
  PhoenixUtil::registerSignals(true, yaw, pitch, roll, yawVelocity,
                               pitchVelocity, rollVelocity);
};

void GyroIOPigeon2::updateInputs(GyroIOInputs &inputs) {
  inputs.connected = ctre::phoenix6::BaseStatusSignal::IsAllGood(
      yaw, pitch, roll, yawVelocity, pitchVelocity, rollVelocity);
  inputs.yawPosition = frc::Rotation2d(yaw.GetValue());
  inputs.pitchPosition = frc::Rotation2d(pitch.GetValue());
  inputs.rollPosition = frc::Rotation2d(roll.GetValue());
  inputs.yawVelocityRadPerSec =
      yawVelocity.GetValue().convert<units::radians_per_second>().value();
  inputs.pitchVelocityRadPerSec =
      pitchVelocity.GetValue().convert<units::radians_per_second>().value();
  inputs.rollVelocityRadPerSec =
      rollVelocity.GetValue().convert<units::radians_per_second>().value();
};
