#pragma once

#include <frc/geometry/Rotation2d.h>

class GyroIO {
public:
  struct GyroIOInputs {
    bool connected;
    frc::Rotation2d yawPosition;
    frc::Rotation2d pitchPosition;
    frc::Rotation2d rollPosition;
    double yawVelocityRadPerSec;
    double pitchVelocityRadPerSec;
    double rollVelocityRadPerSec;
  };

  virtual void updateInputs(GyroIOInputs &inputs) = 0;
};
