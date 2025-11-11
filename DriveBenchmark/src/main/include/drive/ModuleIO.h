#pragma once

#include <frc/geometry/Rotation2d.h>

enum class ModuleIOOutputMode {
  COAST = 0,
  BRAKE = 1,
  DRIVE = 2,
  CHARACTERIZE = 3
};

class ModuleIO {
public:
  struct ModuleIOInputs {
    bool driveConnected;
    double drivePositionRad;
    double driveVelocityRadPerSec;
    double driveAppliedVolts;
    double driveSupplyCurrentAmps;
    double driveTorqueCurrentAmps;
    bool turnConnected;
    frc::Rotation2d turnAbsolutePosition;
    frc::Rotation2d turnPosition;
    double turnVelocityRadPerSec;
    double turnAppliedVolts;
    double turnSupplyCurrentAmps;
    double turnTorqueCurrentAmps;
  };

  struct ModuleIOOutputs {
    ModuleIOOutputMode mode;
    double driveVelocityRadPerSec;
    double driveFeedforward;
    double driveCharacterizationOutput;
    frc::Rotation2d turnRotation;
    bool turnNeutral;
  };

  virtual void updateInputs(ModuleIOInputs &inputs) = 0;
  virtual void applyOutputs(const ModuleIOOutputs &outputs) = 0;
  virtual void stop() = 0;
};
