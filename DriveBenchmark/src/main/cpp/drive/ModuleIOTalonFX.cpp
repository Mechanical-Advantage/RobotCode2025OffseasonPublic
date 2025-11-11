#include "drive/ModuleIOTalonFX.h"

#include <frc/geometry/Rotation2d.h>

#include "Robot.h"
#include "drive/DriveConstants.h"
#include "util/PhoenixUtil.h"

ModuleIOTalonFX::ModuleIOTalonFX(const int driveId, const int turnId,
                                 const int encoderId,
                                 const double encoderOffset,
                                 const bool turnInverted,
                                 const bool encoderInverted)
    : driveTalon(driveId, DriveConstants::canBus),
      turnTalon(turnId, DriveConstants::canBus),
      encoder(encoderId, DriveConstants::canBus),
      drivePosition(driveTalon.GetPosition()),
      driveVelocity(driveTalon.GetVelocity()),
      driveAppliedVolts(driveTalon.GetMotorVoltage()),
      driveSupplyCurrent(driveTalon.GetSupplyCurrent()),
      driveTorqueCurrent(driveTalon.GetTorqueCurrent()),
      turnAbsolutePosition(encoder.GetAbsolutePosition()),
      turnPosition(turnTalon.GetPosition()),
      turnVelocity(turnTalon.GetVelocity()),
      turnAppliedVolts(turnTalon.GetMotorVoltage()),
      turnSupplyCurrent(turnTalon.GetSupplyCurrent()),
      turnTorqueCurrent(turnTalon.GetTorqueCurrent()) {
  ctre::phoenix6::configs::TalonFXConfiguration driveConfig{};
  driveConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  driveConfig.Slot0 = ctre::phoenix6::configs::Slot0Configs()
                          .WithKP(DriveConstants::driveKp)
                          .WithKD(DriveConstants::driveKd);
  driveConfig.Feedback.SensorToMechanismRatio = DriveConstants::driveReduction;
  driveConfig.TorqueCurrent.PeakForwardTorqueCurrent =
      units::ampere_t{DriveConstants::driveCurrentLimitAmps};
  driveConfig.TorqueCurrent.PeakReverseTorqueCurrent =
      -units::ampere_t{DriveConstants::driveCurrentLimitAmps};
  driveConfig.CurrentLimits.StatorCurrentLimit =
      units::ampere_t{DriveConstants::driveCurrentLimitAmps};
  driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02_s;

  PhoenixUtil::tryUntilOk(
      5, [&]() { return driveTalon.GetConfigurator().Apply(driveConfig); });
  PhoenixUtil::tryUntilOk(
      5, [&]() { return driveTalon.SetPosition(0.0_rad, 0.25_s); });

  ctre::phoenix6::configs::TalonFXConfiguration turnConfig{};
  turnConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  turnConfig.Slot0 = ctre::phoenix6::configs::Slot0Configs()
                         .WithKP(DriveConstants::turnKp)
                         .WithKD(DriveConstants::turnKd);
  turnConfig.Feedback.FeedbackRemoteSensorID = encoderId;
  turnConfig.Feedback.FeedbackSensorSource =
      ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
  turnConfig.Feedback.RotorToSensorRatio = DriveConstants::turnReduction;
  turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
  turnConfig.TorqueCurrent.PeakForwardTorqueCurrent =
      units::ampere_t{DriveConstants::turnCurrentLimitAmps};
  turnConfig.TorqueCurrent.PeakReverseTorqueCurrent =
      -units::ampere_t{DriveConstants::turnCurrentLimitAmps};
  turnConfig.CurrentLimits.StatorCurrentLimit =
      units::ampere_t{DriveConstants::turnCurrentLimitAmps};
  turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  turnConfig.MotorOutput.Inverted =
      turnInverted
          ? ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive
          : ctre::phoenix6::signals::SensorDirectionValue::
                CounterClockwise_Positive;

  PhoenixUtil::tryUntilOk(5, [&]() {
    return turnTalon.GetConfigurator().Apply(turnConfig, 0.25_s);
  });

  ctre::phoenix6::configs::CANcoderConfiguration encoderConfig{};
  encoderConfig.MagnetSensor.MagnetOffset = units::radian_t{encoderOffset};
  encoderConfig.MagnetSensor.SensorDirection =
      encoderInverted
          ? ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive
          : ctre::phoenix6::signals::SensorDirectionValue::
                CounterClockwise_Positive;

  PhoenixUtil::tryUntilOk(5, [&]() {
    return encoder.GetConfigurator().Apply(encoderConfig, 0.25_s);
  });

  ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
      50_Hz, drivePosition, driveVelocity, driveAppliedVolts,
      driveSupplyCurrent, driveTorqueCurrent, turnAbsolutePosition,
      turnPosition, turnVelocity, turnAppliedVolts, turnSupplyCurrent,
      turnTorqueCurrent);
  PhoenixUtil::tryUntilOk(5, [&]() {
    return ctre::phoenix6::hardware::ParentDevice::OptimizeBusUtilizationForAll(
        driveTalon, turnTalon, encoder);
  });

  PhoenixUtil::registerSignals(
      true, drivePosition, driveVelocity, driveAppliedVolts, driveSupplyCurrent,
      driveTorqueCurrent, turnAbsolutePosition, turnPosition, turnVelocity,
      turnAppliedVolts, turnSupplyCurrent, turnTorqueCurrent);
};

void ModuleIOTalonFX::updateInputs(ModuleIOInputs &inputs) {
  inputs.driveConnected = ctre::phoenix6::BaseStatusSignal::IsAllGood(
      drivePosition, driveVelocity, driveAppliedVolts, driveSupplyCurrent,
      driveTorqueCurrent);
  inputs.drivePositionRad =
      drivePosition.GetValue().convert<units::radian>().value();
  inputs.driveVelocityRadPerSec =
      driveVelocity.GetValue().convert<units::rad_per_s>().value();
  inputs.driveAppliedVolts =
      driveAppliedVolts.GetValue().convert<units::volt>().value();
  inputs.driveSupplyCurrentAmps =
      driveSupplyCurrent.GetValue().convert<units::ampere>().value();
  inputs.driveTorqueCurrentAmps =
      driveTorqueCurrent.GetValue().convert<units::ampere>().value();

  inputs.turnConnected = ctre::phoenix6::BaseStatusSignal::IsAllGood(
      turnAbsolutePosition, turnPosition, turnVelocity, turnAppliedVolts,
      turnSupplyCurrent, turnTorqueCurrent);
  inputs.turnAbsolutePosition =
      frc::Rotation2d{turnAbsolutePosition.GetValue()};
  inputs.turnPosition = frc::Rotation2d{turnPosition.GetValue()};
  inputs.turnVelocityRadPerSec =
      turnVelocity.GetValue().convert<units::rad_per_s>().value();
  inputs.turnAppliedVolts =
      turnAppliedVolts.GetValue().convert<units::volt>().value();
  inputs.turnSupplyCurrentAmps =
      turnSupplyCurrent.GetValue().convert<units::ampere>().value();
  inputs.turnTorqueCurrentAmps =
      turnTorqueCurrent.GetValue().convert<units::ampere>().value();
}

void ModuleIOTalonFX::applyOutputs(const ModuleIOOutputs &outputs) {
  switch (outputs.mode) {
  case ModuleIOOutputMode::COAST:
    driveTalon.SetControl(coastRequest);
    turnTalon.SetControl(coastRequest);
    break;

  case ModuleIOOutputMode::BRAKE:
    driveTalon.SetControl(brakeRequest);
    turnTalon.SetControl(brakeRequest);
    break;

  case ModuleIOOutputMode::DRIVE:
    driveTalon.SetControl(
        velocityTorqueCurrentRequest
            .WithVelocity(
                units::radians_per_second_t{outputs.driveVelocityRadPerSec})
            .WithFeedForward(units::ampere_t{outputs.driveFeedforward}));
    if (outputs.turnNeutral) {
      turnTalon.SetControl(brakeRequest);
    } else {
      turnTalon.SetControl(positionTorqueCurrentRequest.WithPosition(
          outputs.turnRotation.Radians()));
    }
    break;

  case ModuleIOOutputMode::CHARACTERIZE:
    driveTalon.SetControl(torqueCurrentRequest.WithOutput(
        units::ampere_t{outputs.driveCharacterizationOutput}));
    turnTalon.SetControl(positionTorqueCurrentRequest.WithPosition(
        outputs.turnRotation.Radians()));
    break;
  }
}

void ModuleIOTalonFX::stop() {
  driveTalon.StopMotor();
  turnTalon.StopMotor();
}
