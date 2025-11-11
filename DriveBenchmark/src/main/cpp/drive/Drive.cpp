#include "drive/Drive.h"

#include <frc/DriverStation.h>
#include <frc/MathUtil.h>

#include "Robot.h"

namespace {

constexpr double deadband = 0.1;
constexpr double ffStartDelay = 2.0;
constexpr double ffRampRate = 0.1;
constexpr double wheelRadiusMaxVelocity = 0.25;
constexpr double wheelRadiusRampRate = 0.05;

}; // namespace

Drive::Drive(GyroIO *gyro, ModuleIO *module0, ModuleIO *module1,
             ModuleIO *module2, ModuleIO *module3)
    : gyro(gyro), module{module0, module1, module2, module3} {};

void Drive::periodic() {
  GyroIO::GyroIOInputs gyroInputs;
  gyro->updateInputs(gyroInputs);

  if (frc::DriverStation::GetStickButton(0, 7) &&
      frc::DriverStation::GetStickButton(0, 8)) {
    gyroOffset = -gyroInputs.yawPosition;
  }
  frc::Rotation2d currentRotation = gyroInputs.yawPosition + gyroOffset;

  if (frc::DriverStation::IsDisabled()) {
    ModuleIO::ModuleIOOutputs moduleOutputs;
    moduleOutputs.mode = ModuleIOOutputMode::BRAKE;
    for (int i = 0; i < 4; i++) {
      module[i]->applyOutputs(moduleOutputs);
    }
    return;
  }

  const double inputX = -frc::DriverStation::GetStickAxis(0, 1);
  const double inputY = -frc::DriverStation::GetStickAxis(0, 0);
  const double linearMagnitude =
      frc::ApplyDeadband(std::hypot(inputX, inputY), deadband);
  const frc::Rotation2d linearDirection{
      units::radian_t{std::atan2(inputY, inputX)}};
  const auto linearVelocity =
      frc::Pose2d(frc::Translation2d{}, linearDirection)
          .TransformBy(frc::Transform2d(
              units::meter_t{linearMagnitude * linearMagnitude}, 0.0_m,
              frc::Rotation2d{}))
          .Translation();

  const double inputOmega = -frc::DriverStation::GetStickAxis(0, 4);
  const double omegaRaw = frc::ApplyDeadband(inputOmega, deadband);
  const double omega = std::copysign(omegaRaw * omegaRaw, omegaRaw);

  const bool isFlipped = frc::DriverStation::GetAlliance().value_or(
                             frc::DriverStation::Alliance::kBlue) ==
                         frc::DriverStation::Alliance::kRed;
  const auto speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      units::meters_per_second_t{linearVelocity.X().value() *
                                 DriveConstants::maxLinearSpeed},
      units::meters_per_second_t{linearVelocity.Y().value() *
                                 DriveConstants::maxLinearSpeed},
      units::radians_per_second_t{omega * DriveConstants::maxAngularSpeed},
      isFlipped ? currentRotation + frc::Rotation2d{units::radian_t{M_PI}}
                : currentRotation);

  const auto discreteSpeeds = frc::ChassisSpeeds::Discretize(
      speeds, units::second_t{Constants::loopPeriodSecs});
  auto setpointStates = kinematics.ToSwerveModuleStates(discreteSpeeds);
  kinematics.DesaturateWheelSpeeds(
      &setpointStates,
      units::meters_per_second_t{DriveConstants::maxLinearSpeed});

  for (int i = 0; i < 4; i++) {
    ModuleIO::ModuleIOInputs moduleInputs;
    module[i]->updateInputs(moduleInputs);

    setpointStates[i].Optimize(moduleInputs.turnPosition);
    setpointStates[i].CosineScale(moduleInputs.turnPosition);

    const double speedRadPerSec =
        setpointStates[i].speed.value() /
        units::inch_t{DriveConstants::wheelRadiusInches}
            .convert<units::meter>()
            .value();
    ModuleIO::ModuleIOOutputs moduleOutputs;
    moduleOutputs.mode = ModuleIOOutputMode::DRIVE;
    moduleOutputs.driveVelocityRadPerSec = speedRadPerSec;
    moduleOutputs.driveFeedforward =
        ffModel.Calculate(units::radians_per_second_t{speedRadPerSec}).value();
    moduleOutputs.turnRotation = setpointStates[i].angle;
    module[i]->applyOutputs(moduleOutputs);
  }
}
