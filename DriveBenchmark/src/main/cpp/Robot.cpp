#include "Robot.h"

#include <ctre/phoenix6/SignalLogger.hpp>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/Threads.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <hal/HAL.h>
#include <networktables/NetworkTableInstance.h>
#include <wpinet/PortForwarder.h>

#include "Constants.h"
#include "drive/DriveConstants.h"
#include "drive/GyroIOPigeon2.h"
#include "drive/ModuleIOTalonFX.h"
#include "util/PhoenixUtil.h"

Robot::Robot()
    : frc::TimedRobot(units::time::second_t{Constants::loopPeriodSecs}) {
  frc::DriverStation::SilenceJoystickConnectionWarning(true);
  frc::RobotController::SetBrownoutVoltage(6.0_V);
  ctre::phoenix6::SignalLogger::EnableAutoLogging(false);

  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());

  gyro = new GyroIOPigeon2();
  module0 = new ModuleIOTalonFX(
      DriveConstants::driveMotorIdFL, DriveConstants::turnMotorIdFL,
      DriveConstants::encoderIdFL, DriveConstants::encoderOffsetFL,
      DriveConstants::turnInvertedFL, DriveConstants::encoderInvertedFL);
  module1 = new ModuleIOTalonFX(
      DriveConstants::driveMotorIdFR, DriveConstants::turnMotorIdFR,
      DriveConstants::encoderIdFR, DriveConstants::encoderOffsetFR,
      DriveConstants::turnInvertedFR, DriveConstants::encoderInvertedFR);
  module2 = new ModuleIOTalonFX(
      DriveConstants::driveMotorIdBL, DriveConstants::turnMotorIdBL,
      DriveConstants::encoderIdBL, DriveConstants::encoderOffsetBL,
      DriveConstants::turnInvertedBL, DriveConstants::encoderInvertedBL);
  module3 = new ModuleIOTalonFX(
      DriveConstants::driveMotorIdBR, DriveConstants::turnMotorIdBR,
      DriveConstants::encoderIdBR, DriveConstants::encoderOffsetBR,
      DriveConstants::turnInvertedBR, DriveConstants::encoderInvertedBR);
  drive = new Drive(gyro, module0, module1, module2, module3);

  frc::SetCurrentThreadPriority(true, 1);
}

void Robot::RobotPeriodic() { PhoenixUtil::refreshAll(); }

void Robot::TeleopPeriodic() { drive->periodic(); };

void Robot::DisabledPeriodic() {
  module0->stop();
  module1->stop();
  module2->stop();
  module3->stop();
};

void Robot::SimulationPeriodic() {};
void Robot::AutonomousPeriodic() {};
void Robot::TestPeriodic() {};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
