#pragma once

#include <frc/TimedRobot.h>

#include "drive/Drive.h"
#include "drive/GyroIO.h"
#include "drive/ModuleIO.h"

class Robot : public frc::TimedRobot {
private:
  Drive *drive;
  GyroIO *gyro;
  ModuleIO *module0;
  ModuleIO *module1;
  ModuleIO *module2;
  ModuleIO *module3;

public:
  Robot();
  void RobotPeriodic() override;
  void TeleopPeriodic() override;
  void DisabledPeriodic() override;

  void SimulationPeriodic() override;
  void AutonomousPeriodic() override;
  void TestPeriodic() override;
};
