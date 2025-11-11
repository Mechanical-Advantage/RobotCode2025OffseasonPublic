#pragma once

#include <string_view>

namespace DriveConstants {

constexpr double trackWidthXInches = 20.75;
constexpr double trackWidthYInches = 20.75;
constexpr double maxLinearSpeed = 4.69;
constexpr double maxAngularSpeed = 6.46;
constexpr double driveKs = 5.0;
constexpr double driveKv = 0.0;
constexpr double driveKp = 35.0;
constexpr double driveKd = 0.0;
constexpr double turnKp = 4000.0;
constexpr double turnKd = 50.0;
constexpr double wheelRadiusInches = 1.9413001940413326;
constexpr double driveReduction = 6.1224489796;
constexpr double turnReduction = 21.4285714286;
constexpr double driveCurrentLimitAmps = 80.0;
constexpr double turnCurrentLimitAmps = 40.0;
constexpr std::string_view canBus = "*";
constexpr int gyroId = 30;
constexpr int driveMotorIdFL = 16;
constexpr int driveMotorIdFR = 10;
constexpr int driveMotorIdBL = 18;
constexpr int driveMotorIdBR = 13;
constexpr int turnMotorIdFL = 15;
constexpr int turnMotorIdFR = 11;
constexpr int turnMotorIdBL = 19;
constexpr int turnMotorIdBR = 14;
constexpr int encoderIdFL = 41;
constexpr int encoderIdFR = 42;
constexpr int encoderIdBL = 43;
constexpr int encoderIdBR = 44;
constexpr double encoderOffsetFL = 2.5356702423749646;
constexpr double encoderOffsetFR = -2.932971266437346;
constexpr double encoderOffsetBL = 0.6458059116998549;
constexpr double encoderOffsetBR = -2.5187964537082226;
constexpr bool turnInvertedFL = true;
constexpr bool turnInvertedFR = true;
constexpr bool turnInvertedBL = true;
constexpr bool turnInvertedBR = true;
constexpr bool encoderInvertedFL = false;
constexpr bool encoderInvertedFR = false;
constexpr bool encoderInvertedBL = false;
constexpr bool encoderInvertedBR = false;

}; // namespace DriveConstants
