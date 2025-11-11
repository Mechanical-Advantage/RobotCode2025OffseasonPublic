// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// All rights reserved. Proprietary and confidential.
// Unauthorized copying of this file via any medium
// is strictly prohibited.

#pragma once

#include <rev/SparkBase.h>

#include <functional>
#include <memory>
#include <vector>

class SparkUtil {
 public:
  SparkUtil() = delete;
  static bool sparkStickyFault;

  /** Return a value from a Spark (or the default if the value is invalid). */
  static double ifOkOrDefault(std::unique_ptr<rev::spark::SparkBase>& spark,
                              const std::function<double()>& supplier,
                              double defaultValue);

  static double ifOkOrDefault(
      std::unique_ptr<rev::spark::SparkBase>& spark,
      const std::vector<std::function<double()>>& suppliers,
      std::function<double(const std::vector<double>&)> transformer,
      double defaultValue);

  static void tryUntilOk(int maxAttempts,
                         const std::function<rev::REVLibError()>& result);

  static double rotationsToRads(double rotations);

  static double radsToRotations(double rads);

  static double RPMToRadsPerSec(double rpm);

  static double radsPerSecToRPM(double radsPerSec);
};
