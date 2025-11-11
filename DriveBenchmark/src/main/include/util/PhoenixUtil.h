#pragma once

#include <ctre/phoenix/StatusCodes.h>

#include <ctre/phoenix6/StatusSignal.hpp>
#include <vector>

class PhoenixUtil {
public:
  PhoenixUtil() = delete;

  static void
  tryUntilOk(int maxAttempts,
             const std::function<ctre::phoenix::StatusCode()> &command);

  template <typename... Signals>
  static void registerSignals(bool canivore, Signals &...signals) {
    std::initializer_list<ctre::phoenix6::BaseStatusSignal *> signalList = {
        &signals...};
    if (canivore) {
      canivoreSignals.insert(canivoreSignals.end(), signalList.begin(),
                             signalList.end());
    } else {
      rioSignals.insert(rioSignals.end(), signalList.begin(), signalList.end());
    }
  }

  static void refreshAll();

private:
  static std::vector<ctre::phoenix6::BaseStatusSignal *> canivoreSignals;
  static std::vector<ctre::phoenix6::BaseStatusSignal *> rioSignals;
};
