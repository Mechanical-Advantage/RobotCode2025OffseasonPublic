#include "util/PhoenixUtil.h"

std::vector<ctre::phoenix6::BaseStatusSignal *> PhoenixUtil::canivoreSignals;
std::vector<ctre::phoenix6::BaseStatusSignal *> PhoenixUtil::rioSignals;

void PhoenixUtil::tryUntilOk(
    int maxAttempts,
    const std::function<ctre::phoenix::StatusCode()> &command) {
  for (int i = 0; i < maxAttempts; i++) {
    ctre::phoenix::StatusCode error = command();
    if (error.IsOK()) {
      break;
    }
  }
}

void PhoenixUtil::refreshAll() {
  if (!canivoreSignals.empty()) {
    ctre::phoenix6::BaseStatusSignal::RefreshAll(canivoreSignals);
  }
  if (!rioSignals.empty()) {
    ctre::phoenix6::BaseStatusSignal::RefreshAll(rioSignals);
  }
}
