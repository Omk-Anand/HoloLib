#pragma once

#include <vector>
#include <cstdint>

class Chassis;

enum class TuneTarget {
  X_PID,
  Y_PID,
  THETA_PID
};

struct TuneConfig {
  float dist;
  int maxCycles = 20;
  float maxSpeed = 127.0f;
  uint32_t timeout = 8000;
};

class AutoTuner {
public:
  static void run(Chassis* chassis, TuneTarget target, TuneConfig config);
};
