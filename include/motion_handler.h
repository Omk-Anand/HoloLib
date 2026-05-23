#pragma once

#include "api.h"
#include <functional>
#include <memory>
#include <queue>

class MotionHandler {
public:
  MotionHandler();

  void enqueue(std::function<void()> motion, bool async);

  void cancelAll();

  void waitUntilDone();

private:
  std::queue<std::function<void()>> queue;
  pros::Mutex mutex;
  pros::Task *task = nullptr;

  void loop();
  friend void motionHandlerTask(void *param);
};
