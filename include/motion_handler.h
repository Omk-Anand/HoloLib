#pragma once

#include "api.h"
#include <queue>
#include <functional>
#include <memory>

class MotionHandler {
public:
    MotionHandler();
    
    // Pushes a movement closure to the queue. Blocks if async is false.
    void enqueue(std::function<void()> motion, bool async);
    
    // Clears all pending movements and stops the current one
    void cancelAll();
    
    // Blocks the current thread until the motion queue is entirely empty
    void waitUntilDone();

private:
    std::queue<std::function<void()>> queue;
    pros::Mutex mutex;
    pros::Task* task = nullptr;
    
    void loop();
    friend void motionHandlerTask(void* param);
};