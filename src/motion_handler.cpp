#include "motion_handler.h"

void motionHandlerTask(void* param) {
    static_cast<MotionHandler*>(param)->loop();
}

MotionHandler::MotionHandler() {
    task = new pros::Task(motionHandlerTask, this, "Motion Handler Task");
}

void MotionHandler::enqueue(std::function<void()> motion, bool async) {
    auto done = std::make_shared<bool>(false);
    
    mutex.take();
    queue.push([motion, done]() {
        motion(); // Execute the movement loop
        *done = true; // Flag completion
    });
    mutex.give();

    if (!async) {
        // Block the calling thread until this specific motion completes
        while (!*done) {
            pros::delay(10);
        }
    }
}

void MotionHandler::cancelAll() {
    mutex.take();
    while(!queue.empty()) {
        queue.pop();
    }
    mutex.give();
}

void MotionHandler::waitUntilDone() {
    while (true) {
        mutex.take();
        bool empty = queue.empty();
        mutex.give();
        
        if (empty) break;
        pros::delay(10);
    }
}

void MotionHandler::loop() {
    while (true) {
        std::function<void()> currentMotion = nullptr;
        
        mutex.take();
        if (!queue.empty()) {
            currentMotion = queue.front();
        }
        mutex.give();

        if (currentMotion) {
            currentMotion(); // Run the motion (blocks this specific background task until done or early exit)
            
            mutex.take();
            if (!queue.empty()) {
                queue.pop();
            }
            mutex.give();
        } else {
            pros::delay(10);
        }
    }
}