#include "timer.h"

// Timer implementation
Timer::Timer()
    : startTime(0)
    , stopTime(0)
    , running(false)
{
}

void Timer::start() {
    startTime = millis();
    running = true;
}

void Timer::stop() {
    if (running) {
        stopTime = millis();
        running = false;
    }
}

void Timer::reset() {
    startTime = millis();
    stopTime = 0;
}

unsigned long Timer::elapsed() const {
    if (running) {
        return millis() - startTime;
    } else {
        return stopTime - startTime;
    }
}

bool Timer::isExpired(unsigned long duration) const {
    return elapsed() >= duration;
}

// PeriodicTimer implementation
PeriodicTimer::PeriodicTimer(unsigned long interval)
    : interval(interval)
    , lastTime(millis())
{
}

bool PeriodicTimer::ready() {
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= interval) {
        lastTime = currentTime;
        return true;
    }
    return false;
}

void PeriodicTimer::setInterval(unsigned long newInterval) {
    interval = newInterval;
}

void PeriodicTimer::reset() {
    lastTime = millis();
}
