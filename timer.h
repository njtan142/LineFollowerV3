#ifndef TIMER_H
#define TIMER_H

#include <Arduino.h>

/**
 * @file timer.h
 * @brief Timing utilities for consistent time management across the codebase
 * 
 * Provides Timer and PeriodicTimer classes to centralize timing logic
 * and eliminate scattered millis() calls throughout the code.
 */

/**
 * @class Timer
 * @brief Simple stopwatch-style timer for measuring elapsed time
 * 
 * Usage:
 * @code
 * Timer timer;
 * timer.start();
 * // ... do work ...
 * unsigned long elapsed = timer.elapsed();
 * @endcode
 */
class Timer {
public:
    /**
     * @brief Construct a new Timer (not started)
     */
    Timer();
    
    /**
     * @brief Start or restart the timer
     */
    void start();
    
    /**
     * @brief Stop the timer
     */
    void stop();
    
    /**
     * @brief Reset the timer to zero
     */
    void reset();
    
    /**
     * @brief Get elapsed time since start
     * @return Elapsed time in milliseconds
     */
    unsigned long elapsed() const;
    
    /**
     * @brief Check if a duration has passed
     * @param duration Duration to check (milliseconds)
     * @return true if duration has elapsed
     */
    bool isExpired(unsigned long duration) const;
    
    /**
     * @brief Check if timer is currently running
     * @return true if running
     */
    bool isRunning() const { return running; }

private:
    unsigned long startTime;  ///< Time when timer was started
    unsigned long stopTime;   ///< Time when timer was stopped
    bool running;             ///< Whether timer is currently running
};

/**
 * @class PeriodicTimer
 * @brief Timer for recurring events at fixed intervals
 * 
 * Usage:
 * @code
 * PeriodicTimer timer(100); // 100ms interval
 * 
 * void loop() {
 *     if (timer.ready()) {
 *         // This runs every 100ms
 *         doPeriodicTask();
 *     }
 * }
 * @endcode
 */
class PeriodicTimer {
public:
    /**
     * @brief Construct a periodic timer
     * @param interval Time between events (milliseconds)
     */
    PeriodicTimer(unsigned long interval);
    
    /**
     * @brief Check if interval has elapsed and reset if so
     * @return true if interval has elapsed (and timer is reset)
     */
    bool ready();
    
    /**
     * @brief Set a new interval
     * @param interval New interval (milliseconds)
     */
    void setInterval(unsigned long interval);
    
    /**
     * @brief Get the current interval
     * @return Interval in milliseconds
     */
    unsigned long getInterval() const { return interval; }
    
    /**
     * @brief Reset the timer without changing interval
     */
    void reset();

private:
    unsigned long interval;   ///< Time between events
    unsigned long lastTime;   ///< Time of last event
};

#endif
