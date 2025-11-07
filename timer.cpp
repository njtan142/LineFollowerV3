#include "timer.h"

/**
 * Timer Class - Simple stopwatch-style timer for measuring elapsed time
 * 
 * Provides start/stop/reset functionality with millisecond precision.
 * Useful for measuring durations, timeouts, and performance profiling.
 */

/**
 * Timer Constructor - Initializes timer in stopped state
 * 
 * Creates a new timer with all values zeroed. Timer must be explicitly
 * started with start() before it will track elapsed time.
 */
Timer::Timer()
    : startTime(0)
    , stopTime(0)
    , running(false)
{
}

/**
 * Starts or restarts the timer
 * 
 * Captures current millisecond count as start time and sets running flag.
 * If timer was already running, this effectively resets it to zero.
 * Uses Arduino's millis() function which returns milliseconds since boot.
 */
void Timer::start() {
    startTime = millis();
    running = true;
}

/**
 * Stops the timer and freezes the elapsed time
 * 
 * Captures the current time as stop time, allowing elapsed() to return
 * the duration between start and stop. If timer isn't running, does nothing
 * to prevent accidentally overwriting the stop time.
 * 
 * After stopping, elapsed() will return the same value until timer is restarted.
 */
void Timer::stop() {
    // Only capture stop time if timer is actually running
    if (running) {
        stopTime = millis();
        running = false;
    }
}

/**
 * Resets the timer back to zero while keeping it running
 * 
 * Sets start time to current moment and clears stop time.
 * Useful for reusing the same timer object for multiple measurements
 * without creating new instances.
 */
void Timer::reset() {
    startTime = millis();
    stopTime = 0;
}

/**
 * Returns elapsed time in milliseconds
 * 
 * If timer is running: returns time since start() was called
 * If timer is stopped: returns duration between start() and stop()
 * 
 * This allows the timer to be used both for ongoing measurements
 * and for storing completed measurements.
 * 
 * @return Elapsed time in milliseconds
 */
unsigned long Timer::elapsed() const {
    // Timer is actively running - calculate current elapsed time
    if (running) {
        return millis() - startTime;
    } 
    // Timer was stopped - return frozen duration
    else {
        return stopTime - startTime;
    }
}

/**
 * Checks if elapsed time has exceeded a specified duration
 * 
 * Convenience method for timeout detection. Common usage pattern:
 *   if (timer.isExpired(5000)) { // 5 seconds have passed
 *       // do something
 *   }
 * 
 * @param duration - Time threshold in milliseconds
 * @return true if elapsed time >= duration, false otherwise
 */
bool Timer::isExpired(unsigned long duration) const {
    return elapsed() >= duration;
}

/**
 * PeriodicTimer Class - Generates periodic timing events
 * 
 * Automatically triggers at regular intervals, useful for tasks that need
 * to run periodically (like sensor reading, display updates, etc.) without
 * blocking the main loop.
 * 
 * Unlike Timer which measures elapsed time, PeriodicTimer tells you WHEN
 * a specific interval has passed and automatically resets for the next interval.
 */

/**
 * PeriodicTimer Constructor - Initializes timer with specified interval
 * 
 * Sets up a timer that will trigger every 'interval' milliseconds.
 * Automatically captures current time as starting point.
 * 
 * Example: PeriodicTimer updateTimer(100); // Triggers every 100ms
 * 
 * @param interval - Time between triggers in milliseconds
 */
PeriodicTimer::PeriodicTimer(unsigned long interval)
    : interval(interval)
    , lastTime(millis())
{
}

/**
 * Checks if the interval has elapsed and auto-resets if so
 * 
 * This is the main method to call in your loop. When the specified interval
 * has passed since last trigger (or construction), it returns true and
 * automatically resets the timer for the next interval.
 * 
 * Usage pattern:
 *   PeriodicTimer displayTimer(50);  // Update every 50ms
 *   
 *   void loop() {
 *       if (displayTimer.ready()) {
 *           updateDisplay();  // This runs every 50ms
 *       }
 *   }
 * 
 * The auto-reset feature ensures you don't miss intervals even if your
 * loop takes varying amounts of time to execute.
 * 
 * @return true if interval has elapsed (and timer has been reset), false otherwise
 */
bool PeriodicTimer::ready() {
    unsigned long currentTime = millis();
    
    // Check if enough time has passed since last trigger
    if (currentTime - lastTime >= interval) {
        // Interval elapsed - update last time to current for next interval
        lastTime = currentTime;
        return true;  // Signal that it's time to execute periodic task
    }
    
    // Interval not yet elapsed
    return false;
}

/**
 * Changes the trigger interval
 * 
 * Updates how often the timer triggers. Takes effect immediately,
 * but doesn't reset the timer - next trigger will be based on when
 * ready() last returned true.
 * 
 * @param newInterval - New time between triggers in milliseconds
 */
void PeriodicTimer::setInterval(unsigned long newInterval) {
    interval = newInterval;
}

/**
 * Resets the timer to start a fresh interval
 * 
 * Sets the "last trigger time" to now, effectively restarting the countdown.
 * Useful when you want to synchronize the timer with an event or skip
 * accumulated time.
 * 
 * Example: After user input, reset display timer to update immediately
 */
void PeriodicTimer::reset() {
    lastTime = millis();
}
