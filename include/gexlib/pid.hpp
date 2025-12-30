#pragma once

#include <concepts>
#include <optional>
namespace gexlib {

/**
 * Simple proportional-integral-derivative (PID) controller helper.
 *
 * PID controllers continuously adjust an output based on how far the system is
 * from a desired target (error), how quickly that error is changing, and how
 * long the error has been present. This class stores the tuning constants and
 * historical data so you just call update() each cycle and get a corrective
 * value to apply to your motors/actuators.
 */
class PID {
public:
    /** Proportional gain: scales the current error (target - state). */
    float kp;
    /** Integral gain: scales the accumulated error over time (helps remove steady offsets). */
    float ki;
    /** Derivative gain: scales how fast the error is changing (damps overshoot). */
    float kd;
    /**
     * When true, the integral term automatically resets whenever set_target()
     * is called so old error does not leak into new maneuvers.
     */
    bool integral_reset = true;

    PID(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f, bool integral_reset = true)
        : kp(kp), ki(ki), kd(kd), integral_reset(integral_reset), integral(0.0f) {}

    /**
     * Clears stored error/integral/timing data so the next update() call starts
     * fresh.
     */
    void reset(void);

    /**
     * Computes the controller output for the current system state.
     *
     * @param state   The measured value from your sensor (e.g. angle, distance).
     * @param target  Optional desired value; if omitted, uses the last value
     *                provided to set_target().
     * @param now     Optional timestamp (in microseconds or milliseconds) for
     *                deterministic timing; omit to let the controller call
     *                pros::micros() internally.
     * @return        Control effort to feed into your actuators.
     */
    template<typename T = float>
        requires std::integral<T> || std::floating_point<T>
    float update(float state, 
        std::optional<float> target = std::nullopt, 
        std::optional<T> now = std::nullopt);

    /**
     * Stores a new desired value for the controller. Optionally resets the
     * integral term and timing history so you can start chasing the new setpoint
     * cleanly.
     */
    void set_target(float target, bool reset = false);

private:
    std::optional<long long> last_update_time = std::nullopt;
    std::optional<float> target = std::nullopt;
    std::optional<float> last_error = std::nullopt;
    float integral;
};
}