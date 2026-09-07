// imu_gnss_sim.hpp
//
// Simulates the two sensors the mission computer fuses: an IMU (accelerometer)
// and a GNSS receiver. There is no real IMU or GPS chip here. A simple
// scripted "true" trajectory is generated in software, and IMU/GNSS readings
// are derived from it with noise added, the same way the companion SDR
// project simulates IQ samples instead of reading a real radio.

#ifndef AUTONOMY_PLATFORM_IMU_GNSS_SIM_HPP
#define AUTONOMY_PLATFORM_IMU_GNSS_SIM_HPP

#include <cmath>
#include <cstdlib>

using namespace std;

// A 2D true vehicle state used only to generate synthetic sensor readings.
struct TrueState {
    float x;
    float y;
    float vx;
    float vy;
    float ax;
    float ay;
};

// One simulated IMU reading (linear acceleration, body/world frame is not
// modeled separately since this is a 2D toy simulation).
struct ImuSample {
    float ax;
    float ay;
    bool valid; // false while a sensor dropout fault is active
};

// One simulated GNSS fix.
struct GnssSample {
    float x;
    float y;
    bool valid; // false while a sensor dropout fault is active
};

// Simple deterministic pseudo-random noise generator (linear congruential
// generator) so simulation runs are reproducible across fault-sweep
// scenarios. Not cryptographic, just a repeatable noise source.
class NoiseSource {
public:
    NoiseSource(unsigned int seed) {
        state_ = seed;
    }

    // Returns a pseudo-random float uniformly distributed in [-amplitude, amplitude].
    float NextUniform(float amplitude) {
        state_ = state_ * 1103515245u + 12345u;
        unsigned int bucket = (state_ / 65536u) % 32768u;
        float unit = (float)bucket / 32768.0f; // [0, 1)
        return (unit * 2.0f - 1.0f) * amplitude;
    }

private:
    unsigned int state_;
};

// Drives a simple figure-eight-ish scripted true trajectory forward in time
// and produces noisy IMU/GNSS samples around it. This stands in for a real
// moving vehicle and real sensors.
class SensorSimulator {
public:
    SensorSimulator(unsigned int seed) : imu_noise_(seed + 1u), gnss_noise_(seed + 2u) {
        elapsed_seconds_ = 0.0f;
        true_state_.x = 0.0f;
        true_state_.y = 0.0f;
        true_state_.vx = 1.0f;
        true_state_.vy = 0.0f;
        true_state_.ax = 0.0f;
        true_state_.ay = 0.0f;
        imu_dropout_ = false;
        gnss_dropout_ = false;
    }

    // Advances the true trajectory by dt seconds using a scripted turn rate,
    // so the vehicle flies a gentle S-shaped path at roughly constant speed.
    void Step(float dt) {
        elapsed_seconds_ = elapsed_seconds_ + dt;

        float turn_rate = 0.4f * sinf(0.15f * elapsed_seconds_); // rad/s, scripted
        float speed = sqrtf(true_state_.vx * true_state_.vx + true_state_.vy * true_state_.vy);
        float heading = atan2f(true_state_.vy, true_state_.vx);
        heading = heading + turn_rate * dt;

        float new_vx = speed * cosf(heading);
        float new_vy = speed * sinf(heading);

        true_state_.ax = (new_vx - true_state_.vx) / dt;
        true_state_.ay = (new_vy - true_state_.vy) / dt;

        true_state_.vx = new_vx;
        true_state_.vy = new_vy;
        true_state_.x = true_state_.x + true_state_.vx * dt;
        true_state_.y = true_state_.y + true_state_.vy * dt;
    }

    ImuSample ReadImu() {
        ImuSample sample;
        sample.valid = !imu_dropout_;
        sample.ax = true_state_.ax + imu_noise_.NextUniform(0.05f);
        sample.ay = true_state_.ay + imu_noise_.NextUniform(0.05f);
        return sample;
    }

    GnssSample ReadGnss() {
        GnssSample sample;
        sample.valid = !gnss_dropout_;
        sample.x = true_state_.x + gnss_noise_.NextUniform(0.5f);
        sample.y = true_state_.y + gnss_noise_.NextUniform(0.5f);
        return sample;
    }

    TrueState GetTrueState() const {
        return true_state_;
    }

    // Fault injection hooks used by FaultInjector.
    void SetImuDropout(bool dropout) {
        imu_dropout_ = dropout;
    }

    void SetGnssDropout(bool dropout) {
        gnss_dropout_ = dropout;
    }

private:
    TrueState true_state_;
    NoiseSource imu_noise_;
    NoiseSource gnss_noise_;
    float elapsed_seconds_;
    bool imu_dropout_;
    bool gnss_dropout_;
};

#endif // AUTONOMY_PLATFORM_IMU_GNSS_SIM_HPP
