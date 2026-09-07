// test_ekf.cpp
//
// AI-assisted (Claude): this test was developed with AI assistance.
//
// Checks that the EKF's position estimate converges toward a known true
// trajectory when fed simulated IMU acceleration and noisy GNSS fixes.

#include <cstdio>
#include <cmath>

using namespace std;

#include "../src/ekf.hpp"

int g_failures = 0;

void Check(bool condition, const char *message) {
    if (!condition) {
        printf("FAIL: %s\n", message);
        g_failures = g_failures + 1;
    } else {
        printf("PASS: %s\n", message);
    }
}

int main() {
    // True trajectory: constant acceleration of 1.0 m/s^2 along x, starting
    // from rest at the origin. True position at time t is 0.5 * a * t^2.
    Ekf ekf;
    float dt = 0.02f;
    float true_ax = 1.0f;
    float true_x = 0.0f;
    float true_vx = 0.0f;

    unsigned int noise_state = 12345u;

    for (int tick = 0; tick < 500; tick = tick + 1) {
        true_vx = true_vx + true_ax * dt;
        true_x = true_x + true_vx * dt;

        // Slightly noisy IMU reading of the known true acceleration.
        noise_state = noise_state * 1103515245u + 12345u;
        float imu_noise = (((float)((noise_state / 65536u) % 1000u) / 1000.0f) - 0.5f) * 0.05f;
        ekf.Predict(true_ax + imu_noise, 0.0f, dt);

        // A GNSS fix arrives every 10 ticks (5 Hz), noisy around true position.
        if (tick % 10 == 0) {
            noise_state = noise_state * 1103515245u + 12345u;
            float gnss_noise = (((float)((noise_state / 65536u) % 1000u) / 1000.0f) - 0.5f) * 0.5f;
            ekf.UpdateWithGnss(true_x + gnss_noise, 0.0f);
        }
    }

    EkfState final_state = ekf.GetState();
    float position_error = fabsf(final_state.x - true_x);
    float velocity_error = fabsf(final_state.vx - true_vx);

    printf("true position: %.3f, estimated position: %.3f (error %.3f)\n", true_x, final_state.x, position_error);
    printf("true velocity: %.3f, estimated velocity: %.3f (error %.3f)\n", true_vx, final_state.vx, velocity_error);

    Check(position_error < 1.0f, "EKF position estimate converges within 1.0 m of true position");
    Check(velocity_error < 1.0f, "EKF velocity estimate converges within 1.0 m/s of true velocity");
    Check(fabsf(final_state.y) < 1.0f, "EKF y estimate stays near zero when no y motion occurs");

    if (g_failures == 0) {
        printf("\nAll test_ekf checks passed.\n");
        return 0;
    }
    printf("\n%d test_ekf check(s) failed.\n", g_failures);
    return 1;
}
