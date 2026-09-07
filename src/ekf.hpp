// ekf.hpp
//
// A small hand-rolled Extended Kalman Filter for 2D position/velocity
// estimation. State vector is [x, y, vx, vy]. The IMU acceleration drives
// the prediction step and GNSS position drives the update (correction)
// step. Because the motion model here is linear (constant velocity plus a
// known acceleration input) this EKF behaves like a plain linear Kalman
// filter, but it is written with the general predict/update structure so
// it matches how a real EKF would extend to nonlinear dynamics.
//
// No linear algebra library is used. All matrices are plain fixed-size
// float arrays (float[4][4]) and every operation is an explicit,
// beginner-readable loop.

#ifndef AUTONOMY_PLATFORM_EKF_HPP
#define AUTONOMY_PLATFORM_EKF_HPP

using namespace std;

// ---- small 4x4 / 4x1 matrix helpers -------------------------------------

void MatSetIdentity4x4(float m[4][4]) {
    for (int i = 0; i < 4; i = i + 1) {
        for (int j = 0; j < 4; j = j + 1) {
            m[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

void MatCopy4x4(const float src[4][4], float dst[4][4]) {
    for (int i = 0; i < 4; i = i + 1) {
        for (int j = 0; j < 4; j = j + 1) {
            dst[i][j] = src[i][j];
        }
    }
}

void MatMultiply4x4(const float a[4][4], const float b[4][4], float out[4][4]) {
    for (int i = 0; i < 4; i = i + 1) {
        for (int j = 0; j < 4; j = j + 1) {
            float sum = 0.0f;
            for (int k = 0; k < 4; k = k + 1) {
                sum = sum + a[i][k] * b[k][j];
            }
            out[i][j] = sum;
        }
    }
}

void MatTranspose4x4(const float a[4][4], float out[4][4]) {
    for (int i = 0; i < 4; i = i + 1) {
        for (int j = 0; j < 4; j = j + 1) {
            out[j][i] = a[i][j];
        }
    }
}

void MatAdd4x4(const float a[4][4], const float b[4][4], float out[4][4]) {
    for (int i = 0; i < 4; i = i + 1) {
        for (int j = 0; j < 4; j = j + 1) {
            out[i][j] = a[i][j] + b[i][j];
        }
    }
}

void MatSub4x4(const float a[4][4], const float b[4][4], float out[4][4]) {
    for (int i = 0; i < 4; i = i + 1) {
        for (int j = 0; j < 4; j = j + 1) {
            out[i][j] = a[i][j] - b[i][j];
        }
    }
}

// Inverts a 2x2 matrix stored in the top-left corner of a 4x4 array. This is
// the only matrix inverse the EKF needs, since the GNSS measurement is 2D
// (x, y position only).
bool MatInvert2x2(const float m[4][4], float out[4][4]) {
    float a = m[0][0];
    float b = m[0][1];
    float c = m[1][0];
    float d = m[1][1];
    float det = a * d - b * c;
    if (det > -1e-9f && det < 1e-9f) {
        return false;
    }
    float inv_det = 1.0f / det;
    out[0][0] = d * inv_det;
    out[0][1] = -b * inv_det;
    out[1][0] = -c * inv_det;
    out[1][1] = a * inv_det;
    return true;
}

// ---- EKF -----------------------------------------------------------------

struct EkfState {
    float x;
    float y;
    float vx;
    float vy;
};

class Ekf {
public:
    Ekf() {
        state_.x = 0.0f;
        state_.y = 0.0f;
        state_.vx = 0.0f;
        state_.vy = 0.0f;

        MatSetIdentity4x4(covariance_);
        for (int i = 0; i < 4; i = i + 1) {
            covariance_[i][i] = 1.0f;
        }

        process_noise_ = 0.02f;
        gnss_noise_ = 0.5f;
    }

    EkfState GetState() const {
        return state_;
    }

    // Prediction step: propagate state forward using IMU acceleration as the
    // control input, and grow the covariance by the process noise.
    void Predict(float ax, float ay, float dt) {
        state_.x = state_.x + state_.vx * dt + 0.5f * ax * dt * dt;
        state_.y = state_.y + state_.vy * dt + 0.5f * ay * dt * dt;
        state_.vx = state_.vx + ax * dt;
        state_.vy = state_.vy + ay * dt;

        float f[4][4];
        MatSetIdentity4x4(f);
        f[0][2] = dt;
        f[1][3] = dt;

        float f_transpose[4][4];
        MatTranspose4x4(f, f_transpose);

        float temp[4][4];
        MatMultiply4x4(f, covariance_, temp);
        float predicted_covariance[4][4];
        MatMultiply4x4(temp, f_transpose, predicted_covariance);

        for (int i = 0; i < 4; i = i + 1) {
            predicted_covariance[i][i] = predicted_covariance[i][i] + process_noise_;
        }

        MatCopy4x4(predicted_covariance, covariance_);
    }

    // Correction step: fuse a GNSS position fix (x, y) into the state.
    void UpdateWithGnss(float measured_x, float measured_y) {
        float innovation_x = measured_x - state_.x;
        float innovation_y = measured_y - state_.y;

        // Innovation covariance S = H P H^T + R, computed directly since H
        // just selects the position rows.
        float s[4][4];
        for (int i = 0; i < 4; i = i + 1) {
            for (int j = 0; j < 4; j = j + 1) {
                s[i][j] = 0.0f;
            }
        }
        s[0][0] = covariance_[0][0] + gnss_noise_;
        s[0][1] = covariance_[0][1];
        s[1][0] = covariance_[1][0];
        s[1][1] = covariance_[1][1] + gnss_noise_;

        float s_inverse[4][4];
        bool invertible = MatInvert2x2(s, s_inverse);
        if (!invertible) {
            return;
        }

        // Kalman gain K = P H^T S^-1. H^T just picks the first two columns
        // of P, so this is written out directly rather than through a
        // general matrix multiply.
        float k[4][2];
        for (int i = 0; i < 4; i = i + 1) {
            k[i][0] = covariance_[i][0] * s_inverse[0][0] + covariance_[i][1] * s_inverse[1][0];
            k[i][1] = covariance_[i][0] * s_inverse[0][1] + covariance_[i][1] * s_inverse[1][1];
        }

        float state_array[4];
        state_array[0] = state_.x + k[0][0] * innovation_x + k[0][1] * innovation_y;
        state_array[1] = state_.y + k[1][0] * innovation_x + k[1][1] * innovation_y;
        state_array[2] = state_.vx + k[2][0] * innovation_x + k[2][1] * innovation_y;
        state_array[3] = state_.vy + k[3][0] * innovation_x + k[3][1] * innovation_y;

        state_.x = state_array[0];
        state_.y = state_array[1];
        state_.vx = state_array[2];
        state_.vy = state_array[3];

        // Covariance update: P_new = P - K * H * P. H just selects rows 0
        // and 1 of P, so (K * H * P)[i][j] = K[i][0] * P[0][j] + K[i][1] * P[1][j].
        float updated_covariance[4][4];
        for (int i = 0; i < 4; i = i + 1) {
            for (int j = 0; j < 4; j = j + 1) {
                float correction = k[i][0] * covariance_[0][j] + k[i][1] * covariance_[1][j];
                updated_covariance[i][j] = covariance_[i][j] - correction;
            }
        }
        MatCopy4x4(updated_covariance, covariance_);
    }

private:
    EkfState state_;
    float covariance_[4][4];
    float process_noise_;
    float gnss_noise_;
};

#endif // AUTONOMY_PLATFORM_EKF_HPP
