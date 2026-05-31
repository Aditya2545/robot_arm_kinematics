/**
 * @file kinematics.cpp
 * @brief Implementation of FK / IK for a 6-DOF serial manipulator.
 */

#include "rak/kinematics.hpp"

#include <algorithm>
#include <cassert>
#include <cstring>
#include <numeric>
#include <sstream>
#include <iomanip>

namespace rak {

// ============================================================
// Low-level linear algebra helpers
// ============================================================

Matrix4d identity() noexcept {
    Matrix4d m{};
    for (std::size_t i = 0; i < 4; ++i)
        for (std::size_t j = 0; j < 4; ++j)
            m[i][j] = (i == j) ? 1.0 : 0.0;
    return m;
}

Matrix4d mat_mul(const Matrix4d& A, const Matrix4d& B) noexcept {
    Matrix4d C{};
    for (std::size_t i = 0; i < 4; ++i)
        for (std::size_t j = 0; j < 4; ++j) {
            double s = 0.0;
            for (std::size_t k = 0; k < 4; ++k)
                s += A[i][k] * B[k][j];
            C[i][j] = s;
        }
    return C;
}

// ============================================================
// Standard DH transform
// ============================================================

Matrix4d dh_transform(const DHParams& p, double q) noexcept {
    const double theta = p.theta + q;
    const double ct = std::cos(theta), st = std::sin(theta);
    const double ca = std::cos(p.alpha), sa = std::sin(p.alpha);

    Matrix4d T{};
    T[0][0] =  ct;      T[0][1] = -st * ca;  T[0][2] =  st * sa;  T[0][3] = p.a * ct;
    T[1][0] =  st;      T[1][1] =  ct * ca;  T[1][2] = -ct * sa;  T[1][3] = p.a * st;
    T[2][0] =  0.0;     T[2][1] =  sa;       T[2][2] =  ca;       T[2][3] = p.d;
    T[3][0] =  0.0;     T[3][1] =  0.0;      T[3][2] =  0.0;      T[3][3] = 1.0;
    return T;
}

// ============================================================
// Forward kinematics
// ============================================================

Matrix4d forward_kinematics_mat(const DHTable& table,
                                 const JointAngles& q) noexcept {
    Matrix4d T = identity();
    for (std::size_t i = 0; i < DOF; ++i)
        T = mat_mul(T, dh_transform(table.joints[i], q[i]));
    return T;
}

std::array<double, 3> rotation_to_euler(const Matrix4d& T) noexcept {
    // ZYX (yaw-pitch-roll) convention
    const double r00 = T[0][0], r10 = T[1][0], r20 = T[2][0];
    const double r21 = T[2][1], r22 = T[2][2];

    double pitch = std::atan2(-r20, std::sqrt(r00 * r00 + r10 * r10));
    double roll, yaw;

    if (std::abs(std::cos(pitch)) < 1e-9) {
        // Gimbal lock
        roll = 0.0;
        yaw  = std::atan2(-T[0][1], T[1][1]);
    } else {
        roll = std::atan2(r21, r22);
        yaw  = std::atan2(r10, r00);
    }
    return {roll, pitch, yaw};
}

Pose forward_kinematics(const DHTable& table,
                        const JointAngles& q) noexcept {
    const auto T = forward_kinematics_mat(table, q);
    auto [roll, pitch, yaw] = rotation_to_euler(T);
    return {T[0][3], T[1][3], T[2][3], roll, pitch, yaw};
}

// ============================================================
// Inverse kinematics — damped least-squares Jacobian
// ============================================================

namespace {

/// Compute the 6×6 geometric Jacobian numerically (central differences).
using Jacobian = std::array<std::array<double, DOF>, 6>;

Jacobian numerical_jacobian(const DHTable& table,
                             const JointAngles& q,
                             double eps = 1e-6) {
    Jacobian J{};
    for (std::size_t j = 0; j < DOF; ++j) {
        JointAngles qp = q, qm = q;
        qp[j] += eps;
        qm[j] -= eps;
        const auto Tp = forward_kinematics(table, qp);
        const auto Tm = forward_kinematics(table, qm);

        J[0][j] = (Tp.x     - Tm.x)     / (2 * eps);
        J[1][j] = (Tp.y     - Tm.y)     / (2 * eps);
        J[2][j] = (Tp.z     - Tm.z)     / (2 * eps);
        J[3][j] = (Tp.roll  - Tm.roll)  / (2 * eps);
        J[4][j] = (Tp.pitch - Tm.pitch) / (2 * eps);
        J[5][j] = (Tp.yaw   - Tm.yaw)   / (2 * eps);
    }
    return J;
}

/// Damped-least-squares: dq = J^T (J J^T + λ²I)^{-1} e
/// We use a simple 6×6 solve via Gaussian elimination.
using Vec6  = std::array<double, 6>;
using Mat66 = std::array<std::array<double, 6>, 6>;

Vec6 solve_dls(const Jacobian& J, const Vec6& e, double lambda) {
    // Build A = J J^T + λ²I   (6×6)
    Mat66 A{};
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c) {
            double s = 0.0;
            for (std::size_t k = 0; k < DOF; ++k)
                s += J[r][k] * J[c][k];
            A[r][c] = s + (r == c ? lambda * lambda : 0.0);
        }

    // Augmented matrix [A | e] → Gaussian elimination
    std::array<std::array<double, 7>, 6> aug{};
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) aug[r][c] = A[r][c];
        aug[r][6] = e[r];
    }
    for (int col = 0; col < 6; ++col) {
        // Pivot
        int pivot = col;
        for (int row = col + 1; row < 6; ++row)
            if (std::abs(aug[row][col]) > std::abs(aug[pivot][col]))
                pivot = row;
        std::swap(aug[col], aug[pivot]);
        if (std::abs(aug[col][col]) < 1e-12) continue;
        double inv = 1.0 / aug[col][col];
        for (int row = 0; row < 6; ++row) {
            if (row == col) continue;
            double factor = aug[row][col] * inv;
            for (int k = col; k <= 6; ++k)
                aug[row][k] -= factor * aug[col][k];
        }
        for (int k = col; k <= 6; ++k)
            aug[col][k] *= inv;
    }
    Vec6 x{};
    for (int r = 0; r < 6; ++r) x[r] = aug[r][6];

    // dq = J^T x
    JointAngles dq{};
    for (std::size_t k = 0; k < DOF; ++k) {
        double s = 0.0;
        for (int r = 0; r < 6; ++r) s += J[r][k] * x[r];
        dq[k] = s;
    }
    Vec6 result{};
    for (std::size_t k = 0; k < DOF; ++k) result[k] = dq[k];
    return result;
}

double angle_diff(double a, double b) {
    double d = a - b;
    while (d >  M_PI) d -= 2 * M_PI;
    while (d < -M_PI) d += 2 * M_PI;
    return d;
}

} // anonymous namespace

std::optional<JointAngles>
inverse_kinematics(const DHTable&     table,
                   const Pose&        target,
                   const JointAngles& q_init,
                   const IKOptions&   opts) {
    JointAngles q = q_init;

    for (int iter = 0; iter < opts.max_iter; ++iter) {
        const Pose cur = forward_kinematics(table, q);

        Vec6 e{};
        e[0] = target.x     - cur.x;
        e[1] = target.y     - cur.y;
        e[2] = target.z     - cur.z;
        e[3] = angle_diff(target.roll,  cur.roll);
        e[4] = angle_diff(target.pitch, cur.pitch);
        e[5] = angle_diff(target.yaw,   cur.yaw);

        // Convergence check
        double pos_err = std::sqrt(e[0]*e[0] + e[1]*e[1] + e[2]*e[2]);
        double ang_err = std::sqrt(e[3]*e[3] + e[4]*e[4] + e[5]*e[5]);
        if (pos_err < opts.position_tol && ang_err < opts.angle_tol)
            return q;

        const auto J  = numerical_jacobian(table, q);
        const auto dq = solve_dls(J, e, opts.damping);

        for (std::size_t k = 0; k < DOF; ++k)
            q[k] = wrap_angle(q[k] + opts.step_size * dq[k]);
    }
    return std::nullopt;
}

// ============================================================
// Utility
// ============================================================

double wrap_angle(double a) noexcept {
    while (a >  M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
}

std::string joint_angles_to_string(const JointAngles& q) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4);
    oss << "q = [";
    for (std::size_t i = 0; i < DOF; ++i) {
        oss << q[i];
        if (i + 1 < DOF) oss << ", ";
    }
    oss << "] rad";
    return oss.str();
}

std::string pose_to_string(const Pose& p) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(5);
    oss << "pos=(" << p.x << ", " << p.y << ", " << p.z << ") m  "
        << "rpy=(" << p.roll << ", " << p.pitch << ", " << p.yaw << ") rad";
    return oss.str();
}

DHTable make_ur5_table() noexcept {
    // UR5 DH parameters (standard, SI units)
    DHTable t;
    t.joints[0] = {0.0,       M_PI / 2.0,   0.089159,  0.0};
    t.joints[1] = {-0.425,    0.0,           0.0,       0.0};
    t.joints[2] = {-0.39225,  0.0,           0.0,       0.0};
    t.joints[3] = {0.0,       M_PI / 2.0,   0.10915,   0.0};
    t.joints[4] = {0.0,      -M_PI / 2.0,   0.09465,   0.0};
    t.joints[5] = {0.0,       0.0,           0.0823,    0.0};
    return t;
}

} // namespace rak
