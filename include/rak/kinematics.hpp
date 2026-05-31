#pragma once
/**
 * @file kinematics.hpp
 * @brief 6-DOF Robot Arm Forward/Inverse Kinematics Library
 *
 * Provides Denavit-Hartenberg (DH) parameter based FK and
 * numerical Jacobian-based IK for a 6-DOF serial manipulator.
 *
 * @author  RAK Project
 * @version 1.0.0
 */

#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <stdexcept>

namespace rak {

/// Number of joints in the manipulator
static constexpr std::size_t DOF = 6;

/// Joint angle array type
using JointAngles = std::array<double, DOF>;

/// 4x4 homogeneous transformation matrix stored in row-major order
using Matrix4d = std::array<std::array<double, 4>, 4>;

// ---------------------------------------------------------------------------
// Denavit-Hartenberg parameters
// ---------------------------------------------------------------------------

/**
 * @brief Standard DH parameters for one joint.
 *
 * Convention: T = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
 */
struct DHParams {
    double a;      ///< Link length        [m]
    double alpha;  ///< Link twist         [rad]
    double d;      ///< Link offset        [m]
    double theta;  ///< Joint angle offset [rad]  (added to q_i at runtime)
};

/**
 * @brief Complete DH table for a 6-DOF arm.
 *
 * Resembles a UR5-like configuration but is standalone / generic.
 */
struct DHTable {
    std::array<DHParams, DOF> joints;
};

// ---------------------------------------------------------------------------
// 3-D pose
// ---------------------------------------------------------------------------

/**
 * @brief Cartesian end-effector pose.
 */
struct Pose {
    double x{0}, y{0}, z{0};          ///< Position [m]
    double roll{0}, pitch{0}, yaw{0}; ///< Euler ZYX angles [rad]
};

// ---------------------------------------------------------------------------
// IK solver options
// ---------------------------------------------------------------------------

/**
 * @brief Configuration for the iterative IK solver.
 */
struct IKOptions {
    double position_tol{1e-4};   ///< Position tolerance [m]
    double angle_tol{1e-4};      ///< Orientation tolerance [rad]
    int    max_iter{200};        ///< Maximum iterations
    double step_size{0.5};       ///< Jacobian damping / step scale
    double damping{1e-3};        ///< Levenberg-Marquardt damping
};

// ---------------------------------------------------------------------------
// Core API
// ---------------------------------------------------------------------------

/**
 * @brief Build the standard DH matrix for joint i given q_i.
 * @param p  DH parameters for this joint
 * @param q  Joint angle [rad]
 * @return   4×4 homogeneous transform
 */
[[nodiscard]] Matrix4d dh_transform(const DHParams& p, double q) noexcept;

/**
 * @brief Multiply two 4×4 homogeneous transforms.
 */
[[nodiscard]] Matrix4d mat_mul(const Matrix4d& A, const Matrix4d& B) noexcept;

/**
 * @brief Identity 4×4 matrix.
 */
[[nodiscard]] Matrix4d identity() noexcept;

/**
 * @brief Forward kinematics — compute end-effector pose from joint angles.
 * @param table  DH parameter table
 * @param q      Joint angles [rad]
 * @return       End-effector pose
 */
[[nodiscard]] Pose forward_kinematics(const DHTable& table,
                                      const JointAngles& q) noexcept;

/**
 * @brief Forward kinematics returning the raw 4×4 transform.
 * @param table  DH parameter table
 * @param q      Joint angles [rad]
 * @return       4×4 homogeneous transform of end-effector w.r.t. base
 */
[[nodiscard]] Matrix4d forward_kinematics_mat(const DHTable& table,
                                               const JointAngles& q) noexcept;

/**
 * @brief Inverse kinematics — find joint angles for a desired pose.
 *
 * Uses a damped-least-squares Jacobian approach.
 *
 * @param table    DH parameter table
 * @param target   Desired end-effector pose
 * @param q_init   Initial joint-angle seed
 * @param opts     Solver options
 * @return         Joint angles on success, std::nullopt on failure
 */
[[nodiscard]] std::optional<JointAngles>
inverse_kinematics(const DHTable&     table,
                   const Pose&        target,
                   const JointAngles& q_init,
                   const IKOptions&   opts = {});

// ---------------------------------------------------------------------------
// Utility helpers
// ---------------------------------------------------------------------------

/**
 * @brief Convert a rotation matrix to ZYX Euler angles.
 * @param R  Top-left 3×3 of a homogeneous matrix
 * @return   {roll, pitch, yaw} [rad]
 */
[[nodiscard]] std::array<double, 3>
rotation_to_euler(const Matrix4d& R) noexcept;

/**
 * @brief Wrap angle to [-π, π].
 */
[[nodiscard]] double wrap_angle(double a) noexcept;

/**
 * @brief Pretty-print joint angles to a string.
 */
[[nodiscard]] std::string joint_angles_to_string(const JointAngles& q);

/**
 * @brief Pretty-print a pose to a string.
 */
[[nodiscard]] std::string pose_to_string(const Pose& p);

/**
 * @brief Create a default UR5-like DH table.
 */
[[nodiscard]] DHTable make_ur5_table() noexcept;

} // namespace rak

/**
 * @mainpage Robot Arm Kinematics (RAK)
 *
 * @section intro Introduction
 * Standalone C++17 forward/inverse kinematics library for a 6-DOF serial manipulator.
 * No ROS. No Eigen. Pure standard library.
 *
 * @section features Features
 * - Forward kinematics via Denavit-Hartenberg transforms
 * - Inverse kinematics via damped-least-squares numerical Jacobian
 * - Built-in UR5 model (swappable via DHTable)
 * - Static + shared library from one CMake build
 * - CLI tool: rak_cli fk / ik / roundtrip
 *
 * @section usage Quick Start
 * @code
 * auto table = rak::make_ur5_table();
 * rak::JointAngles q = {0.5, -1.0, 1.2, -0.5, 0.3, -0.2};
 * rak::Pose pose = rak::forward_kinematics(table, q);
 * auto result   = rak::inverse_kinematics(table, pose, {});
 * @endcode
 */
