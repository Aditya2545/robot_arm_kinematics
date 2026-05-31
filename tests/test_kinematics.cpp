/**
 * @file test_kinematics.cpp
 * @brief Lightweight unit tests (no external framework).
 */

#include "rak/kinematics.hpp"

#include <cmath>
#include <iostream>
#include <string>

static int g_pass = 0, g_fail = 0;

#define EXPECT_NEAR(a, b, eps, name)                                     \
    do {                                                                  \
        double _a = (a), _b = (b), _e = (eps);                          \
        if (std::abs(_a - _b) <= _e) {                                   \
            ++g_pass;                                                     \
            std::cout << "  PASS  " << (name) << "\n";                  \
        } else {                                                          \
            ++g_fail;                                                     \
            std::cout << "  FAIL  " << (name)                           \
                      << "  got=" << _a << "  exp=" << _b               \
                      << "  diff=" << std::abs(_a-_b) << "\n";          \
        }                                                                 \
    } while(false)

#define EXPECT_TRUE(cond, name)                                          \
    do {                                                                  \
        if (cond) { ++g_pass; std::cout << "  PASS  " << (name) << "\n"; } \
        else      { ++g_fail; std::cout << "  FAIL  " << (name) << "\n"; } \
    } while(false)

void test_identity_fk() {
    std::cout << "\n[test_identity_fk]\n";
    const auto table = rak::make_ur5_table();
    rak::JointAngles q{};
    const auto pose = rak::forward_kinematics(table, q);
    EXPECT_TRUE(!std::isnan(pose.x), "pose.x is not NaN");
    EXPECT_TRUE(!std::isnan(pose.y), "pose.y is not NaN");
    EXPECT_TRUE(!std::isnan(pose.z), "pose.z is not NaN");
    // UR5 home: end-effector reaches far out along x
    EXPECT_TRUE(std::abs(pose.x) > 0.1, "pose.x non-trivial at home");
}

void test_dh_transform_identity() {
    std::cout << "\n[test_dh_transform]\n";
    rak::DHParams p{0, 0, 0, 0};
    const auto T = rak::dh_transform(p, 0.0);
    EXPECT_NEAR(T[0][0], 1.0, 1e-9, "T[0][0]==1");
    EXPECT_NEAR(T[1][1], 1.0, 1e-9, "T[1][1]==1");
    EXPECT_NEAR(T[2][2], 1.0, 1e-9, "T[2][2]==1");
    EXPECT_NEAR(T[3][3], 1.0, 1e-9, "T[3][3]==1");
    EXPECT_NEAR(T[0][3], 0.0, 1e-9, "tx==0");
    EXPECT_NEAR(T[1][3], 0.0, 1e-9, "ty==0");
    EXPECT_NEAR(T[2][3], 0.0, 1e-9, "tz==0");
}

void test_mat_mul_identity() {
    std::cout << "\n[test_mat_mul_identity]\n";
    const auto I = rak::identity();
    const auto R = rak::mat_mul(I, I);
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            EXPECT_NEAR(R[i][j], I[i][j], 1e-12, "I*I element");
}

void test_wrap_angle() {
    std::cout << "\n[test_wrap_angle]\n";
    EXPECT_NEAR(rak::wrap_angle(0.0),            0.0,            1e-9, "wrap 0");
    EXPECT_NEAR(rak::wrap_angle(M_PI + 0.001),  -M_PI + 0.001,  1e-3, "wrap >pi");
    EXPECT_NEAR(rak::wrap_angle(-M_PI - 0.001),  M_PI - 0.001,  1e-3, "wrap <-pi");
}

void test_fk_ik_roundtrip() {
    std::cout << "\n[test_fk_ik_roundtrip]\n";
    const auto table = rak::make_ur5_table();

    struct Case { const char* name; rak::JointAngles q; };
    Case cases[] = {
        {"home",    {0.0,  0.0,   0.0,  0.0,  0.0,  0.0}},
        {"elbow",   {0.5, -1.0,   1.2, -0.5,  0.3, -0.2}},
        {"wrist",   {0.0, -1.57,  1.57, -1.57, 1.57, 0.0}},
    };

    for (const auto& c : cases) {
        const auto target  = rak::forward_kinematics(table, c.q);
        const auto ik_res  = rak::inverse_kinematics(table, target, {});
        EXPECT_TRUE(ik_res.has_value(), std::string("IK converged: ") + c.name);
        if (!ik_res) continue;

        const auto achieved = rak::forward_kinematics(table, *ik_res);
        double dp = std::sqrt(
            (target.x - achieved.x)*(target.x - achieved.x) +
            (target.y - achieved.y)*(target.y - achieved.y) +
            (target.z - achieved.z)*(target.z - achieved.z));
        EXPECT_NEAR(dp, 0.0, 1e-3, std::string("pos_err < 1mm: ") + c.name);
    }
}

void test_fk_deterministic() {
    std::cout << "\n[test_fk_deterministic]\n";
    const auto table = rak::make_ur5_table();
    rak::JointAngles q = {0.5, -1.0, 1.2, -0.5, 0.3, -0.2};
    const auto p1 = rak::forward_kinematics(table, q);
    const auto p2 = rak::forward_kinematics(table, q);
    EXPECT_NEAR(p1.x, p2.x, 1e-12, "FK deterministic x");
    EXPECT_NEAR(p1.y, p2.y, 1e-12, "FK deterministic y");
    EXPECT_NEAR(p1.z, p2.z, 1e-12, "FK deterministic z");
}

int main() {
    std::cout << "=== Robot Arm Kinematics Unit Tests ===\n";
    test_identity_fk();
    test_dh_transform_identity();
    test_mat_mul_identity();
    test_wrap_angle();
    test_fk_ik_roundtrip();
    test_fk_deterministic();

    std::cout << "\n--- Summary ---\n"
              << "  Passed : " << g_pass << "\n"
              << "  Failed : " << g_fail << "\n";
    return (g_fail == 0) ? 0 : 1;
}
