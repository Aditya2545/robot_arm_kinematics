/**
 * @file main.cpp
 * @brief CLI tool for the Robot Arm Kinematics library.
 *
 * Usage:
 *   rak_cli fk  <q1> <q2> <q3> <q4> <q5> <q6>
 *   rak_cli ik  <x> <y> <z> <roll> <pitch> <yaw>  [q1..q6 seed]
 *   rak_cli roundtrip
 */

#include "rak/kinematics.hpp"

#include <array>
#include <cstdlib>
#include <iostream>
#include <string>
#include <charconv>
#include <iomanip>

static void print_usage(const char* prog) {
    std::cout << "Usage:\n"
              << "  " << prog << " fk <q1> <q2> <q3> <q4> <q5> <q6>   (angles in radians)\n"
              << "  " << prog << " ik <x> <y> <z> <roll> <pitch> <yaw> [seed q1..q6]\n"
              << "  " << prog << " roundtrip                            (self-test)\n";
}

static double parse_double(const char* s) {
    char* end;
    double v = std::strtod(s, &end);
    if (end == s) {
        std::cerr << "Cannot parse '" << s << "' as a number.\n";
        std::exit(1);
    }
    return v;
}

static rak::JointAngles parse_joints(char* argv[], int start) {
    rak::JointAngles q{};
    for (int i = 0; i < 6; ++i)
        q[static_cast<std::size_t>(i)] = parse_double(argv[start + i]);
    return q;
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

static int cmd_fk(int argc, char* argv[]) {
    if (argc < 8) { print_usage(argv[0]); return 1; }

    const auto table = rak::make_ur5_table();
    const auto q     = parse_joints(argv, 2);
    const auto pose  = rak::forward_kinematics(table, q);

    std::cout << "[FK Result]\n"
              << "  Joints : " << rak::joint_angles_to_string(q)  << "\n"
              << "  Pose   : " << rak::pose_to_string(pose)       << "\n";
    return 0;
}

static int cmd_ik(int argc, char* argv[]) {
    if (argc < 8) { print_usage(argv[0]); return 1; }

    const auto table = rak::make_ur5_table();

    rak::Pose target;
    target.x     = parse_double(argv[2]);
    target.y     = parse_double(argv[3]);
    target.z     = parse_double(argv[4]);
    target.roll  = parse_double(argv[5]);
    target.pitch = parse_double(argv[6]);
    target.yaw   = parse_double(argv[7]);

    // Seed: user-supplied or zeros
    rak::JointAngles seed{};
    if (argc >= 14)
        seed = parse_joints(argv, 8);

    rak::IKOptions opts;
    const auto result = rak::inverse_kinematics(table, target, seed, opts);

    if (!result) {
        std::cerr << "[IK] Solver did not converge.\n";
        return 2;
    }

    const auto achieved_pose = rak::forward_kinematics(table, *result);

    std::cout << "[IK Result]\n"
              << "  Joints  : " << rak::joint_angles_to_string(*result)   << "\n"
              << "  Target  : " << rak::pose_to_string(target)            << "\n"
              << "  Achieved: " << rak::pose_to_string(achieved_pose)     << "\n";
    return 0;
}

static int cmd_roundtrip(int /*argc*/, char* /*argv*/[]) {
    const auto table = rak::make_ur5_table();

    struct TestCase {
        const char* name;
        rak::JointAngles q;
    };

    const std::array<TestCase, 5> cases = {{
        {"Home",      {0.0,  0.0,  0.0,  0.0,  0.0,  0.0}},
        {"Elbow-up",  {0.5, -1.0,  1.2, -0.5,  0.3, -0.2}},
        {"Reach-out", {1.0,  0.0, -0.5,  0.0, -1.0,  0.0}},
        {"Wrist-bent",{0.0, -1.57, 1.57, -1.57, 1.57, 0.0}},
        {"Random",    {-0.8,  0.4, -1.1,  0.7, -0.3,  1.5}},
    }};

    constexpr double POS_EPS = 1e-3;
    constexpr double ANG_EPS = 1e-3;

    bool all_pass = true;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "=== IK → FK Roundtrip Test ===\n\n";

    for (const auto& tc : cases) {
        const auto fk_pose   = rak::forward_kinematics(table, tc.q);
        rak::IKOptions rt_opts;
        rt_opts.max_iter   = 500;
        rt_opts.damping    = 5e-4;
        rt_opts.step_size  = 0.3;
        // Use a slightly perturbed version of true q as seed (realistic scenario)
        rak::JointAngles seed = tc.q;
        for (auto& qi : seed) qi += 0.1;   // small offset
        const auto ik_result = rak::inverse_kinematics(table, fk_pose, seed, rt_opts);
        if (!ik_result) {
            std::cout << "FAIL [" << tc.name << "] IK did not converge\n";
            all_pass = false;
            continue;
        }
        const auto roundtrip_pose = rak::forward_kinematics(table, *ik_result);

        double dp = std::sqrt(
            (fk_pose.x - roundtrip_pose.x) * (fk_pose.x - roundtrip_pose.x) +
            (fk_pose.y - roundtrip_pose.y) * (fk_pose.y - roundtrip_pose.y) +
            (fk_pose.z - roundtrip_pose.z) * (fk_pose.z - roundtrip_pose.z));

        auto da = [](double a, double b) {
            double d = a - b;
            while (d >  M_PI) d -= 2 * M_PI;
            while (d < -M_PI) d += 2 * M_PI;
            return std::abs(d);
        };
        double da_max = std::max(da(fk_pose.roll,  roundtrip_pose.roll),
                          std::max(da(fk_pose.pitch, roundtrip_pose.pitch),
                                   da(fk_pose.yaw,   roundtrip_pose.yaw)));

        bool pass = (dp < POS_EPS) && (da_max < ANG_EPS);
        std::cout << (pass ? "PASS" : "FAIL")
                  << " [" << tc.name << "]"
                  << "  pos_err=" << dp
                  << "  ang_err=" << da_max << "\n";
        if (!pass) all_pass = false;
    }

    std::cout << "\n" << (all_pass ? "All tests passed." : "Some tests FAILED.") << "\n";
    return all_pass ? 0 : 1;
}

// ---------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    if (argc < 2) { print_usage(argv[0]); return 1; }

    const std::string cmd = argv[1];
    if      (cmd == "fk")        return cmd_fk(argc, argv);
    else if (cmd == "ik")        return cmd_ik(argc, argv);
    else if (cmd == "roundtrip") return cmd_roundtrip(argc, argv);
    else {
        std::cerr << "Unknown command: " << cmd << "\n";
        print_usage(argv[0]);
        return 1;
    }
}
