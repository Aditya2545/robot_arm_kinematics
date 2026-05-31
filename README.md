# Robot Arm Kinematics (RAK)

A **standalone C++17** forward/inverse kinematics library for a 6-DOF serial manipulator, built entirely without ROS.

## Features

| Feature | Details |
|---|---|
| **Forward Kinematics** | Standard Denavit-Hartenberg transforms |
| **Inverse Kinematics** | Damped-least-squares numerical Jacobian |
| **Robot model** | UR5-like, swappable via `DHTable` |
| **Static + shared lib** | Both built from the same source |
| **CLI binary** | `rak_cli fk / ik / roundtrip` |
| **Unit tests** | Zero-dependency harness |
| **Bash regression** | IK→FK roundtrip within epsilon |
| **Doxygen** | Wired into CMake `docs` target |

## Build (Ubuntu 22.04)

```bash
# Prerequisites
sudo apt install cmake g++ doxygen graphviz

# Configure + build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel

# Run unit tests
ctest --test-dir build --output-on-failure

# Generate docs
cmake --build build --target docs
```

## Usage

```bash
# Forward kinematics
./build/bin/rak_cli fk 0 0 0 0 0 0

# Inverse kinematics
./build/bin/rak_cli ik 0.3 0.1 0.4 0 1.57 0

# Built-in roundtrip self-test
./build/bin/rak_cli roundtrip

# Bash regression suite
bash scripts/regression_test.sh
```

## Library API

```cpp
#include <rak/kinematics.hpp>

auto table = rak::make_ur5_table();

// Forward kinematics
rak::JointAngles q = {0.5, -1.0, 1.2, -0.5, 0.3, -0.2};
rak::Pose pose = rak::forward_kinematics(table, q);

// Inverse kinematics
auto result = rak::inverse_kinematics(table, pose, /*seed=*/{});
if (result) {
    // *result  →  rak::JointAngles
}
```

## Project Structure

```
robot_arm_kinematics/
├── include/rak/kinematics.hpp   # Public API
├── src/kinematics.cpp           # Implementation
├── cli/main.cpp                 # CLI tool
├── tests/test_kinematics.cpp    # Unit tests
├── scripts/regression_test.sh   # Bash IK→FK harness
├── docs/Doxyfile                # Doxygen config
└── CMakeLists.txt               # Build system
```

## C++17 Highlights Used

- `std::array` — fixed-size joint and matrix storage
- Structured bindings — `auto [roll, pitch, yaw] = rotation_to_euler(T)`
- `if constexpr` — compile-time joint-count dispatch
- `std::optional<JointAngles>` — clean IK failure signalling
- `[[nodiscard]]` attributes throughout the public API

## License

MIT
