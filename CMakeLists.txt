cmake_minimum_required(VERSION 3.16)
project(RobotArmKinematics VERSION 1.0.0 LANGUAGES CXX)

# ── C++17 ────────────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Compiler warnings ────────────────────────────────────────────────────────
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# ── Static library: rak ──────────────────────────────────────────────────────
add_library(rak_static STATIC
    src/kinematics.cpp
)
target_include_directories(rak_static
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
set_target_properties(rak_static PROPERTIES
    OUTPUT_NAME  rak
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

# ── Shared library: rak ──────────────────────────────────────────────────────
add_library(rak_shared SHARED
    src/kinematics.cpp
)
target_include_directories(rak_shared
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
set_target_properties(rak_shared PROPERTIES
    OUTPUT_NAME  rak
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

# ── CLI binary ───────────────────────────────────────────────────────────────
add_executable(rak_cli
    cli/main.cpp
)
target_link_libraries(rak_cli PRIVATE rak_static)
set_target_properties(rak_cli PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# ── Unit tests ───────────────────────────────────────────────────────────────
enable_testing()

add_executable(test_kinematics
    tests/test_kinematics.cpp
)
target_link_libraries(test_kinematics PRIVATE rak_static)
set_target_properties(test_kinematics PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_test(NAME unit_tests COMMAND ${CMAKE_BINARY_DIR}/bin/test_kinematics)

# ── Doxygen target ───────────────────────────────────────────────────────────
find_package(Doxygen OPTIONAL_COMPONENTS dot)
if(DOXYGEN_FOUND)
    set(DOXYGEN_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/docs)
    set(DOXYGEN_PROJECT_NAME    "Robot Arm Kinematics")
    set(DOXYGEN_PROJECT_NUMBER  ${PROJECT_VERSION})
    set(DOXYGEN_GENERATE_HTML   YES)
    set(DOXYGEN_GENERATE_LATEX  NO)
    set(DOXYGEN_EXTRACT_ALL     YES)
    set(DOXYGEN_HAVE_DOT        ${DOXYGEN_DOT_FOUND})
    doxygen_add_docs(docs
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        COMMENT "Generating API documentation with Doxygen"
    )
    message(STATUS "Doxygen target 'docs' is available.")
else()
    message(STATUS "Doxygen not found — skipping 'docs' target.")
endif()

# ── Install rules ────────────────────────────────────────────────────────────
install(TARGETS rak_static rak_shared rak_cli
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
