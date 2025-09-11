#ifndef _GEOMETRY_TEST_ROTATION_
#define _GEOMETRY_TEST_ROTATION_
#include <gtest/gtest.h>

// Include all the necessary headers from your library
#include "../tol.h"
#include "refx/geometry.h"
#include "refx/math.h"

// This preprocessor check is the key to conditional compilation.
// The Eigen-specific tests will only be compiled if CMake has found Eigen
// and defined the REFX_ENABLE_EIGEN_SUPPORT macro.
#ifdef REFX_ENABLE_EIGEN_SUPPORT
#include <Eigen/Dense>
#endif

// Use a consistent namespace for tests
namespace refx {
namespace testing {

/**
 * @test Test fixture for Rotation, UnitQuaternion, and EulerAngles.
 * This suite verifies the correctness of orientation representations and
 * the frame-safe transformation logic.
 */
class RotationTest : public ::testing::Test {};

using UnitQuaterniond = UnitQuaternion<double>;
// --- UnitQuaternion Tests ---

TEST_F(RotationTest, UnitQuaternionDefaultConstructorIsIdentity) {
    UnitQuaterniond q;
    EXPECT_NEAR(q.w(), 1.0, HARD_TOLERANCE);
    EXPECT_NEAR(q.x(), 0.0, HARD_TOLERANCE);
    EXPECT_NEAR(q.y(), 0.0, HARD_TOLERANCE);
    EXPECT_NEAR(q.z(), 0.0, HARD_TOLERANCE);
}

/// @brief @brief verifies that the brace initializer list, assigns the params in the right order to
/// base class
TEST_F(RotationTest, UnitQuaternionBraceInit) {
    // not mathematically a quaternion (just check the parameter passing)
    UnitQuaterniond q = {1, 2, 3, 4};

    EXPECT_NEAR(q.x(), 2.0, HARD_TOLERANCE);
    EXPECT_NEAR(q.y(), 3.0, HARD_TOLERANCE);
    EXPECT_NEAR(q.z(), 4.0, HARD_TOLERANCE);
    EXPECT_NEAR(q.w(), 1.0, HARD_TOLERANCE);
}

TEST_F(RotationTest, UnitQuaternionMultiplication) {
    // 90-degree rotation about Z-axis
    UnitQuaterniond q1(std::cos(M_PI / 4.0), 0, 0, std::sin(M_PI / 4.0));
    // 90-degree rotation about X-axis
    UnitQuaterniond q2(std::cos(M_PI / 4.0), std::sin(M_PI / 4.0), 0, 0);

    // Compose the rotations: q1 * q2
    UnitQuaterniond q_comp = q1 * q2;

    // Expected result from manual calculation
    EXPECT_NEAR(q_comp.w(), 0.5, HARD_TOLERANCE);
    EXPECT_NEAR(q_comp.x(), 0.5, HARD_TOLERANCE);
    EXPECT_NEAR(q_comp.y(), 0.5, HARD_TOLERANCE);
    EXPECT_NEAR(q_comp.z(), 0.5, HARD_TOLERANCE);
}

// --- EulerAngles Tests ---

TEST_F(RotationTest, EulerAnglesZYXConstructorAndAccessors) {
    YawPitchRoll<double> rpy(deg2rad(10.0), deg2rad(20.0), deg2rad(30.0));  // Yaw, Pitch, Roll

    EXPECT_NEAR(rpy.yaw(), deg2rad(10.0), HARD_TOLERANCE);
    EXPECT_NEAR(rpy.pitch(), deg2rad(20.0), HARD_TOLERANCE);
    EXPECT_NEAR(rpy.roll(), deg2rad(30.0), HARD_TOLERANCE);
}

// --- Rotation Class Tests ---

TEST_F(RotationTest, RotationDefaultConstructorIsIdentity) {
    Rotation<ned, frd> R;
    UnitQuaterniond q = R.to_quaternion();
    EXPECT_NEAR(q.w(), 1.0, HARD_TOLERANCE);
    EXPECT_NEAR(q.x(), 0.0, HARD_TOLERANCE);
    EXPECT_NEAR(q.y(), 0.0, HARD_TOLERANCE);
    EXPECT_NEAR(q.z(), 0.0, HARD_TOLERANCE);
}

TEST_F(RotationTest, FromElementaryRotationX) {
    // Create a simple 40-degree yaw rotation
    EulerAngles<EulerSequence::ZYX, double> eul(deg2rad(0.0), deg2rad(0.0), deg2rad(40.0));
    auto R = Rotation<ned, frd>(eul);

    UnitQuaterniond q = R.to_quaternion();
    // Expected quaternion for 40-deg yaw (Z-axis rotation)
    EXPECT_NEAR(q.x(), 0.3420201433, HARD_TOLERANCE);
    EXPECT_NEAR(q.y(), 0.0, HARD_TOLERANCE);
    EXPECT_NEAR(q.z(), 0.0, HARD_TOLERANCE);
    EXPECT_NEAR(q.w(), 0.9396926208, HARD_TOLERANCE);
}

TEST_F(RotationTest, FromElementaryRotationY) {
    // Create a simple -35-degree pitch rotation
    EulerAngles<EulerSequence::ZYX, double> eul(deg2rad(0.0), deg2rad(-35.0), deg2rad(0.0));
    auto R = Rotation<ned, frd>(eul);

    UnitQuaterniond q = R.to_quaternion();
    // Expected quaternion for -35-degree pitch (Y-axis rotation)
    EXPECT_NEAR(q.x(), 0.0, HARD_TOLERANCE);
    EXPECT_NEAR(q.y(), -0.3007057995, HARD_TOLERANCE);
    EXPECT_NEAR(q.z(), 0.0, HARD_TOLERANCE);
    EXPECT_NEAR(q.w(), 0.9537169507, HARD_TOLERANCE);
}

TEST_F(RotationTest, FromElementaryRotationZ) {
    // Create a simple 50-degree roll rotation
    EulerAngles<EulerSequence::ZYX, double> eul(deg2rad(50.0), deg2rad(0.0), deg2rad(0.0));
    auto R = Rotation<ned, frd>(eul);

    UnitQuaterniond q = R.to_quaternion();
    // Expected quaternion for 50-degree roll (X-axis rotation)
    EXPECT_NEAR(q.x(), 0.0, HARD_TOLERANCE);
    EXPECT_NEAR(q.y(), 0.0, HARD_TOLERANCE);
    EXPECT_NEAR(q.z(), 0.4226182617, HARD_TOLERANCE);
    EXPECT_NEAR(q.w(), 0.906307787, HARD_TOLERANCE);
}

TEST_F(RotationTest, FromEulerAnglesZYX) {
    // Create a complex RPY rotation
    EulerAngles<EulerSequence::ZYX, double> eul(deg2rad(50.0), deg2rad(-35.0), deg2rad(40.0));
    auto R = Rotation<ned, frd>(eul);
    UnitQuaterniond q = R.to_quaternion();
    // Expected quaternion for RPY rotation (All-axes rotation)
    // These values are found using scipy.spatial.transform and the function
    // `Rotation.from_euler('ZYX', ...)`
    EXPECT_NEAR(q.x(), 0.4150485806, HARD_TOLERANCE);
    EXPECT_NEAR(q.y(), -0.1182422933, HARD_TOLERANCE);
    EXPECT_NEAR(q.z(), 0.4719622525, HARD_TOLERANCE);
    EXPECT_NEAR(q.w(), 0.7687685399, HARD_TOLERANCE);
}

TEST_F(RotationTest, FromEulerAnglesZXY) {
    // Create a simple 90-degree yaw rotation
    EulerAngles<EulerSequence::ZXY, double> eul(deg2rad(50.0), deg2rad(-35.0), deg2rad(40.0));
    auto R = Rotation<ned, frd>(eul);

    UnitQuaterniond q = R.to_quaternion();
    // Expected quaternion for 90-deg yaw (Z-axis rotation)
    // These values are found using scipy.spatial.transform and the function
    // `Rotation.from_euler('ZXY', ...)`
    EXPECT_NEAR(q.x(), -0.3939503399, HARD_TOLERANCE);
    EXPECT_NEAR(q.y(), 0.1762092334, HARD_TOLERANCE);
    EXPECT_NEAR(q.z(), 0.2855393799, HARD_TOLERANCE);
    EXPECT_NEAR(q.w(), 0.8556989531, HARD_TOLERANCE);
}

TEST_F(RotationTest, FromEulerAnglesXYZ) {
    // Create a simple 90-degree yaw rotation
    EulerAngles<EulerSequence::XYZ, double> eul(deg2rad(50.0), deg2rad(-35.0), deg2rad(40.0));
    auto R = Rotation<ned, frd>(eul);
    UnitQuaterniond q = R.to_quaternion();
    // Expected quaternion for 90-deg yaw (Z-axis rotation)
    // These values are found using scipy.spatial.transform and the function
    // `Rotation.from_euler('XYZ', ...)`
    EXPECT_NEAR(q.x(), 0.2855393799, HARD_TOLERANCE);
    EXPECT_NEAR(q.y(), -0.3939503399, HARD_TOLERANCE);
    EXPECT_NEAR(q.z(), 0.1762092334, HARD_TOLERANCE);
    EXPECT_NEAR(q.w(), 0.8556989531, HARD_TOLERANCE);
}

TEST_F(RotationTest, FromEulerAnglesXZY) {
    // Create a simple 90-degree yaw rotation
    EulerAngles<EulerSequence::XZY, double> eul(deg2rad(50.0), deg2rad(-35.0), deg2rad(40.0));
    auto R = Rotation<ned, frd>(eul);

    UnitQuaterniond q = R.to_quaternion();
    // Expected quaternion for 90-deg yaw (Z-axis rotation)
    // These values are found using scipy.spatial.transform and the function
    // `Rotation.from_euler('XZY', ...)`
    EXPECT_NEAR(q.x(), 0.4719622525, HARD_TOLERANCE);
    EXPECT_NEAR(q.y(), 0.4150485806, HARD_TOLERANCE);
    EXPECT_NEAR(q.z(), -0.1182422933, HARD_TOLERANCE);
    EXPECT_NEAR(q.w(), 0.7687685399, HARD_TOLERANCE);
}

TEST_F(RotationTest, FromEulerAnglesYZX) {
    // Create a simple 90-degree yaw rotation
    EulerAngles<EulerSequence::YZX, double> eul(deg2rad(50.0), deg2rad(-35.0), deg2rad(40.0));
    auto R = Rotation<ned, frd>(eul);

    UnitQuaterniond q = R.to_quaternion();
    // Expected quaternion for 90-deg yaw (Z-axis rotation)
    // These values are found using scipy.spatial.transform and the function
    // `Rotation.from_euler('YZX', ...)`
    EXPECT_NEAR(q.x(), 0.1762092334, HARD_TOLERANCE);
    EXPECT_NEAR(q.y(), 0.2855393799, HARD_TOLERANCE);
    EXPECT_NEAR(q.z(), -0.3939503399, HARD_TOLERANCE);
    EXPECT_NEAR(q.w(), 0.8556989531, HARD_TOLERANCE);
}

TEST_F(RotationTest, FromEulerAnglesYXZ) {
    // Create a simple 90-degree yaw rotation
    EulerAngles<EulerSequence::YXZ, double> eul(deg2rad(50.0), deg2rad(-35.0), deg2rad(40.0));
    auto R = Rotation<ned, frd>(eul);

    UnitQuaterniond q = R.to_quaternion();
    // Expected quaternion for 90-deg yaw (Z-axis rotation)
    // These values are found using scipy.spatial.transform and the function
    // `Rotation.from_euler('YXZ', ...)`
    EXPECT_NEAR(q.x(), -0.1182422933, HARD_TOLERANCE);
    EXPECT_NEAR(q.y(), 0.4719622525, HARD_TOLERANCE);
    EXPECT_NEAR(q.z(), 0.4150485806, HARD_TOLERANCE);
    EXPECT_NEAR(q.w(), 0.7687685399, HARD_TOLERANCE);
}

TEST_F(RotationTest, EulerAnglesInverse) {
    double ang1 = deg2rad(-25.0), ang2 = deg2rad(40.0), ang3 = deg2rad(35.0);
    // Test ZYX
    {
        EulerAngles<EulerSequence::ZYX, double> original_rpy(ang1, ang2, ang3);
        auto q = euler_to_quat(original_rpy);
        auto qinv_R = q.conjugate();
        auto qinv_res = euler_to_quat(original_rpy.inverse());

        EXPECT_NEAR(qinv_res.w(), qinv_R.w(), HARD_TOLERANCE);
        EXPECT_NEAR(qinv_res.x(), qinv_R.x(), HARD_TOLERANCE);
        EXPECT_NEAR(qinv_res.y(), qinv_R.y(), HARD_TOLERANCE);
        EXPECT_NEAR(qinv_res.y(), qinv_R.y(), HARD_TOLERANCE);
    }
    // Test ZXY
    {
        EulerAngles<EulerSequence::ZXY, double> original_rpy(ang1, ang2, ang3);
        auto q = euler_to_quat(original_rpy);
        auto qinv_R = q.conjugate();
        auto qinv_res = euler_to_quat(original_rpy.inverse());

        EXPECT_NEAR(qinv_res.w(), qinv_R.w(), HARD_TOLERANCE);
        EXPECT_NEAR(qinv_res.x(), qinv_R.x(), HARD_TOLERANCE);
        EXPECT_NEAR(qinv_res.y(), qinv_R.y(), HARD_TOLERANCE);
        EXPECT_NEAR(qinv_res.z(), qinv_R.z(), HARD_TOLERANCE);
    }
    // Test YXZ
    {
        EulerAngles<EulerSequence::YXZ, double> original_rpy(ang1, ang2, ang3);
        auto q = euler_to_quat(original_rpy);
        auto qinv_R = q.conjugate();
        auto qinv_res = euler_to_quat(original_rpy.inverse());

        EXPECT_NEAR(qinv_res.w(), qinv_R.w(), HARD_TOLERANCE);
        EXPECT_NEAR(qinv_res.x(), qinv_R.x(), HARD_TOLERANCE);
        EXPECT_NEAR(qinv_res.y(), qinv_R.y(), HARD_TOLERANCE);
        EXPECT_NEAR(qinv_res.z(), qinv_R.z(), HARD_TOLERANCE);
    }
    // Test YZX
    {
        EulerAngles<EulerSequence::YZX, double> original_rpy(ang1, ang2, ang3);
        auto q = euler_to_quat(original_rpy);
        auto qinv_R = q.conjugate();
        auto qinv_res = euler_to_quat(original_rpy.inverse());

        EXPECT_NEAR(qinv_res.w(), qinv_R.w(), HARD_TOLERANCE);
        EXPECT_NEAR(qinv_res.x(), qinv_R.x(), HARD_TOLERANCE);
        EXPECT_NEAR(qinv_res.y(), qinv_R.y(), HARD_TOLERANCE);
        EXPECT_NEAR(qinv_res.z(), qinv_R.z(), HARD_TOLERANCE);
    }
    // Test XYZ
    {
        EulerAngles<EulerSequence::XYZ, double> original_rpy(ang1, ang2, ang3);
        auto q = euler_to_quat(original_rpy);
        auto qinv_R = q.conjugate();
        auto qinv_res = euler_to_quat(original_rpy.inverse());

        EXPECT_NEAR(qinv_res.w(), qinv_R.w(), HARD_TOLERANCE);
        EXPECT_NEAR(qinv_res.x(), qinv_R.x(), HARD_TOLERANCE);
        EXPECT_NEAR(qinv_res.y(), qinv_R.y(), HARD_TOLERANCE);
        EXPECT_NEAR(qinv_res.z(), qinv_R.z(), HARD_TOLERANCE);
    }
    // Test XZY
    {
        EulerAngles<EulerSequence::XZY, double> original_rpy(ang1, ang2, ang3);
        auto q = euler_to_quat(original_rpy);
        auto qinv_R = q.conjugate();
        auto qinv_res = euler_to_quat(original_rpy.inverse());

        EXPECT_NEAR(qinv_res.w(), qinv_R.w(), HARD_TOLERANCE);
        EXPECT_NEAR(qinv_res.x(), qinv_R.x(), HARD_TOLERANCE);
        EXPECT_NEAR(qinv_res.y(), qinv_R.y(), HARD_TOLERANCE);
        EXPECT_NEAR(qinv_res.z(), qinv_R.z(), HARD_TOLERANCE);
    }
}

TEST_F(RotationTest, EulerAnglesRoundTripZYX) {
    EulerAngles<EulerSequence::ZYX, double> original_rpy(deg2rad(-25.0), deg2rad(40.0),
                                                         deg2rad(35.0));
    auto R = Rotation<ned, frd>(original_rpy);
    auto final_rpy = R.to_euler_angles<EulerSequence::ZYX>();

    EXPECT_NEAR(final_rpy.yaw(), original_rpy.yaw(), HARD_TOLERANCE);
    EXPECT_NEAR(final_rpy.pitch(), original_rpy.pitch(), HARD_TOLERANCE);
    EXPECT_NEAR(final_rpy.roll(), original_rpy.roll(), HARD_TOLERANCE);
}

TEST_F(RotationTest, EulerAnglesRoundTripZXY) {
    EulerAngles<EulerSequence::ZXY, double> original_rpy(deg2rad(-25.0), deg2rad(40.0),
                                                         deg2rad(35.0));
    auto R = Rotation<ned, frd>(original_rpy);
    auto final_rpy = R.to_euler_angles<EulerSequence::ZXY>();

    EXPECT_NEAR(final_rpy.angle_x(), original_rpy.angle_x(), HARD_TOLERANCE);
    EXPECT_NEAR(final_rpy.angle_y(), original_rpy.angle_y(), HARD_TOLERANCE);
    EXPECT_NEAR(final_rpy.angle_z(), original_rpy.angle_z(), HARD_TOLERANCE);
}

TEST_F(RotationTest, EulerAnglesRoundTripXYZ) {
    EulerAngles<EulerSequence::XYZ, double> original_rpy(deg2rad(-25.0), deg2rad(40.0),
                                                         deg2rad(35.0));
    auto R = Rotation<ned, frd>(original_rpy);
    auto final_rpy = R.to_euler_angles<EulerSequence::XYZ>();

    EXPECT_NEAR(final_rpy.angle_x(), original_rpy.angle_x(), HARD_TOLERANCE);
    EXPECT_NEAR(final_rpy.angle_y(), original_rpy.angle_y(), HARD_TOLERANCE);
    EXPECT_NEAR(final_rpy.angle_z(), original_rpy.angle_z(), HARD_TOLERANCE);
}

TEST_F(RotationTest, EulerAnglesXYZToZYX) {
    EulerAngles<EulerSequence::XYZ, double> original_rpy(deg2rad(50.0), deg2rad(-35.0),
                                                         deg2rad(40.0));
    auto R = Rotation<ned, frd>(original_rpy);
    auto final_rpy = R.to_euler_angles<EulerSequence::ZYX>();

    // These values are found using scipy.spatial.transform and the function
    // `rot = Rotation.from_euler('XYZ', ...)` and then `rot.as_euler('ZYX')`
    EXPECT_NEAR(final_rpy.angle_x(), 0.5864349257, HARD_TOLERANCE);
    EXPECT_NEAR(final_rpy.angle_y(), -0.8864542619, HARD_TOLERANCE);
    EXPECT_NEAR(final_rpy.angle_z(), 0.1214496901, HARD_TOLERANCE);
}

TEST_F(RotationTest, EulerAnglesRoundTripXZY) {
    EulerAngles<EulerSequence::XZY, double> original_rpy(deg2rad(-25.0), deg2rad(40.0),
                                                         deg2rad(35.0));
    auto R = Rotation<ned, frd>(original_rpy);
    auto final_rpy = R.to_euler_angles<EulerSequence::XZY>();

    EXPECT_NEAR(final_rpy.angle_x(), original_rpy.angle_x(), HARD_TOLERANCE);
    EXPECT_NEAR(final_rpy.angle_y(), original_rpy.angle_y(), HARD_TOLERANCE);
    EXPECT_NEAR(final_rpy.angle_z(), original_rpy.angle_z(), HARD_TOLERANCE);
}

TEST_F(RotationTest, EulerAnglesRoundTripYXZ) {
    EulerAngles<EulerSequence::YXZ, double> original_rpy(deg2rad(-25.0), deg2rad(40.0),
                                                         deg2rad(35.0));
    auto R = Rotation<ned, frd>(original_rpy);
    auto final_rpy = R.to_euler_angles<EulerSequence::YXZ>();

    EXPECT_NEAR(final_rpy.angle_x(), original_rpy.angle_x(), HARD_TOLERANCE);
    EXPECT_NEAR(final_rpy.angle_y(), original_rpy.angle_y(), HARD_TOLERANCE);
    EXPECT_NEAR(final_rpy.angle_z(), original_rpy.angle_z(), HARD_TOLERANCE);
}

TEST_F(RotationTest, EulerAnglesRoundTripYZX) {
    EulerAngles<EulerSequence::YZX, double> original_rpy(deg2rad(-25.0), deg2rad(40.0),
                                                         deg2rad(35.0));
    auto R = Rotation<ned, frd>(original_rpy);
    auto final_rpy = R.to_euler_angles<EulerSequence::YZX>();

    EXPECT_NEAR(final_rpy.angle_x(), original_rpy.angle_x(), HARD_TOLERANCE);
    EXPECT_NEAR(final_rpy.angle_y(), original_rpy.angle_y(), HARD_TOLERANCE);
    EXPECT_NEAR(final_rpy.angle_z(), original_rpy.angle_z(), HARD_TOLERANCE);
}

TEST_F(RotationTest, InverseIsCorrect) {
    YawPitchRoll<double> rpy(0.1, 0.2, 0.3);
    Rotation<ned, frd> R = Rotation<ned, frd>(rpy);
    Rotation<frd, ned> R_inv = R.inverse();

    // The composition of a rotation and its inverse should be the identity
    Rotation<ned, ned> R_identity = R * R_inv;
    UnitQuaterniond q_identity = R_identity.to_quaternion();

    EXPECT_NEAR(q_identity.w(), 1.0, HARD_TOLERANCE);
    EXPECT_NEAR(q_identity.x(), 0.0, HARD_TOLERANCE);
    EXPECT_NEAR(q_identity.y(), 0.0, HARD_TOLERANCE);
    EXPECT_NEAR(q_identity.z(), 0.0, HARD_TOLERANCE);
}

TEST_F(RotationTest, VectorRotation) {
    // A 90-degree yaw rotation from body (frd) to world (ned)
    auto R_ned_frd = Rotation<ned, frd>(EulerAngles<EulerSequence::ZYX>{deg2rad(90.0), 0, 0});

    // A vector pointing purely forward in the body frame
    Vector3D<frd, double> v_frd(10.0, 0.0, 0.0);

    // Rotate the vector into the world frame
    Vector3D<ned, double> v_ned = R_ned_frd * v_frd;

    // A 90-degree yaw should rotate the forward vector to point East
    EXPECT_NEAR(v_ned.north(), 0.0, HARD_TOLERANCE);
    EXPECT_NEAR(v_ned.east(), 10.0, HARD_TOLERANCE);
    EXPECT_NEAR(v_ned.down(), 0.0, HARD_TOLERANCE);
}

TEST_F(RotationTest, RotationComposition) {
    // R_ab: 90-degree yaw
    auto R_ab = Rotation<ned, frd>(EulerAngles<EulerSequence::ZYX>{deg2rad(90.0), 0, 0});
    // R_bc: 90-degree pitch
    auto R_bc = Rotation<frd, flu>(EulerAngles<EulerSequence::ZYX>{0, deg2rad(90.0), 0});

    // Compose them: R_ac = R_ab * R_bc
    Rotation<ned, flu> R_ac = R_ab * R_bc;

    // A vector in the 'c' (flu) frame
    Vector3D<flu, double> v_c(1.0, 0.0, 0.0);

    // Transform v_c to v_a
    Vector3D<ned, double> v_a = R_ac * v_c;

    // Expected result from manual calculation (Yaw(90) * Pitch(90) * [1,0,0]')
    EXPECT_NEAR(v_a.north(), 0.0, HARD_TOLERANCE);
    EXPECT_NEAR(v_a.east(), 0.0, HARD_TOLERANCE);
    EXPECT_NEAR(v_a.down(), -1.0, HARD_TOLERANCE);
}

/// @brief A function to calculate the ECEF to NED rotation.
/// @note It returns a fully type-safe object.
/// @note The following coputations are taken by Fig 3.1, page 50, and Eq. (3.12), page 57, Book:
/// "Applied Mathematics in Integrated Navigation Systems" 3rd Edition, Robert M. Rogers
refx::Rotation<refx::ned, refx::ecef> create_R_ned_from_ecef(double lat_rad, double lon_rad) {
    // --- Step 1: Frame-Agnostic Computation (The "Scratchpad") ---
    // We perform all intermediate calculations using the raw `UnitQuaternion` type.
    // These objects have no concept of "To" or "From" frames.

    // Rotation about the Y-axis by pi/2 (NED->Intermediate F1)
    UnitQuaternion<double> q_pi_2 = UnitQuaternion<double>::from_rotation_y(M_PI_2);
    // Rotation about the Y-axis by latitude (Intermediate F1 -> Intermediate F2)
    UnitQuaternion<double> q_lat = UnitQuaternion<double>::from_rotation_y(lat_rad);
    // Rotation about the Z-axis by -longitude (Intermediate F2 -> ECEF)
    UnitQuaternion<double> q_lon = UnitQuaternion<double>::from_rotation_z(-lon_rad);
    // Compose the rotations. This is a pure mathematical operation.
    UnitQuaternion<double> final_q = q_pi_2 * q_lat * q_lon;

    // --- Step 2: Assign Geometric Meaning (The "Final Report") ---
    // Now that our calculation is complete, we construct the final, type-safe
    // Rotation object from our result.
    return refx::Rotation<refx::ned, refx::ecef>(final_q);
}

TEST_F(RotationTest, RotationCompositionECEF_NED) {
    Rotation<ned, ecef> R_ned_from_ecef = create_R_ned_from_ecef(deg2rad(45.0), deg2rad(-93.0));
    UnitQuaternion<double> q_ned_from_ecef = R_ned_from_ecef.to_quaternion();

    // manual calculation made by finding the quaternion values of the known ECEF->NED matrix
    EXPECT_NEAR(q_ned_from_ecef.w(), 0.2634218917, HARD_TOLERANCE);
    EXPECT_NEAR(q_ned_from_ecef.x(), 0.6701585348, HARD_TOLERANCE);
    EXPECT_NEAR(q_ned_from_ecef.y(), 0.6359567036, HARD_TOLERANCE);
    EXPECT_NEAR(q_ned_from_ecef.z(), 0.277588754, HARD_TOLERANCE);
}

// =============================================================================
// == Test Suite for Eigen Interoperability (Conditionally Compiled)
// =============================================================================
#ifdef REFX_ENABLE_EIGEN_SUPPORT

class EigenRotationInteroperability : public ::testing::Test {};

TEST_F(EigenRotationInteroperability, EigenQuaternionToRefxRotation) {
    // 1. ARRANGE: Create an Eigen quaternion for a 90-degree yaw.
    Eigen::AngleAxisd angle_axis(M_PI / 2.0, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond eigen_quat(angle_axis);

    // 2. ACT: Construct a refx::Rotation from it.
    Rotation<ned, frd> refx_rot(eigen_quat);
    UnitQuaternion q_out = refx_rot.to_quaternion();

    // 3. ASSERT: Check that the internal quaternion has the correct values.
    EXPECT_NEAR(q_out.w(), std::cos(M_PI / 4.0), HARD_TOLERANCE);
    EXPECT_NEAR(q_out.x(), 0.0, HARD_TOLERANCE);
    EXPECT_NEAR(q_out.y(), 0.0, HARD_TOLERANCE);
    EXPECT_NEAR(q_out.z(), std::sin(M_PI / 4.0), HARD_TOLERANCE);
}

TEST_F(EigenRotationInteroperability, EigenMatrixToRefxRotation) {
    // 1. ARRANGE: Create an Eigen rotation matrix for a 90-degree yaw.
    Eigen::Matrix3d eigen_matrix;
    eigen_matrix = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ());

    // 2. ACT: Construct a refx::Rotation from it.
    Rotation<ned, frd> refx_rot(eigen_matrix);
    UnitQuaternion q_out = refx_rot.to_quaternion();

    // 3. ASSERT: Check that the internal quaternion has the correct values.
    EXPECT_NEAR(q_out.w(), std::cos(M_PI / 4.0), HARD_TOLERANCE);
    EXPECT_NEAR(q_out.x(), 0.0, HARD_TOLERANCE);
    EXPECT_NEAR(q_out.y(), 0.0, HARD_TOLERANCE);
    EXPECT_NEAR(q_out.z(), std::sin(M_PI / 4.0), HARD_TOLERANCE);
}

TEST_F(EigenRotationInteroperability, RefxRotationToEigenQuaternion) {
    // 1. ARRANGE: Create a refx rotation for a 90-degree yaw.
    auto refx_rot = Rotation<ned, frd>(YawPitchRoll<double>(deg2rad(90.0), 0, 0));

    // 2. ACT: Convert it to an Eigen quaternion.
    Eigen::Quaterniond eigen_quat = refx_rot.to_eigen_quat();

    // 3. ASSERT: Check the Eigen quaternion's components.
    EXPECT_NEAR(eigen_quat.w(), std::cos(M_PI / 4.0), HARD_TOLERANCE);
    EXPECT_NEAR(eigen_quat.x(), 0.0, HARD_TOLERANCE);
    EXPECT_NEAR(eigen_quat.y(), 0.0, HARD_TOLERANCE);
    EXPECT_NEAR(eigen_quat.z(), std::sin(M_PI / 4.0), HARD_TOLERANCE);
}

TEST_F(EigenRotationInteroperability, RefxRotationToEigenMatrix) {
    // 1. ARRANGE: Create a refx rotation for a 90-degree yaw.
    auto refx_rot = Rotation<ned, frd>(YawPitchRoll<double>(deg2rad(90.0), 0, 0));

    // 2. ACT: Convert it to an Eigen matrix.
    Eigen::Matrix3d eigen_matrix = refx_rot.to_eigen_matrix();

    // 3. ASSERT: Check the matrix elements.
    EXPECT_NEAR(eigen_matrix(0, 0), 0.0, HARD_TOLERANCE);
    EXPECT_NEAR(eigen_matrix(0, 1), -1.0, HARD_TOLERANCE);
    EXPECT_NEAR(eigen_matrix(1, 0), 1.0, HARD_TOLERANCE);
    EXPECT_NEAR(eigen_matrix(1, 1), 0.0, HARD_TOLERANCE);
    EXPECT_NEAR(eigen_matrix(2, 2), 1.0, HARD_TOLERANCE);
}

TEST_F(EigenRotationInteroperability, EigenQuaternionRoundTrip) {
    // 1. ARRANGE: Create an Eigen quaternion.
    Eigen::AngleAxisd angle_axis(deg2rad(35.0), Eigen::Vector3d(1, 2, 3).normalized());
    Eigen::Quaterniond eigen_quat_in(angle_axis);

    // 2. ACT: Convert to refx and back to Eigen.
    Rotation<ned, frd> refx_rot(eigen_quat_in);
    Eigen::Quaterniond eigen_quat_out = refx_rot.to_eigen_quat();

    // 3. ASSERT: Check that the quaternions are almost identical.
    // isApprox() is the preferred way to compare Eigen types.
    ASSERT_TRUE(eigen_quat_in.isApprox(eigen_quat_out, HARD_TOLERANCE));
}

#endif  // REFX_ENABLE_EIGEN_SUPPORT

}  // namespace testing
}  // namespace refx

#endif /* _GEOMETRY_TEST_ROTATION_ */
