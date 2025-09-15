#ifndef _GEOMETRY_TEST_TRANSFORMATION_
#define _GEOMETRY_TEST_TRANSFORMATION_

#include "../tol.h"
#include "gtest/gtest.h"
#include "refx/geometry.h"
#include "refx/math.h"
#include "refx/transformations.h"

// Use a consistent namespace for tests
namespace refx {
namespace testing {

// Define a test fixture for Transformation tests.
// This allows us to set up common objects for all tests in this suite.
class TransformationTest : public ::testing::Test {
   protected:
    // We can define common types and objects here.
    using TransformNED_FRD = refx::Transformation<refx::ned, refx::frd>;
    using RotationNED_FRD = refx::Rotation<refx::ned, refx::frd>;
    using VectorNED = refx::Vector3D<refx::ned>;

    // A known rotation for testing (90-degree yaw around the Z-axis)
    RotationNED_FRD test_rotation = RotationNED_FRD(YawPitchRoll<double>(deg2rad(90.0), 0, 0));

    // A known translation for testing
    VectorNED test_translation{10.0, 20.0, 30.0};
};

/**
 * @test FrameSafety (Compile-Time Test)
 * @brief Demonstrates the compile-time frame safety of the Transformation struct.
 *
 * This is a conceptual test. The code inside is commented out because it
 * SHOULD NOT COMPILE. If it were to compile, it would indicate a failure of the
 * library's core type-safety feature.
 */
TEST_F(TransformationTest, FrameSafety) {
    // The following code should produce a compile-time error.
    // We are trying to assign a transform between <ned, frd> to a variable
    // expecting a transform between <enu, frd>. The refx library's
    // type system correctly prevents this dangerous operation.
    /*
    refx::Transformation<refx::ned, refx::frd> T_ned_frd;
    refx::Transformation<refx::enu, refx::frd> T_enu_frd;

    // COMPILE ERROR: cannot convert
    // 'refx::Transformation<refx::ned, ...>' to
    // 'refx::Transformation<refx::enu, ...>'
    T_enu_frd = T_ned_frd;
    */

    // The "success" of this test is that the code above fails to build.
    SUCCEED();
}

/**
 * @test CompositionOfTransformations
 * @brief Verifies the correctness of chaining (composing) two transformations.
 *
 * This is a critical corner case. It checks if the mathematical formula for
 * combining two SE(3) transforms is implemented correctly. We transform from a
 * camera frame to a vehicle frame, and then from the vehicle frame to the world.
 */
TEST_F(TransformationTest, CompositionOfTransformations) {
    // 1. Define the transform from vehicle to world (ned)
    auto T_ned_vehicle = TransformNED_FRD{test_rotation, test_translation};

    // 2. Define the transform from camera to vehicle (frd)
    // The camera is mounted 2m forward, 0.5m right, and pointing 45 degrees down (pitch).
    auto R_vehicle_camera = refx::Rotation<refx::frd, refx::flu>(
        refx::YawPitchRoll<double>(0.0, refx::deg2rad(-45.0), 0.0));
    auto t_vehicle_camera = refx::Vector3D<refx::frd>{2.0, 0.5, 0.0};
    auto T_vehicle_camera =
        refx::Transformation<refx::frd, refx::flu>{R_vehicle_camera, t_vehicle_camera};

    // 3. Compose the transformations to get the transform from camera to world
    auto T_ned_camera = T_ned_vehicle * T_vehicle_camera;

    // 4. Manually calculate the expected result
    auto expected_rotation = T_ned_vehicle.rotation * T_vehicle_camera.rotation;
    auto expected_translation =
        T_ned_vehicle.rotation * T_vehicle_camera.translation + T_ned_vehicle.translation;

    // 5. Assert that the results are nearly equal
    // We use EXPECT_NEAR for floating point comparisons.
    refx::UnitQuaternion<double> expected_quat = expected_rotation.to_quaternion();
    refx::UnitQuaternion<double> ned_camera_quat = T_ned_camera.rotation.to_quaternion();
    EXPECT_NEAR(ned_camera_quat.w(), expected_quat.w(), HARD_TOLERANCE);
    EXPECT_NEAR(ned_camera_quat.x(), expected_quat.x(), HARD_TOLERANCE);
    EXPECT_NEAR(ned_camera_quat.y(), expected_quat.y(), HARD_TOLERANCE);
    EXPECT_NEAR(ned_camera_quat.z(), expected_quat.z(), HARD_TOLERANCE);

    EXPECT_NEAR(T_ned_camera.translation.x(), expected_translation.x(), HARD_TOLERANCE);
    EXPECT_NEAR(T_ned_camera.translation.y(), expected_translation.y(), HARD_TOLERANCE);
    EXPECT_NEAR(T_ned_camera.translation.z(), expected_translation.z(), HARD_TOLERANCE);
}

/**
 * @test InverseTransformation
 * @brief Verifies the correctness of the .inverse() method.
 *
 * This test ensures that inverting a transformation T_A_B correctly yields
 * T_B_A. It checks both the rotational and translational components against
 * the known mathematical formula for the inverse of an SE(3) transformation.
 */
TEST_F(TransformationTest, InverseTransformation) {
    // 1. Arrange: Create an initial transformation from vehicle (frd) to world (ned).
    auto T_ned_frd = TransformNED_FRD{test_rotation, test_translation};

    // 2. Act: Compute the inverse of the transformation.
    auto T_frd_ned = T_ned_frd.inverse();

    // 3. Assert: Manually calculate the expected inverse and compare.
    // The expected inverse rotation is the inverse of the original rotation.
    auto expected_inv_rotation = test_rotation.inverse();
    refx::UnitQuaternion<double> expected_quat = expected_inv_rotation.to_quaternion();

    // The expected inverse translation is -(R_inv * t).
    auto expected_inv_translation = -1.0 * (expected_inv_rotation * test_translation);
    refx::UnitQuaternion<double> frd_ned_quat = T_frd_ned.rotation.to_quaternion();

    // Assert that the computed inverse rotation is nearly equal to the expected one.
    EXPECT_NEAR(frd_ned_quat.w(), expected_quat.w(), HARD_TOLERANCE);
    EXPECT_NEAR(frd_ned_quat.x(), expected_quat.x(), HARD_TOLERANCE);
    EXPECT_NEAR(frd_ned_quat.y(), expected_quat.y(), HARD_TOLERANCE);
    EXPECT_NEAR(frd_ned_quat.z(), expected_quat.z(), HARD_TOLERANCE);

    // Assert that the computed inverse translation is nearly equal to the expected one.
    EXPECT_NEAR(T_frd_ned.translation.x(), expected_inv_translation.x(), HARD_TOLERANCE);
    EXPECT_NEAR(T_frd_ned.translation.y(), expected_inv_translation.y(), HARD_TOLERANCE);
    EXPECT_NEAR(T_frd_ned.translation.z(), expected_inv_translation.z(), HARD_TOLERANCE);
}

/**
 * @test TramsformationCoordinateProduct
 * @brief Verifies the correctness of the operator* method, when rhs is a Coordinate<>
 */
TEST_F(TransformationTest, TramsformationCoordinateProduct) {
    // 1. Arrange: Create an initial transformation from vehicle (frd) to world (ned).
    auto T_ned_frd = TransformNED_FRD{test_rotation, test_translation};

    // 2. Generate a Coordinate3D of the coherent frame type (frd).
    Coordinate3D<frd> p_frd(1, 2, 3);

    // 3. Test multiplication legit and results
    Coordinate3D<ned> p_ned = T_ned_frd * p_frd;
    // The same result is obtained via:
    // Coordinate3D<ned> p_ned = frame_transform(T_ned_frd, p_frd);

    // These are compile-time errors
    // Coordinate3D<frd> p_ned = T_ned_frd * v_frd; //output must be <ned>
    // Coordinate3D<ned> p_ned = T_ned_frd.inverse() * v_frd; //inverse() maps ned to frd
    // Coordinate3D<ned> p_ned = T_ned_frd * Vector3D<frd>(); //rhs must be a Coordinate3D

    // This is ok: p_ned is a 'Vector3D'
    // auto p_ned = T_ned_frd * Vector3D<frd>();

    // Assert that the computed inverse translation is nearly equal to the expected one.
    EXPECT_NEAR(p_ned.north(), 8.0, HARD_TOLERANCE);
    EXPECT_NEAR(p_ned.east(), 21.0, HARD_TOLERANCE);
    EXPECT_NEAR(p_ned.down(), 33.0, HARD_TOLERANCE);
}

/**
 * @test TramsformationCoordinateProduct
 * @brief Verifies the correctness of the operator* method, when rhs is a Vector<>
 */
TEST_F(TransformationTest, TramsformationVectorProduct) {
    // 1. Arrange: Create an initial transformation from vehicle (frd) to world (ned).
    auto T_ned_frd = TransformNED_FRD{test_rotation, test_translation};

    // 2. Generate a Coordinate3D of the coherent frame type (frd).
    Vector3D<frd> v_frd(1, 2, 3);

    // 3. Test multiplication legit and results
    Vector3D<ned> v_ned = T_ned_frd * v_frd;
    // The same result is obtained via:
    // Vector3D<ned> v_ned = frame_transform(T_ned_frd, v_frd);

    // These are compile-time errors
    // Vector3D<frd> v_ned = T_ned_frd * v_frd;
    // Vector3D<ned> v_ned = T_ned_frd.inverse() * v_frd;

    // Assert that the computed inverse translation is nearly equal to the expected one.
    EXPECT_NEAR(v_ned.north(), 8.0, HARD_TOLERANCE);
    EXPECT_NEAR(v_ned.east(), 21.0, HARD_TOLERANCE);
    EXPECT_NEAR(v_ned.down(), 33.0, HARD_TOLERANCE);

    // This is possible: exploits object slicing (Coordinate3D is derived from Vector3D)
    // TODO: decide about this to be lecit or not.
    // v_ned = T_ned_frd * Coordinate3D<frd>();
}

}  // namespace testing
}  // namespace refx
#endif /* _GEOMETRY_TEST_TRANSFORMATION_ */
