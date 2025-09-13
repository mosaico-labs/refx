#ifndef _GEOMETRY_TEST_VECTOR_COORDINATE_
#define _GEOMETRY_TEST_VECTOR_COORDINATE_
#include <gtest/gtest.h>

// Include all the necessary headers from your library
#include "../tol.h"
#include "refx/geometry.h"

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
 * @test Test fixture for Vector3D and Coordinate3D classes.
 * This suite verifies the core mathematical semantics and frame-specific logic.
 */
class VectorCoordinateTest : public ::testing::Test {
   protected:
    // Define some common objects to be used in the tests
    Coordinate3D<ned, double> p1_ned{10.0, 20.0, -5.0};
    Coordinate3D<ned, double> p2_ned{12.0, 25.0, -4.0};
    Vector3D<ned, double> v_ned{2.0, 5.0, 1.0};
};

// Test the fundamental mathematical relationship: Coordinate - Coordinate = Vector
TEST_F(VectorCoordinateTest, CoordinateSubtractionYieldsVector) {
    Vector3D<ned, double> displacement = p2_ned - p1_ned;

    EXPECT_NEAR(displacement.north(), 2.0, HARD_TOLERANCE);
    EXPECT_NEAR(displacement.east(), 5.0, HARD_TOLERANCE);
    EXPECT_NEAR(displacement.down(), 1.0, HARD_TOLERANCE);
}

// Test the fundamental mathematical relationship: Coordinate + Vector = Coordinate
TEST_F(VectorCoordinateTest, CoordinatePlusVectorYieldsCoordinate) {
    Coordinate3D<ned, double> result_pos = p1_ned + v_ned;

    EXPECT_NEAR(result_pos.north(), 12.0, HARD_TOLERANCE);
    EXPECT_NEAR(result_pos.east(), 25.0, HARD_TOLERANCE);
    EXPECT_NEAR(result_pos.down(), -4.0, HARD_TOLERANCE);
}

// Test the fundamental mathematical relationship: Vector + Vector = Vector
TEST_F(VectorCoordinateTest, VectorAdditionYieldsVector) {
    Vector3D<ned, double> v2_ned{1.0, -1.0, 1.0};
    Vector3D<ned, double> result_vec = v_ned + v2_ned;

    EXPECT_NEAR(result_vec.north(), 3.0, HARD_TOLERANCE);
    EXPECT_NEAR(result_vec.east(), 4.0, HARD_TOLERANCE);
    EXPECT_NEAR(result_vec.down(), 2.0, HARD_TOLERANCE);
}

// This test demonstrates that adding two coordinates is correctly disallowed.
// Uncommenting the line below should cause a compile-time error.
// TEST_F(VectorCoordinateTest, CoordinateAdditionFailsToCompile) { auto result = p1_ned + p2_ned; }

// Test the specialized angular subtraction for longitude (LLA frame)
TEST_F(VectorCoordinateTest, LongitudeSubtractionWrapsCorrectly) {
    // Two points on opposite sides of the antimeridian
    Coordinate3D<lla, double> p1_lla(89.0, 179.0, 0.0);
    Coordinate3D<lla, double> p2_lla(-89.0, -179.0, 0.0);

    // The displacement should be 2 degrees eastward, not -358 degrees.
    Vector3D<lla, double> displacement = p2_lla - p1_lla;

    EXPECT_NEAR(displacement.delta_latitude(), 2.0, HARD_TOLERANCE);
    EXPECT_NEAR(displacement.delta_longitude(), 2.0,
                HARD_TOLERANCE);  // 179 -> -179 is a 2 deg eastward change
    EXPECT_NEAR(displacement.delta_altitude(), 0.0, HARD_TOLERANCE);
}

// Test the specialized angular subtraction for longitude (LLA frame)
TEST_F(VectorCoordinateTest, VectorCoordinateECEF) {
    // Two points on opposite sides of the antimeridian
    Coordinate3D<ecef, double> p1_ecef(0, 0, 6356752.31);
    Coordinate3D<ecef, double> p2_ecef(4441151.14, 827930.16, 4487421.95);

    // The displacement should be 2 degrees eastward, not -358 degrees.
    Vector3D<ecef, double> displacement = p2_ecef - p1_ecef;

    EXPECT_NEAR(displacement.x(), 4441151.14, HARD_TOLERANCE);
    EXPECT_NEAR(displacement.y(), 827930.16, HARD_TOLERANCE);
    EXPECT_NEAR(displacement.z(), -1869330.36, HARD_TOLERANCE);

    Coordinate3D<ecef, double> p2_ecef_again = p1_ecef + displacement;

    EXPECT_NEAR(p2_ecef_again.x(), p2_ecef.x(), HARD_TOLERANCE);
    EXPECT_NEAR(p2_ecef_again.y(), p2_ecef.y(), HARD_TOLERANCE);
    EXPECT_NEAR(p2_ecef_again.z(), p2_ecef.z(), HARD_TOLERANCE);
}

// Test the specialized angular subtraction for longitude (LLA frame)
TEST_F(VectorCoordinateTest, VectorCoordinateIMU) {
    // Two points on opposite sides of the antimeridian
    Coordinate3D<imu, double> p1_imu(0.5, 0.7, -9.81);
    Coordinate3D<imu, double> p2_imu(0.3, -1.2, 0.89);

    // The displacement should be 2 degrees eastward, not -358 degrees.
    Vector3D<imu, double> displacement = p2_imu - p1_imu;

    EXPECT_NEAR(displacement.x(), -0.2, HARD_TOLERANCE);
    EXPECT_NEAR(displacement.y(), -1.9, HARD_TOLERANCE);
    EXPECT_NEAR(displacement.z(), 10.7, HARD_TOLERANCE);

    Coordinate3D<imu, double> p2_imu_again = p1_imu + displacement;

    EXPECT_NEAR(p2_imu_again.x(), p2_imu.x(), HARD_TOLERANCE);
    EXPECT_NEAR(p2_imu_again.y(), p2_imu.y(), HARD_TOLERANCE);
    EXPECT_NEAR(p2_imu_again.z(), p2_imu.z(), HARD_TOLERANCE);
}

// Test the specialized angular subtraction for longitude (LLA frame)
TEST_F(VectorCoordinateTest, VectorCoordinateCAM) {
    // Two points on opposite sides of the antimeridian
    Coordinate3D<camera, double> p1_cam(0.5, 0.7, -9.81);
    Coordinate3D<camera, double> p2_cam(0.3, -1.2, 0.89);

    // The displacement should be 2 degrees eastward, not -358 degrees.
    Vector3D<camera, double> displacement = p2_cam - p1_cam;

    EXPECT_NEAR(displacement.x(), -0.2, HARD_TOLERANCE);
    EXPECT_NEAR(displacement.y(), -1.9, HARD_TOLERANCE);
    EXPECT_NEAR(displacement.z(), 10.7, HARD_TOLERANCE);

    Coordinate3D<camera, double> p2_cam_again = p1_cam + displacement;

    EXPECT_NEAR(p2_cam_again.x(), p2_cam.x(), HARD_TOLERANCE);
    EXPECT_NEAR(p2_cam_again.y(), p2_cam.y(), HARD_TOLERANCE);
    EXPECT_NEAR(p2_cam_again.z(), p2_cam.z(), HARD_TOLERANCE);
}

// Test the specialized angular subtraction for longitude (LLA frame)
TEST_F(VectorCoordinateTest, VectorCoordinateWA) {
    // Two points on opposite sides of the antimeridian
    Coordinate3D<wa, double> p1_wa(0.5, 0.7, -9.81);
    Coordinate3D<wa, double> p2_wa(0.3, -1.2, 0.89);

    // The displacement should be 2 degrees eastward, not -358 degrees.
    Vector3D<wa, double> displacement = p2_wa - p1_wa;

    EXPECT_NEAR(displacement.x(), -0.2, HARD_TOLERANCE);
    EXPECT_NEAR(displacement.y(), -1.9, HARD_TOLERANCE);
    EXPECT_NEAR(displacement.z(), 10.7, HARD_TOLERANCE);

    Coordinate3D<wa, double> p2_wa_again = p1_wa + displacement;

    EXPECT_NEAR(p2_wa_again.x(), p2_wa.x(), HARD_TOLERANCE);
    EXPECT_NEAR(p2_wa_again.y(), p2_wa.y(), HARD_TOLERANCE);
    EXPECT_NEAR(p2_wa_again.z(), p2_wa.z(), HARD_TOLERANCE);
}
// =============================================================================
// == Test Suite for Eigen Interoperability (Conditionally Compiled)
// =============================================================================
#ifdef REFX_ENABLE_EIGEN_SUPPORT

/**
 * @test EigenInteroperability.Vector3DToEigen
 * @brief Verifies the conversion from a refx::Vector3D to an Eigen::Vector3d.
 * @details This test ensures that the .to_eigen() method correctly maps the
 * internal std::array data to a new Eigen::Vector3d with identical components.
 */
TEST(EigenInteroperability, Vector3DToEigen) {
    // 1. ARRANGE: Create a refx vector with known values.
    const Vector3D<ned, double> refx_vec(1.2, -3.4, 5.6);

    // 2. ACT: Convert it to an Eigen vector.
    Eigen::Vector3d eigen_vec = refx_vec.to_eigen();

    // 3. ASSERT: Check that each component matches.
    EXPECT_DOUBLE_EQ(eigen_vec.x(), 1.2);
    EXPECT_DOUBLE_EQ(eigen_vec.y(), -3.4);
    EXPECT_DOUBLE_EQ(eigen_vec.z(), 5.6);
}

/**
 * @test EigenInteroperability.Coordinate3DToEigen
 * @brief Verifies the conversion from a refx::Coordinate3D to an Eigen::Vector3d.
 * @details This test ensures that the .to_eigen() method correctly maps the
 * internal std::array data to a new Eigen::Vector3d with identical components.
 */
TEST(EigenInteroperability, Coordinate3DToEigen) {
    {
        // 1. ARRANGE: Create a refx coordinate with known values.
        const Coordinate3D<ned, double> refx_vec(-0.98, 0.35, -190.57);
        // 2. ACT: Convert it to an Eigen vector.
        Eigen::Vector3d eigen_vec = refx_vec.to_eigen();

        // 3. ASSERT: Check that each component matches.
        EXPECT_DOUBLE_EQ(eigen_vec.x(), -0.98);
        EXPECT_DOUBLE_EQ(eigen_vec.y(), 0.35);
        EXPECT_DOUBLE_EQ(eigen_vec.z(), -190.57);
    }
    {
        // 1. ARRANGE: Create a refx coordinate with known values.
        const Coordinate3D<lla, double> refx_vec(-0.98, 0.35, -190.57);
        // 2. ACT: Convert it to an Eigen vector.
        Eigen::Vector3d eigen_vec = refx_vec.to_eigen();

        // 3. ASSERT: Check that each component matches.
        EXPECT_DOUBLE_EQ(eigen_vec.x(), -0.98);
        EXPECT_DOUBLE_EQ(eigen_vec.y(), 0.35);
        EXPECT_DOUBLE_EQ(eigen_vec.z(), -190.57);
    }
}

/**
 * @test EigenInteroperability.EigenToVector3D
 * @brief Verifies the construction of a refx::Vector3D from an Eigen::Vector3d.
 * @details This test ensures that the specialized constructor correctly copies the
 * data from an Eigen vector into a new refx::Vector3D object.
 */
TEST(EigenInteroperability, EigenToVector3D) {
    // 1. ARRANGE: Create an Eigen vector with known values.
    Eigen::Vector3d eigen_vec;
    eigen_vec << -7.8, 9.1, -2.3;

    // 2. ACT: Construct a refx vector from the Eigen vector.
    const Vector3D<ned, double> refx_vec(eigen_vec);

    // 3. ASSERT: Check that each component of the new refx vector matches.
    EXPECT_DOUBLE_EQ(refx_vec.north(), -7.8);
    EXPECT_DOUBLE_EQ(refx_vec.east(), 9.1);
    EXPECT_DOUBLE_EQ(refx_vec.down(), -2.3);
}

/**
 * @test EigenInteroperability.EigenToCoordinate3D
 * @brief Verifies the construction of a refx::Coordinate3D from an Eigen::Vector3d.
 * @details This test ensures that the specialized constructor correctly copies the
 * data from an Eigen vector into a new refx::Coordinate3D object.
 */
TEST(EigenInteroperability, EigenToCoordinate3D) {
    // 1. ARRANGE: Create an Eigen vector with known values.
    Eigen::Vector3d eigen_vec;
    eigen_vec << -0.98, 0.35, -190.57;

    {
        // 2. ACT: Construct a refx vector from the Eigen vector.
        const Coordinate3D<enu, double> refx_coord(eigen_vec);

        // 3. ASSERT: Check that each component of the new refx vector matches.
        EXPECT_DOUBLE_EQ(refx_coord.east(), -0.98);
        EXPECT_DOUBLE_EQ(refx_coord.north(), 0.35);
        EXPECT_DOUBLE_EQ(refx_coord.up(), -190.57);
    }
    {
        // 2. ACT: Construct a refx vector from the Eigen vector.
        const Coordinate3D<lla, double> refx_coord(eigen_vec);

        // 3. ASSERT: Check that each component of the new refx vector matches.
        EXPECT_DOUBLE_EQ(refx_coord.latitude(), -0.98);
        EXPECT_DOUBLE_EQ(refx_coord.longitude(), 0.35);
        EXPECT_DOUBLE_EQ(refx_coord.altitude(), -190.57);
    }
}

#endif /* REFX_ENABLE_EIGEN_SUPPORT */
}  // namespace testing
}  // namespace refx

#endif /* _GEOMETRY_TEST_VECTOR_COORDINATE_ */
