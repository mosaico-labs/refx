#ifndef _TRANSFORMATIONS_TEST_TRANSFORMATIONS_
#define _TRANSFORMATIONS_TEST_TRANSFORMATIONS_

#include <gtest/gtest.h>

#include "../tol.h"
#include "refx/transformations.h"

// Use a consistent namespace for tests
namespace refx {
namespace testing {
//
// ---------------------- Frame Transformations Testing ----------------------
//

// A test fixture to hold common data for all tests
class FrameCastTest : public ::testing::Test {
   protected:
    // A generic vector used for most tests
    Vector3D<ned, double> v_ned{10.0, 20.0, 30.0};  // {N, E, D}
    Vector3D<frd, double> v_frd{10.0, 20.0, 30.0};  // {F, R, D}
    Vector3D<imu, double> v_imu{10.0, 20.0, 30.0};  // {IMU.X, IMU.Y, IMU.Z}
};

// --- General Tests ---

TEST_F(FrameCastTest, IdentityConversion) {
    auto v_ned_again = frame_cast<ned>(v_ned);
    EXPECT_NEAR(v_ned_again.north(), 10.0, HARD_TOLERANCE);
    EXPECT_NEAR(v_ned_again.east(), 20.0, HARD_TOLERANCE);
    EXPECT_NEAR(v_ned_again.down(), 30.0, HARD_TOLERANCE);
}

// --- Local Tangent Frame Conversion Tests ---

TEST_F(FrameCastTest, LocalFramesDirect) {
    // NED -> ENU
    auto v_enu = frame_cast<enu>(v_ned);
    EXPECT_NEAR(v_enu.east(), 20.0, HARD_TOLERANCE);   // E = E
    EXPECT_NEAR(v_enu.north(), 10.0, HARD_TOLERANCE);  // N = N
    EXPECT_NEAR(v_enu.up(), -30.0, HARD_TOLERANCE);    // U = -D

    // ENU -> NED (Round Trip)
    auto v_ned_again = frame_cast<ned>(v_enu);
    EXPECT_NEAR(v_ned_again.x(), v_ned.x(), HARD_TOLERANCE);
    EXPECT_NEAR(v_ned_again.y(), v_ned.y(), HARD_TOLERANCE);
    EXPECT_NEAR(v_ned_again.z(), v_ned.z(), HARD_TOLERANCE);
}

TEST_F(FrameCastTest, LocalFramesChained) {
    // This is a key test: ENU -> NWU must pass through the canonical NED frame.
    // ENU -> NED -> NWU
    Vector3D<enu, double> v_enu(10.0, 20.0, 30.0);  // {E, N, U}

    auto v_nwu = frame_cast<nwu>(v_enu);

    // Manual Calculation:
    // 1. ENU(10, 20, 30) -> NED(20, 10, -30)
    // 2. NED(20, 10, -30) -> NWU(20, -10, 30)
    EXPECT_NEAR(v_nwu.north(), 20.0, HARD_TOLERANCE);  // N = N
    EXPECT_NEAR(v_nwu.west(), -10.0, HARD_TOLERANCE);  // W = -E
    EXPECT_NEAR(v_nwu.up(), 30.0, HARD_TOLERANCE);     // U = U

    // NWU -> ENU (Round Trip)
    auto v_enu_again = frame_cast<enu>(v_nwu);
    EXPECT_NEAR(v_enu_again.x(), v_enu.x(), HARD_TOLERANCE);
    EXPECT_NEAR(v_enu_again.y(), v_enu.y(), HARD_TOLERANCE);
    EXPECT_NEAR(v_enu_again.z(), v_enu.z(), HARD_TOLERANCE);
}

TEST_F(FrameCastTest, SphericalToCartesian) {
    // Test AER to NED conversion (Coordinate only)
    Coordinate3D<aer, double> c_aer = Coordinate3D<aer, double>(45.0, 30.0, 100.0);

    auto c_ned = frame_transform<ned>(c_aer);

    // Manually calculated expected values
    EXPECT_NEAR(c_ned.north(), 61.2372435695, HARD_TOLERANCE);
    EXPECT_NEAR(c_ned.east(), 61.2372435695, HARD_TOLERANCE);
    EXPECT_NEAR(c_ned.down(), -50.0, HARD_TOLERANCE);

    // test with a different target frame
    auto c_nwu = frame_transform<nwu>(c_aer);

    // This is an error!
    // auto c_frd = frame_transform<frd>(c_aer);

    // Manually calculated expected values
    EXPECT_NEAR(c_nwu.north(), 61.2372435695, HARD_TOLERANCE);
    EXPECT_NEAR(c_nwu.west(), -61.2372435695, HARD_TOLERANCE);
    EXPECT_NEAR(c_nwu.up(), 50.0, HARD_TOLERANCE);

    {  // Round Trip
        auto c_aer_again = frame_transform<aer>(c_nwu);
        EXPECT_NEAR(c_aer_again.azimuth(), 45.0, HARD_TOLERANCE);
        EXPECT_NEAR(c_aer_again.elevation(), 30.0, HARD_TOLERANCE);
        EXPECT_NEAR(c_aer_again.range(), 100.0, HARD_TOLERANCE);
    }

    {  // Round Trip
        auto c_aer_again = frame_transform<aer>(c_ned);
        EXPECT_NEAR(c_aer_again.azimuth(), 45.0, HARD_TOLERANCE);
        EXPECT_NEAR(c_aer_again.elevation(), 30.0, HARD_TOLERANCE);
        EXPECT_NEAR(c_aer_again.range(), 100.0, HARD_TOLERANCE);
    }
}

// --- Body-Fixed Frame Conversion Tests ---

TEST_F(FrameCastTest, BodyFramesDirect) {
    // FRD -> FLU
    auto v_flu = frame_cast<flu>(v_frd);
    EXPECT_NEAR(v_flu.forward(), 10.0, HARD_TOLERANCE);  // F = F
    EXPECT_NEAR(v_flu.left(), -20.0, HARD_TOLERANCE);    // L = -R
    EXPECT_NEAR(v_flu.up(), -30.0, HARD_TOLERANCE);      // U = -D

    auto v_rfu = frame_cast<rfu>(v_frd);
    EXPECT_NEAR(v_rfu.forward(), 10.0, HARD_TOLERANCE);  // F = F
    EXPECT_NEAR(v_rfu.right(), 20.0, HARD_TOLERANCE);    // L = -R
    EXPECT_NEAR(v_rfu.up(), -30.0, HARD_TOLERANCE);      // U = -D

    {  // FLU -> FRD (Round Trip)
        auto v_frd_again = frame_cast<frd>(v_flu);
        EXPECT_NEAR(v_frd_again.x(), v_frd.x(), HARD_TOLERANCE);
        EXPECT_NEAR(v_frd_again.y(), v_frd.y(), HARD_TOLERANCE);
        EXPECT_NEAR(v_frd_again.z(), v_frd.z(), HARD_TOLERANCE);
    }
    {  // RFU -> FRD (Round Trip)
        auto v_frd_again = frame_cast<frd>(v_rfu);
        EXPECT_NEAR(v_frd_again.x(), v_frd.x(), HARD_TOLERANCE);
        EXPECT_NEAR(v_frd_again.y(), v_frd.y(), HARD_TOLERANCE);
        EXPECT_NEAR(v_frd_again.z(), v_frd.z(), HARD_TOLERANCE);
    }
}

TEST_F(FrameCastTest, BodyFramesChained) {
    // This is a key test: FLU -> RFU must pass through the canonical FRD frame.
    // FLU -> FRD -> RFU
    Vector3D<flu, double> v_flu(10.0, 20.0, 30.0);  // {F, L, U}

    auto v_rfu = frame_cast<rfu>(v_flu);

    // Manual Calculation:
    EXPECT_NEAR(v_rfu.right(), -20.0, HARD_TOLERANCE);   // R = -L
    EXPECT_NEAR(v_rfu.forward(), 10.0, HARD_TOLERANCE);  // F = F
    EXPECT_NEAR(v_rfu.up(), 30.0, HARD_TOLERANCE);       // U = U

    // RFU -> FLU (Round Trip)
    auto v_flu_again = frame_cast<flu>(v_rfu);
    EXPECT_NEAR(v_flu_again.x(), v_flu.x(), HARD_TOLERANCE);
    EXPECT_NEAR(v_flu_again.y(), v_flu.y(), HARD_TOLERANCE);
    EXPECT_NEAR(v_flu_again.z(), v_flu.z(), HARD_TOLERANCE);
}

// --- Sensor Frame Conversion Tests ---

TEST_F(FrameCastTest, SensorFramesDirect) {
    // IMU -> CAM
    auto v_cam = frame_cast<camera>(v_imu);
    EXPECT_NEAR(v_cam.x(), 20.0, HARD_TOLERANCE);  // Cam.X (Right) = IMU.Y (Right)
    EXPECT_NEAR(v_cam.y(), 30.0, HARD_TOLERANCE);  // Cam.Y (Down) = IMU.Z (Down)
    EXPECT_NEAR(v_cam.z(), 10.0, HARD_TOLERANCE);  // Cam.Z (Fwd) = IMU.X (Fwd)

    // CAM -> IMU (Round Trip)
    auto v_imu_again = frame_cast<imu>(v_cam);
    EXPECT_NEAR(v_imu_again.x(), v_imu.x(), HARD_TOLERANCE);
    EXPECT_NEAR(v_imu_again.y(), v_imu.y(), HARD_TOLERANCE);
    EXPECT_NEAR(v_imu_again.z(), v_imu.z(), HARD_TOLERANCE);
}

// --- Compile-Time Safety Test ---

TEST_F(FrameCastTest, InvalidCategoryCast) {
    // This code should fail to compile because `ned` (local_tangent) and
    // `frd` (body) belong to different categories.
    /*
    Vector3D<frd, double> v_frd_invalid(1, 2, 3);
    auto v_ned_invalid = frame_cast<ned>(v_frd_invalid);
    */

    // The success of this test is that the code above fails to build.
    SUCCEED();
}

/**
 * @test Test fixture for frame_transform functions.
 * This suite verifies the correctness of the library's conversion and
 * transformation functionalities.
 */
class TransformationConversionTest : public ::testing::Test {
   protected:
    // Use the double-precision WGS-84 model for all transformation tests
    EarthModelWGS84<double> earth;
};

// Verifies the LLA -> ECEF transformation for a simple, known point.
TEST_F(TransformationConversionTest, FrameTransformGeocentric) {
    // A point on the equator at the prime meridian and sea level
    Coordinate3D<lla, double> lla_pos(0.0, 0.0, 0.0);

    // Transform to ECEF
    auto ecef_pos = frame_transform<ecef>(lla_pos, earth);

    // Expected ECEF coordinates:
    // X should be the Earth's semi-major axis.
    // Y and Z should be zero.
    const double semi_major_axis = earth.reference_ellipsoid().semi_major_axis();
    EXPECT_NEAR(ecef_pos.x(), semi_major_axis, HARD_TOLERANCE);
    EXPECT_NEAR(ecef_pos.y(), 0.0, HARD_TOLERANCE);
    EXPECT_NEAR(ecef_pos.z(), 0.0, HARD_TOLERANCE);
}

// Verifies the mathematical integrity of transformations with a round-trip test.
TEST_F(TransformationConversionTest, FrameTransformRoundTrip) {
    // An arbitrary point with non-zero latitude, longitude, and altitude
    Coordinate3D<lla, double> original_lla(30.0, -90.0, 1000.0);

    // Forward transformation: LLA -> ECEF
    auto ecef_pos = frame_transform<ecef>(original_lla, earth);
    const double exp_xe = 0.0;
    const double exp_ye = -5529122.66;  // approximated to cm
    const double exp_ze = 3170873.74;   // approximated to cm
    EXPECT_NEAR(ecef_pos.x(), exp_xe, CM_LEVEL_TOLERANCE);
    EXPECT_NEAR(ecef_pos.y(), exp_ye, CM_LEVEL_TOLERANCE);
    EXPECT_NEAR(ecef_pos.z(), exp_ze, CM_LEVEL_TOLERANCE);

    // Inverse transformation: ECEF -> LLA
    auto final_lla = frame_transform<lla>(ecef_pos, earth);

    // The final LLA should be extremely close to the original
    EXPECT_NEAR(final_lla.latitude(), original_lla.latitude(), SOFT_TOLERANCE);
    EXPECT_NEAR(final_lla.longitude(), original_lla.longitude(), SOFT_TOLERANCE);
    EXPECT_NEAR(final_lla.altitude(), original_lla.altitude(), SOFT_TOLERANCE);
}

// --- Tests for frame_transform (Local Tangent Plane Transformations) ---

// Verifies the projection of a global LLA coordinate into a local NED frame.
TEST_F(TransformationConversionTest, FrameTransformLocalTangent) {
    // Define the origin of our local NED world in LLA
    Coordinate3D<lla, double> local_origin(45.0, 10.0, 100.0);
    auto local_origin_ecef = frame_transform<ecef>(local_origin, earth);

    const double lat_origin_rad = local_origin.latitude(AngleUnit::Rad);

    // Define another point slightly north and east of the origin
    // A small delta in latitude (approx 111m north)
    // A small delta in longitude (approx 78m east at 45 deg lat)
    Coordinate3D<lla, double> point_to_transform(45.001, 10.001, 120.0);

    // difference between coordinates: is a displacement vector
    Vector3D<lla> lla_displacement = point_to_transform - local_origin;

    // Transform the point into the local NED frame centered at the origin
    auto local_pos = frame_transform<ned>(point_to_transform, local_origin, earth);

    // Calculate the (approximated) expected North and East distances using radii of curvature
    const double expected_north = lla_displacement.delta_latitude(AngleUnit::Rad) *
                                  (earth.meridian_radius(lat_origin_rad) + local_origin.altitude());
    const double expected_east = lla_displacement.delta_longitude(AngleUnit::Rad) *
                                 (earth.normal_radius(lat_origin_rad) + local_origin.altitude()) *
                                 std::cos(lat_origin_rad);
    // Expected down is the negative altitude difference
    const double expected_down = -lla_displacement.delta_altitude();

    // Check if the transformed coordinates are close to the expected planar projection
    EXPECT_NEAR(local_pos.north(), expected_north, SOFT_TOLERANCE);
    EXPECT_NEAR(local_pos.east(), expected_east, SOFT_TOLERANCE);
    EXPECT_NEAR(local_pos.down(), expected_down, SOFT_TOLERANCE);

    // recomputation, using different origin frame
    {
        auto local_pos = frame_transform<nwu>(point_to_transform, local_origin_ecef, earth);
        // Check if the transformed coordinates are close to the expected planar projection
        EXPECT_NEAR(local_pos.north(), expected_north, SOFT_TOLERANCE);
        EXPECT_NEAR(local_pos.west(), -expected_east, SOFT_TOLERANCE);
        EXPECT_NEAR(local_pos.up(), -expected_down, SOFT_TOLERANCE);
    }

    // revert: to lla
    {
        auto lla_again = frame_transform<lla>(local_pos, local_origin, earth);

        // Check if the transformed coordinates are close to the expected planar projection
        EXPECT_NEAR(lla_again.latitude(), point_to_transform.latitude(),
                    HARD_TOLERANCE);  // Looser HARD_HARD_TOLERANCE due to linearization
        EXPECT_NEAR(lla_again.longitude(), point_to_transform.longitude(), HARD_TOLERANCE);
        EXPECT_NEAR(lla_again.altitude(), point_to_transform.altitude(), HARD_TOLERANCE);
    }
    // revert: to ecef
    {
        auto ecef_p = frame_transform<ecef>(point_to_transform, earth);
        auto ecef_again = frame_transform<ecef>(local_pos, local_origin, earth);

        // Check if the transformed coordinates are close to the expected planar projection
        EXPECT_NEAR(ecef_p.x(), ecef_again.x(),
                    HARD_TOLERANCE);  // Looser HARD_HARD_TOLERANCE due to linearization
        EXPECT_NEAR(ecef_p.y(), ecef_again.y(), HARD_TOLERANCE);
        EXPECT_NEAR(ecef_p.z(), ecef_again.z(), HARD_TOLERANCE);
    }
}

}  // namespace testing
}  // namespace refx

#endif /* _TRANSFORMATIONS_TEST_TRANSFORMATIONS_ */
