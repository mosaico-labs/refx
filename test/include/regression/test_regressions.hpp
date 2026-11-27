#ifndef _REGRESSION_TEST_REGRESSIONS_
#define _REGRESSION_TEST_REGRESSIONS_

#include <cmath>
#include <type_traits>

#include "../tol.h"
#include "gtest/gtest.h"
#include "refx/geometry.h"
#include "refx/math.h"
#include "refx/models.h"
#include "refx/transformations.h"

/**
 * @file test_regressions.hpp
 * @brief Regression tests for fixed bugs.
 */

namespace refx {
namespace testing {

// ---------------------------------------------------------------------------
// Angular arithmetic on non-Cartesian coordinates (lla, aer)
// ---------------------------------------------------------------------------

// The latitude is bounded, not periodic: the difference between two latitudes never wraps.
TEST(RegressionTest, LatitudeDifferenceDoesNotWrap) {
    Coordinate3D<lla> a(80.0, 0.0, 0.0), b(-80.0, 0.0, 0.0);
    EXPECT_NEAR((a - b).delta_latitude(), 160.0, HARD_TOLERANCE);
    EXPECT_NEAR((b - a).delta_latitude(), -160.0, HARD_TOLERANCE);
}

// Crossing a pole reflects the latitude and moves the longitude by 180 degrees.
TEST(RegressionTest, LatitudeTranslationCrossesPole) {
    {
        Coordinate3D<lla> c = Coordinate3D<lla>(80.0, 10.0, 5.0) + Vector3D<lla>(20.0, 0.0, 1.0);
        EXPECT_NEAR(c.latitude(), 80.0, HARD_TOLERANCE);
        EXPECT_NEAR(c.longitude(), -170.0, HARD_TOLERANCE);
        EXPECT_NEAR(c.altitude(), 6.0, HARD_TOLERANCE);
    }
    {
        Coordinate3D<lla> c = Coordinate3D<lla>(-80.0, 10.0, 0.0) + Vector3D<lla>(-20.0, 0.0, 0.0);
        EXPECT_NEAR(c.latitude(), -80.0, HARD_TOLERANCE);
        EXPECT_NEAR(c.longitude(), -170.0, HARD_TOLERANCE);
    }
    {
        // Longitude wraps across the antimeridian.
        Coordinate3D<lla> c = Coordinate3D<lla>(0.0, 170.0, 0.0) + Vector3D<lla>(0.0, 20.0, 0.0);
        EXPECT_NEAR(c.latitude(), 0.0, HARD_TOLERANCE);
        EXPECT_NEAR(c.longitude(), -170.0, HARD_TOLERANCE);
    }
}

// Increments are vectors: their arithmetic is linear, with no angular normalization.
TEST(RegressionTest, GeodeticIncrementArithmeticIsLinear) {
    Vector3D<lla> d1(50.0, 170.0, 1.0), d2(60.0, 20.0, 2.0);
    Vector3D<lla> scaled = d1 * 2.0;
    EXPECT_NEAR(scaled.delta_latitude(), 100.0, HARD_TOLERANCE);
    EXPECT_NEAR(scaled.delta_longitude(), 340.0, HARD_TOLERANCE);
    Vector3D<lla> sum = d1 + d2;
    EXPECT_NEAR(sum.delta_latitude(), 110.0, HARD_TOLERANCE);
    EXPECT_NEAR(sum.delta_longitude(), 190.0, HARD_TOLERANCE);
    Vector3D<lla> neg = -d1;
    EXPECT_NEAR(neg.delta_latitude(), -50.0, HARD_TOLERANCE);
    EXPECT_NEAR(neg.delta_longitude(), -170.0, HARD_TOLERANCE);
}

// c + (b - c) must give back b, also across the antimeridian.
TEST(RegressionTest, CoordinateDifferenceRoundTrip) {
    const Coordinate3D<lla> points[] = {
        {45.0, 10.0, 100.0}, {-33.0, 179.5, 0.0}, {12.0, -179.5, -20.0}, {89.0, -45.0, 3.0}};
    for (const auto& b : points) {
        for (const auto& c : points) {
            Coordinate3D<lla> r = c + (b - c);
            EXPECT_NEAR(r.latitude(), b.latitude(), HARD_TOLERANCE);
            EXPECT_NEAR(r.longitude(), b.longitude(), HARD_TOLERANCE);
            EXPECT_NEAR(r.altitude(), b.altitude(), HARD_TOLERANCE);
        }
    }
}

// The azimuth difference is the shortest angle, in [-180, 180).
TEST(RegressionTest, AzimuthDifferenceIsShortest) {
    Coordinate3D<aer> a(350.0, 10.0, 100.0), b(10.0, 10.0, 100.0);
    EXPECT_NEAR((a - b).delta_azimuth(), -20.0, HARD_TOLERANCE);
    EXPECT_NEAR((b - a).delta_azimuth(), 20.0, HARD_TOLERANCE);
}

// Crossing the zenith reflects the elevation and moves the azimuth by 180 degrees.
TEST(RegressionTest, ElevationTranslationCrossesZenith) {
    Coordinate3D<aer> c = Coordinate3D<aer>(30.0, 80.0, 100.0) + Vector3D<aer>(0.0, 20.0, 0.0);
    EXPECT_NEAR(c.azimuth(), 210.0, HARD_TOLERANCE);
    EXPECT_NEAR(c.elevation(), 80.0, HARD_TOLERANCE);
    EXPECT_NEAR(c.range(), 100.0, HARD_TOLERANCE);
}

// ---------------------------------------------------------------------------
// ECEF -> LLA at the poles
// ---------------------------------------------------------------------------

TEST(RegressionTest, EcefToLlaAtThePoles) {
    EarthModelWGS84<double> earth;
    const double b = earth.polar_radius();
    {
        auto p = frame_transform<lla>(Coordinate3D<ecef>(0.0, 0.0, b + 100.0), earth);
        EXPECT_NEAR(p.latitude(), 90.0, HARD_TOLERANCE);
        EXPECT_NEAR(p.altitude(), 100.0, CM_LEVEL_TOLERANCE);
    }
    {
        auto p = frame_transform<lla>(Coordinate3D<ecef>(0.0, 0.0, -(b + 100.0)), earth);
        EXPECT_NEAR(p.latitude(), -90.0, HARD_TOLERANCE);
        EXPECT_NEAR(p.altitude(), 100.0, CM_LEVEL_TOLERANCE);
    }
    for (double lat : {89.9999999, 89.99, -89.99, 0.0, 45.0}) {
        Coordinate3D<lla> orig(lat, 30.0, 250.0);
        auto rt = frame_transform<lla>(frame_transform<ecef>(orig, earth), earth);
        EXPECT_NEAR(rt.latitude(), lat, SOFT_TOLERANCE);
        EXPECT_NEAR(rt.altitude(), 250.0, CM_LEVEL_TOLERANCE);
    }
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

// Brace and parenthesis initialization must agree: missing components are zero.
TEST(RegressionTest, InitializerListZeroFillsMissingComponents) {
    Vector3D<ecef> v{1.0};
    EXPECT_EQ(v.x(), 1.0);
    EXPECT_EQ(v.y(), 0.0);
    EXPECT_EQ(v.z(), 0.0);
    Vector3D<ecef> w{1.0, 2.0};
    EXPECT_EQ(w.z(), 0.0);
}

// Too many elements: assertion in debug builds, never an out-of-bounds write.
TEST(RegressionTest, InitializerListTooManyElements) {
    auto build_oversized = [] {
        Vector3D<ecef> v{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
        (void)v;
    };
    EXPECT_DEBUG_DEATH(build_oversized(), "");
}

// ---------------------------------------------------------------------------
// Rotations
// ---------------------------------------------------------------------------

// inverse() must keep the scalar type and be callable on const objects.
TEST(RegressionTest, EulerInverseKeepsScalarTypeAndIsConst) {
    const YawPitchRoll<float> e(0.3f, -0.2f, 0.1f);
    const EulerAngles<EulerSequence::XYZ, float> inv = e.inverse();
    auto R = Rotation<ned, frd, float>(e);
    auto R_inv = Rotation<frd, ned, float>(inv);
    UnitQuaternion<float> q = (R * R_inv).to_quaternion();
    EXPECT_NEAR(std::abs(q.w()), 1.0f, 1e-6f);
}

// ---------------------------------------------------------------------------
// Angles
// ---------------------------------------------------------------------------

// Integral inputs are promoted to double instead of being truncated.
TEST(RegressionTest, AngleConversionPromotesIntegers) {
    EXPECT_NEAR(deg2rad(90), M_PI / 2.0, HARD_TOLERANCE);
    EXPECT_NEAR(rad2deg(1), 180.0 / M_PI, HARD_TOLERANCE);
    static_assert(std::is_same_v<decltype(deg2rad(1.0f)), float>);
}

// The sign of angles in (-1, 0) degrees must not be lost.
TEST(RegressionTest, DmsKeepsSignOfSmallNegativeAngles) {
    {
        auto [d, m, s] = degToDMS(-0.5);
        EXPECT_EQ(d, 0);
        EXPECT_EQ(m, -30);
        EXPECT_NEAR(s, 0.0, 1e-6);
    }
    {
        auto [d, m, s] = degToDMS(-12.5);
        EXPECT_EQ(d, -12);
        EXPECT_EQ(m, 30);
        EXPECT_NEAR(s, 0.0, 1e-6);
    }
    for (double angle : {-12.5, 12.5, -0.5, -0.001, 0.001, 179.999}) {
        auto [d, m, s] = degToDMS(angle);
        EXPECT_NEAR(DMSToDeg(d, m, s), angle, 1e-9);
    }
}

// ---------------------------------------------------------------------------
// Models
// ---------------------------------------------------------------------------

// The eccentricity must be consistent with the flattening: e^2 = f (2 - f).
TEST(RegressionTest, EllipsoidEccentricityConsistentWithFlattening) {
    auto check = [](const ReferenceEllipsoid<double>& el) {
        const double f = 1.0 / el.inverse_flattening();
        const double e2 = f * (2.0 - f);
        EXPECT_NEAR(el.eccentricity() * el.eccentricity(), e2, 1e-15) << el.name();
    };
    check(ReferenceEllipsoidWGS84<double>());
    check(ReferenceEllipsoidGRS80<double>());
}

}  // namespace testing
}  // namespace refx

#endif /* _REGRESSION_TEST_REGRESSIONS_ */
