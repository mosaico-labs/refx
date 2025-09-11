#ifndef _MODELS_TEST_MODELS_
#define _MODELS_TEST_MODELS_

#include <gtest/gtest.h>

#include "../tol.h"

// Include all the necessary headers from your library
#include "refx/models.h"

// Use a consistent namespace for tests
namespace refx {
namespace testing {

//
// ---------------------- Earth Model Testing ----------------------
//

/**
 * @test Test fixture for the EarthModel and its components.
 * This suite verifies the correctness of the WGS-84 standard models.
 */
class EarthModelTest : public ::testing::Test {
   protected:
    // Use the double-precision WGS-84 model for all tests
    EarthModelWGS84<double> earth;
};

// Verify the key defining constants of the WGS-84 reference ellipsoid
TEST_F(EarthModelTest, WGS84EllipsoidConstantsAreCorrect) {
    const auto& ellipsoid = earth.reference_ellipsoid();
    EXPECT_NEAR(ellipsoid.semi_major_axis(), 6378137.0, HARD_TOLERANCE);
    EXPECT_NEAR(ellipsoid.inverse_flattening(), 298.257223563, HARD_TOLERANCE);
    EXPECT_NEAR(ellipsoid.angular_velocity(), 7.292115e-5, HARD_TOLERANCE);
    const double semi_major_axis = earth.reference_ellipsoid().semi_major_axis();
    const double semi_minor_axis = earth.reference_ellipsoid().semi_minor_axis();
    const double computed_ecc_squared =
        1.0 - semi_minor_axis * semi_minor_axis / (semi_major_axis * semi_major_axis);
    EXPECT_NEAR(earth.reference_ellipsoid().eccentricity(), std::sqrt(computed_ecc_squared),
                HARD_TOLERANCE);
}

// Verify the key defining constants of the WGS-84 gravity model
TEST_F(EarthModelTest, WGS84GravityConstantsAreCorrect) {
    const auto& gravity = earth.gravity_model();
    EXPECT_NEAR(gravity.gamma_e(), 9.7803253359, HARD_TOLERANCE);  // Gravity at equator
    EXPECT_NEAR(gravity.gamma_p(), 9.8321849378, HARD_TOLERANCE);  // Gravity at poles
    EXPECT_NEAR(gravity.GM(), 3.986004418e14, HARD_TOLERANCE);
}

// Test the normal gravity calculation at two key locations: the equator and the North Pole
TEST_F(EarthModelTest, GravityCalculationIsCorrectAtEquatorAndPole) {
    // At the equator (0 degrees latitude), gravity should match gamma_e
    Coordinate3D<lla, double> equator_pos(0.0, 0.0, 0.0);
    EXPECT_NEAR(earth.gravity(equator_pos), earth.gravity_model().gamma_e(), HARD_TOLERANCE);

    // At the North Pole (90 degrees latitude), gravity should match gamma_p
    Coordinate3D<lla, double> pole_pos(90.0, 0.0, 0.0);
    EXPECT_NEAR(earth.gravity(pole_pos), earth.gravity_model().gamma_p(), HARD_TOLERANCE);
}

// Test the radii of curvature calculations at the equator
TEST_F(EarthModelTest, RadiiOfCurvatureAreCorrectAtEquator) {
    const double lat_equator_rad = 0.0;
    const double semi_major_axis = earth.reference_ellipsoid().semi_major_axis();
    const double semi_minor_axis = earth.reference_ellipsoid().semi_minor_axis();

    // At the equator, the normal radius is just the semi-major axis 'a'
    EXPECT_NEAR(earth.normal_radius(lat_equator_rad), semi_major_axis, HARD_TOLERANCE);

    // At the equator, the meridian radius is b^2 / a
    const double expected_meridian_radius = (semi_minor_axis * semi_minor_axis) / semi_major_axis;
    EXPECT_NEAR(earth.meridian_radius(lat_equator_rad), expected_meridian_radius, SOFT_TOLERANCE);
}
}  // namespace testing
}  // namespace refx
#endif /* _MODELS_TEST_MODELS_ */
