#ifndef _GEOMETRY_TEST_VECTOR_OPERATIONS_
#define _GEOMETRY_TEST_VECTOR_OPERATIONS_
#include <gtest/gtest.h>

// Include all the necessary headers from your library
#include "../tol.h"
#include "refx/geometry.h"

// Use a consistent namespace for tests
namespace refx {
namespace testing {

/**
 * @test Test fixture for Vector3D arithmetic operations.
 * This suite verifies the correctness of vector operations
 */
class VectorOperationsTest : public ::testing::Test {
   protected:
    // Define some common vectors for testing
    Vector3D<ned, double> v1_ned{3.0, 4.0, 0.0};  // magnitude = 5.0
    Vector3D<ned, double> v2_ned{1.0, 0.0, 0.0};  // unit vector in north direction
    Vector3D<ned, double> v3_ned{0.0, 1.0, 0.0};  // unit vector in east direction
};

// Test that dot product returns a scalar value and is computed correctly
TEST_F(VectorOperationsTest, DotProductScalarResult) {
    // Compute dot product
    double dot_result = dot(v1_ned, v2_ned);

    // Verify the mathematical result: (3,4,0) · (1,0,0) = 3
    EXPECT_NEAR(dot_result, 3.0, HARD_TOLERANCE);

    // Test with perpendicular vectors (dot product should be 0)
    double perpendicular_dot = dot(v2_ned, v3_ned);
    EXPECT_NEAR(perpendicular_dot, 0.0, HARD_TOLERANCE);

    // Test with parallel vectors (dot product should be product of magnitudes)
    Vector3D<ned, double> v4_ned{2.0, 0.0, 0.0};  // parallel to v2_ned
    double parallel_dot = dot(v2_ned, v4_ned);
    EXPECT_NEAR(parallel_dot, 2.0, HARD_TOLERANCE);  // 1 * 2 = 2
}

}  // namespace testing
}  // namespace refx

#endif /* _GEOMETRY_TEST_VECTOR_OPERATIONS_ */