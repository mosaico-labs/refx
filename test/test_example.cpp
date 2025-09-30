#include <gtest/gtest.h>

//
// ---------------------- Geometry Entities and Operations Testing ----------------------
//
#include "include/geometry/test_rotation.hpp"
#include "include/geometry/test_transformation.hpp"
#include "include/geometry/test_vector_coordinate.hpp"
#include "include/geometry/test_vector_operations.hpp"

//
// ---------------------- Earth Model Testing ----------------------
//
#include "include/models/test_models.hpp"

//
// ---------------------- Frame Transformations Testing ----------------------
//
#include "include/transformations/test_transformations.hpp"
/**
 * @brief Main function to run all Google Test suites.
 */
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
