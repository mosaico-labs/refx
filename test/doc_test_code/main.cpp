#include "include/cookbook.h"
#include "include/customization.h"
#include "include/getting_started.h"

int main(int argc, char** argv) {
    /*
     * GETTING STARTED-RELATED TESTS
     */
    std::cout << "\n================ Getting Started=> Test: geometry and transformations  "
                 "================"
              << std::endl;
    getting_started::test_geometry_transform();

    /*
     * CUSTOMIZATION-RELATED TESTS
     */
    std::cout << "\n================ Customization=> Test: my_robot ================" << std::endl;
    customization::test_my_robot();

    /*
     * COOKBOOK-RELATED TESTS
     */
    std::cout << "\n================ Cookbook=> Test: From Relative Sensor Data to Global Position "
                 "================"
              << std::endl;
    cookbook::test_recipe_relative_to_global();

    std::cout << "\n================ Cookbook=> Test: IMU Measurements Compensation for Navigation "
                 "Filters ================"
              << std::endl;
    cookbook::test_recipe_imu_comp();

    return 0;
}