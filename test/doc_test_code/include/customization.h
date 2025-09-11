#ifndef _INCLUDE_CUSTOMIZATION_
#define _INCLUDE_CUSTOMIZATION_

#include <refx/geometry.h>
#include <refx/transformations.h>

#include <iostream>

#include "my_robot_frames.h"
#include "my_robot_vectors.h"

using namespace refx;

namespace customization {

void test_my_robot() {
    // 1. Create a vector in our custom frame.
    auto vec_laser = Coordinate3D<my_robot::laser_scanner>(10.0, 5.0, 2.0);

    // double fwd_distance = vec_laser.x(); // Works, but...
    double fwd_distance = vec_laser.laser_front();  // ...is much clearer with the specialization.
    // Expected result: 10
    std::cout << "Fwd distance: " << fwd_distance << std::endl;

    Rotation<frd, my_robot::laser_scanner> R_laser_to_body(
        YawPitchRoll<double>(M_PI / 2.0, 0.0, 0.0));

    // For example you can rotate from the sensor frame to the body frame, ensuring type-safety
    // the resulting vector is still in the origin of the sensor frame, but aligned with the body
    // axis
    Vector3D<frd> vec_laser_in_body = R_laser_to_body * vec_laser;
    // Then you can add a displacement vector in the body frame, ensuring type-safety, for example
    // the translation calibration
    Vector3D<frd> T_laser_calib_body(2.0, 0.8, 1.5);
    // THIS RESULTS IN A COMPILE ERROR!
    // Vector3D<frd> vec_body = vec_laser + T_laser_calib_body;

    // The resulting (calibrated) mesaurement from laser to body
    // now this sum is safe to do
    Vector3D<frd> vec_body = vec_laser_in_body + T_laser_calib_body;

    // Expected result: [-3, 10.8, 3.5]
    std::cout << "Vector in body frame (frd): " << vec_body << std::endl;

    // 2. Use `frame_cast` to convert to a standard library frame.
    // This works automatically with ZERO custom conversion code!
    // The engine deduces the transform from axis_flu to axis_rdf.
    // remember that with such function, you can transfrom between
    // frames that belongs to the same FrameTag (Body, Sensor, ...)
    auto vec_camera = frame_cast<camera>(vec_laser);

    // Expected result: [-5, -2, 10]
    std::cout << "Vector in camera frame (frd): " << vec_camera << std::endl;

    // 3. Use a Transformation object, to calibrate the sensor measuement in one shot.
    // Define the pose of the laser scanner relative to the vehicle's body.
    auto T_body_from_laser = Transformation<frd, my_robot::laser_scanner>(
        // Assign the rotation calibration
        R_laser_to_body,
        // And the translation calibration
        T_laser_calib_body);

    // The custom frame is a first-class citizen in all type-safe operations.
    vec_body = T_body_from_laser * vec_laser;

    // Expected result: [-3, 10.8, 3.5]
    std::cout << "Vector in body frame (via Transform): " << vec_body << std::endl;
}
}  // namespace customization

#endif /* _INCLUDE_CUSTOMIZATION_ */
