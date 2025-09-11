#ifndef _INCLUDE_GETTING_STARTED_
#define _INCLUDE_GETTING_STARTED_

#include <refx/geometry.h>
#include <refx/transformations.h>

#include <iostream>

using namespace refx;
namespace getting_started {

void test_geometry_transform() {
    // 1. Create frame-aware vectors. Types are tagged with their frame.
    Vector3D<ned> velocity_ned{10.0, -2.0, 0.5};  // {N, E, D}
    Vector3D<frd> omega_body{0.0, 0.0, 0.03};     // {F, R, D}

    std::cout << "Velocity in NED frame: " << velocity_ned << std::endl;

    // 2. COMPILE-TIME SAFETY: Mixing frames is a compiler error.
    // Uncommenting the line below will cause a compile-time error, preventing a common bug.
    // velocity_ned + thrust_in_body;  // ERROR: Incompatible frames!

    // 3. Define a rotation from the body frame to the world (NED) frame.
    // Let's assume a 45-degree yaw (pi/4 radians).
    auto yaw_pitch_roll = YawPitchRoll<double>(M_PI / 4.0, 0.0, 0.0);
    Rotation<ned, frd> R_world_from_body(yaw_pitch_roll);

    // 4. Correctly transform the thrust vector to the world frame.
    Vector3D<frd> velocity_body = R_world_from_body.inverse() * velocity_ned;
    // This will generate a compile-time error:
    // Vector3D<frd> velocity_body = R_world_from_body * velocity_ned; //need .inverse() to rotation
    // Vector3D<flu> velocity_body = R_world_from_body.inverse() * velocity_ned; //result is <frd>

    std::cout << "Velocity in Body Frame: " << velocity_body << std::endl;

    // 5. Compute centripetal acceleration.
    Vector3D<frd> centripetal_acceleration_body = cross(omega_body, velocity_body);
    std::cout << "Centripetal Acceleration Body:  " << centripetal_acceleration_body << std::endl;

    std::cout << "\n--- Coordinate Transformations ---\n" << std::endl;

    // 6. Use `frame_cast` for simple, co-origin conversions (e.g., NED -> ENU).
    // This is a zero-cost, compile-time operation.
    Vector3D<enu> velocity_in_enu = frame_cast<enu>(velocity_ned);
    std::cout << "Velocity in ENU: " << velocity_in_enu << std::endl;

    // 7. Use `frame_transform` for complex projections (e.g., LLA -> NED).
    // This requires a physical context (an origin point).
    // Let's use a location in Rome, Italy.
    Coordinate3D<lla> local_origin(41.9028, 12.4964, 50.0);  // Lat/Lon in deg, Alt in m
    Coordinate3D<lla> target_gps_point(41.9030, 12.4966, 60.0);

    auto earthmodel = EarthModelWGS84<double>();

    // This performs a runtime projection of the GPS point onto a flat plane at the origin.
    Coordinate3D<ned> target_in_local_ned =
        frame_transform<ned>(target_gps_point, local_origin, earthmodel);

    std::cout << "Target GPS in Local NED Frame: " << target_in_local_ned << " (meters)"
              << std::endl;
    // Convert a LLA position to ECEF
    auto local_origin_ecef = frame_transform<ecef>(local_origin, earthmodel);
    std::cout << "Local Origin in ECEF: " << local_origin_ecef << std::endl;

    // The same local NED position can be obtained with the same origin expressed in ECEF frame
    target_in_local_ned = frame_transform<ned>(target_gps_point, local_origin_ecef, earthmodel);
    std::cout << "Target GPS in Local NED Frame (from ecef origin): " << target_in_local_ned
              << " (meters)" << std::endl;
}

}  // namespace getting_started

#endif /* _INCLUDE_GETTING_STARTED_ */
