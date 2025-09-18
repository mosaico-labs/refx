#ifndef _INCLUDE_COOKBOOK_
#define _INCLUDE_COOKBOOK_

#include <refx/geometry.h>
#include <refx/transformations.h>

#include <iostream>

#include "my_robot_frames.h"

using namespace refx;
namespace cookbook {

/// @brief Implementation of the code in the Recipe: 'From Relative Sensor Data to Global Position'
void test_recipe_relative_to_global() {
    // --------- Step 1: Define the Ego-Vehicle's State --------- //
    // Our ego-vehicle is at a known global position.
    auto ego_position_global = Coordinate3D<lla>(44.3040729, 11.9530427);

    // It's pointing -45 degrees off the North (a -45-degree yaw).
    auto ego_orientation_global =
        Rotation<ned, frd>(YawPitchRoll<double>(deg2rad(-45.0), 0.0, 0.0));

    // --------- Step 2: Define the Sensor Measurement --------- //
    // The complete pose of the radar wrt body frame (relative mounting)
    auto T_frd_radar = refx::Transformation<refx::frd, my_robot::radar>(
        Rotation<refx::frd, my_robot::radar>(
            YawPitchRoll<double>(refx::deg2rad(2.0), -refx::deg2rad(1.5), 0.0)),
        refx::Vector3D<refx::frd>(1.0, -0.3, 0.15));

    // The radar detects the target 75 meters directly in front of us
    // and 10 meters to the left.
    auto target_position_relative = Coordinate3D<my_robot::radar>(75.0, -10.0, 0.0);
    // The radar measurement in the body frame
    auto target_position_relative_body = T_frd_radar * target_position_relative;

    std::cout << "Target position relative (Body): " << target_position_relative_body << " [m]"
              << std::endl;

    // --------- Step 3: Convert and Transform --------- //
    Coordinate3D<ned> target_position_relative_ned =
        ego_orientation_global * target_position_relative_body;

    std::cout << "Target position relative (NED): " << target_position_relative_ned << " [m]"
              << std::endl;

    Coordinate3D<lla> target_position_global = frame_transform<lla>(
        target_position_relative_ned, ego_position_global, EarthModelWGS84<double>());

    // We can now use target_position_global for navigation, tracking, etc.
    std::cout << "Target's Absolute Position (LLA): " << target_position_global << " [m]"
              << std::endl;
}

/// @brief Implementation of the code in the Recipe: 'IMU Measurements Compensation for Navigation
/// Filters'
void test_recipe_imu_comp() {
    // --------- Step 1: Define the Vehicle's Current State --------- //
    // The vehicle is located near Cuneo, Italy.
    auto current_position = refx::Coordinate3D<refx::lla>(44.39, 7.58, 534.0);

    // The vehicle is traveling North but is pitched up by 5 degrees and banked by 2 degrees.
    auto R_ned_from_frd = refx::Rotation<refx::ned, refx::frd>(
        refx::YawPitchRoll<double>(0.0, refx::deg2rad(5.0), refx::deg2rad(2.0)));

    // --------- Step 2: Get Raw IMU Measurements --------- //
    refx::Rotation<refx::frd, refx::imu> R_body_imu;  // this is the identity rotation

    // Raw accelerometer reading in body frame (m/s^2)
    // Note that the sensed gravity is with the minus sign: the gravity in the accelerometers
    // reading acts as an apparent acceleration, directed toward up
    auto accel_raw_body = R_body_imu * refx::Vector3D<refx::imu>(0.85, 0.1, -9.75);

    // Raw gyroscope reading in body frame (rad/s)
    auto gyro_raw_body = R_body_imu * refx::Vector3D<refx::imu>(0.01, -0.02, 0.03);

    // --------- Step 3: Correct the Accelerometer for Gravity --------- //
    // Instantiate the Earth model
    refx::EarthModelWGS84<double> earth_model;

    // 1. Calculate the gravity vector in the navigation (NED) frame.
    //    Gravity points "Down", so its N and E components are zero.
    auto gravity_in_ned =
        refx::Vector3D<refx::ned>(0.0, 0.0, earth_model.gravity(current_position));

    // 2. Rotate the gravity vector into the vehicle's body (FRD) frame.
    // We need the inverse rotation to go from NED -> FRD.
    // The minus sign on the body gravity is justified to account for the apparent nature of gravity
    // acceleration on the accelerometers reading
    auto gravity_in_frd = -(R_ned_from_frd.inverse() * gravity_in_ned);

    // 3. Compensate the gravity from the accelerometer, to get the true linear acceleration.
    auto linear_accel_true = accel_raw_body - gravity_in_frd;

    // --------- Step 4: Correct the Gyroscope for Earth's Rotatio --------- //

    // 1. Calculate the Earth's rotation vector in the body (FRD) frame.
    // To do so, we must first compute the ECEF-to-NED rotation,
    // that later canbe composed with the FRD-to-NED rotation ;
    // The following computations are taken by Fig 3.1, page 50, and Eq. (3.12), page 57, Book:
    // "Applied Mathematics in Integrated Navigation Systems" 3rd Edition, Robert M. Rogers

    // Rotation about the Y-axis by pi/2 (NED->Intermediate F1)
    UnitQuaternion<double> q_pi_2 = UnitQuaternion<double>::from_rotation_y(M_PI_2);
    // Rotation about the Y-axis by latitude (Intermediate F1 -> Intermediate F2)
    UnitQuaternion<double> q_lat =
        UnitQuaternion<double>::from_rotation_y(current_position.latitude(AngleUnit::Rad));
    // Rotation about the Z-axis by -longitude (Intermediate F2 -> ECEF)
    UnitQuaternion<double> q_lon =
        UnitQuaternion<double>::from_rotation_z(-current_position.longitude(AngleUnit::Rad));
    // Compose the rotations. This is a pure mathematical operation.
    UnitQuaternion<double> final_q = q_pi_2 * q_lat * q_lon;

    // Construct the final, type-safe Rotation object from our result.
    refx::Rotation<refx::ned, refx::ecef> R_ecef_to_ned(final_q);

    // 1.2 Create the Earth rate vector in its native (ECEF) frame
    auto earth_rate_in_ecef =
        refx::Vector3D<refx::ecef>(0.0, 0.0, earth_model.reference_ellipsoid().angular_velocity());

    // 2. Rotate the Earth rate vector into the vehicle's body (FRD) frame.
    auto earth_rate_in_frd = R_ned_from_frd.inverse() * R_ecef_to_ned * earth_rate_in_ecef;

    // 3. Subtract the Earth rate to get the vehicle's true angular velocity relative to the Earth.
    auto angular_velocity_true = gyro_raw_body - earth_rate_in_frd;

    std::cout << "Corrected Linear Accel (FRD): " << linear_accel_true << " [m/s^2]" << std::endl;
    std::cout << "Corrected Angular Velocity (FRD): " << angular_velocity_true << " [m/s^2]"
              << std::endl;
}
}  // namespace cookbook
#endif /* _INCLUDE_COOKBOOK_ */
