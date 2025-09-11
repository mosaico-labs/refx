← [Back to Cookbook](main.md) | 

---

## IMU Measurements Compensation for Navigation Filters

This recipe is for robotics engineers developing Inertial Navigation Systems (INS) or sensor fusion filters for localization. It details the critical first step in processing IMU data: correcting raw measurements for physical effects like gravity and the Earth's rotation. Failure to perform these corrections correctly and in the right reference frame is a primary source of large, compounding drift in navigation systems.

### The Goal

Given raw accelerometer and gyroscope readings from an IMU mounted on a vehicle, our goal is to compute the vehicle's **true linear acceleration and angular velocity** relative to the Earth's surface. These corrected values are the essential inputs for the "prediction" step in a Kalman filter or similar state estimator.

### The Challenge

1.  An **accelerometer** does not measure true linear acceleration. It measures "specific force," which is the sum of true acceleration and the opposing force of gravity. We must subtract the gravity vector to isolate the true motion.
2.  A **gyroscope** measures angular velocity relative to an inertial frame. For navigation on Earth, we need the angular velocity relative to the Earth's surface. We must subtract the Earth's rotation rate to get this.

### Ingredients

  * `refx::EarthModelWGS84`: To get accurate local gravity and Earth rotation data.
  * `refx::Vector3D`: To represent all physical quantities (acceleration, angular velocity, gravity).
  * `refx::Rotation`: To transform vectors between the navigation and body frames.
  * `refx::Coordinate3D`: To specify the vehicle's location, which affects gravity and Earth rate.
  * **Frames**:
      * `refx::ned`: The global **N**orth-**E**ast-**D**own navigation frame.
      * `refx::frd`: The vehicle's body-fixed **F**orward-**R**ight-**D**own frame where the IMU is mounted.

-----

### Step-by-Step Instructions

#### Step 1: Define the Vehicle's Current State

We need to know the vehicle's current position (`Coordinate`) and orientation (`Rotation`) to calculate the corrections.

```cpp
//TODO: maybe we have to define this header
#include <refx/geometry.h>

// The vehicle is located near Cuneo, Italy.
auto current_position = refx::Coordinate3D<refx::lla>(44.39, 7.58, 534.0);

// The vehicle is traveling North but is pitched up by 5 degrees and banked by 2 degrees.
auto R_ned_from_frd = refx::Rotation<refx::ned, refx::frd>(
    refx::YawPitchRoll<double>(0.0, refx::deg2rad(5.0), refx::deg2rad(2.0))
);
```

#### Step 2: Get Raw IMU Measurements

These are the raw readings from our sensor, expressed in the vehicle's body-fixed `frd` frame. Here we assume for simplicity that the imu is aligned with the body `frd` axes.

```cpp
// Raw accelerometer reading (m/s^2)
// Note that the sensed gravity is with the minus sign: the gravity in the accelerometers
// reading acts as an apparent acceleration, directed toward up
auto accel_raw = refx::Vector3D<refx::frd>(0.85, 0.1, -9.75);

// Raw gyroscope reading (rad/s)
auto gyro_raw = refx::Vector3D<refx::frd>(0.01, -0.02, 0.03);
```

#### Step 3: Correct the Accelerometer for Gravity

To get *true* acceleration, we subtract the gravity vector. This operation **must** be done in the same frame, so we will calculate gravity in the `ned` frame and then rotate it into the `frd` frame to match the sensor reading.

```cpp
// Instantiate the Earth model
refx::EarthModelWGS84<double> earth_model;

// 1. Calculate the gravity vector in the navigation (NED) frame.
// Gravity points "Down", so its N and E components are zero.
auto gravity_in_ned = refx::Vector3D<refx::ned>(
    0.0, 0.0, earth_model.gravity(current_position)
);

// 2. Rotate the gravity vector into the vehicle's body (FRD) frame.
// We need the inverse rotation to go from NED -> FRD.
// The minus sign on the body gravity is justified to account for the apparent nature of gravity
// acceleration on the accelerometers reading
auto gravity_in_frd = -R_ned_from_frd.inverse() * gravity_in_ned;

// 3. Compensate the gravity from the accelerometer, to get the true linear acceleration.
auto linear_accel_true = accel_raw - gravity_in_frd;
```

#### Step 4: Correct the Gyroscope for Earth's Rotation

Similarly, we subtract the Earth's rotation vector. We calculate it in the `ned` frame (where it has North and Down components) and then rotate it into the `frd` frame.

```cpp

// 1. Calculate the Earth's rotation vector in the body (FRD) frame.
// To do so, we must first compute the ECEF-to-NED rotation, 
// that later canbe composed with the FRD-to-NED rotation ;
// The following computations are taken by Fig 3.1, page 50, and Eq. (3.12), page 57, Book:
// "Applied Mathematics in Integrated Navigation Systems" 3rd Edition, Robert M. Rogers 

// Rotation about the Y-axis by pi/2 (NED->Intermediate F1)
UnitQuaternion<double> q_pi_2 = UnitQuaternion<double>::from_rotation_y(M_PI_2);
// Rotation about the Y-axis by latitude (Intermediate F1 -> Intermediate F2)
UnitQuaternion<double> q_lat = UnitQuaternion<double>::from_rotation_y(current_position.latitude(AngleUnit::Rad));
// Rotation about the Z-axis by -longitude (Intermediate F2 -> ECEF)
UnitQuaternion<double> q_lon = UnitQuaternion<double>::from_rotation_z(-current_position.longitude(AngleUnit::Rad));
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
auto angular_velocity_true = gyro_raw - earth_rate_in_frd;

std::cout << "Corrected Linear Accel (FRD): " << linear_accel_true << std::endl;
std::cout << "Corrected Angular Velocity (FRD): " << angular_velocity_true << std::endl;
```

-----

### The Main Takeaway

High-fidelity navigation requires correcting for physical phenomena. The refx library ensures this is done safely by enforcing that these corrections are applied in the correct reference frame. By using `Rotation` to transform vectors like gravity and Earth's rotation rate into the sensor's frame **before** subtraction, you eliminate a class of errors that are notoriously difficult to debug and a major source of navigation filter divergence.
