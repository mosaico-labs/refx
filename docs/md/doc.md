# Documentation

## Table of Contents

  * [Introduction and Core Concepts](#introduction-and-core-concepts)
  * [Geometric Primitives](#geometric-primitives)
  * [Geodetic Models](#geodetic-models)
  * [Frame Conversions and Transformations](#frame-conversions-and-transformations)
  * [Common Workflows for Developers](#common-workflows-for-developers)
  * [Types Customization](#types-customization)
  * [Appendix: Eigen3 Integration](#appendix-eigen3-integration)
  * [Appendix: Default Frame Tags Reference](#appendix-default-frame-tags-reference)
  * [Appendix: Default Frame Axis Reference](#appendix-default-frame-axis-reference)



## Introduction and Core Concepts

### The `refx` Frame System

The `refx` library's frame system is architected to move the semantics of coordinate frames from runtime properties or implicit conventions into the C++ type system itself. This is achieved through a compositional type architecture, where a frame's semantics are constructed from a set of two fundamental, compile-time entities: its **axis system** and its **frame tag**.

1.  **Axis Systems**: This describes the physical structure and properties of the frame's basis, that is the orientation and direction of its orthogonal system of axes. To handle the fundamental differences between coordinate systems, the library provides two parallel axis definition systems:
      * **`DirectionalAxis<...>`**: For right-handed Cartesian frames, this template uses `AxisDirection` enumerators (`Forw`, `Right`, `Up`, etc.) to define the geometric orientation of the X, Y, and Z axes. The library provides all of the 24 possible right-handed frame axis configurations.
      * **`SemanticAxis<...>`**: For non-Cartesian frames (e.g., geodetic or spherical), this template uses `AxisSemantic` enumerators (`Latitude`, `Longitude`, `Altitude`, etc.) to define the physical meaning of each vector component.
2.  **`FrameTag`**: This `enum class` serves as the highest-level classifier, defining the conceptual domain or category to which a frame belongs (e.g., `Geocentric`, `LocalTangent`, `Body`). It is the primary mechanism for enforcing high-level logical rules, such as preventing a direct cast between a body-fixed frame and a geocentric one.

Some examples of axis definition are the following:

```cpp
using axis_frd = DirectionalAxis<AxisDirection::Forw, AxisDirection::Right, AxisDirection::Down>;
using axis_flu = DirectionalAxis<AxisDirection::Forw, AxisDirection::Left, AxisDirection::Up>;
// ...
using axis_lla = SemanticAxis<AxisSemantic::Latitude, AxisSemantic::Longitude, AxisSemantic::Altitude>;
using axis_aer = SemanticAxis<AxisSemantic::Azimuth, AxisSemantic::Elevation, AxisSemantic::Range>;
// ...
```

This architecture allows for both precision and reuse. For example, multiple frames can share an axis system while belonging to different categories, making them semantically distinct.

```cpp
// from "frames.h"
namespace refx {
    /**
     * @brief A local tangent frame for navigation.
     * Semantics: Composed from a Forward-Right-Down axis system within the
     * LocalTangent domain.
     */
    struct ned {
        static constexpr auto name = "ned";
        using axis = axis_frd; // DirectionalAxis<Forw, Right, Down>
        static constexpr FrameTag tag = FrameTag::LocalTangent;
    };
    
    /**
     * @brief A body-fixed frame for a vehicle.
     * Semantics: Also composed from a Forward-Right-Down axis system, but
     * within the Body domain. This makes it a distinct type from 'ned'.
     */
    struct frd {
        static constexpr auto name = "frd";
        using axis = axis_frd; // DirectionalAxis<Forw, Right, Down>
        static constexpr FrameTag tag = FrameTag::Body;
    };

    /**
     * @brief A non-Cartesian geodetic frame.
     * Semantics: Composed from a Latitude-Longitude-Altitude system within
     * the Geocentric domain.
     */
    struct lla {
        static constexpr auto name = "lla";
        using axis = axis_lla; // SemanticAxis<Latitude, Longitude, Altitude>
        static constexpr FrameTag tag = FrameTag::Geocentric;
    };
}
```

#### How the Architecture Enables Safety and Static Polymorphism

Geometric types are templated on these frame structs, embedding the complete semantic definition into the type of every object. This enables the library's internal engines to use C++ metaprogramming techniques to query these properties at compile time:

  * **Type Safety**: An operation between vectors, coordinates and rotations operators of incompatible frames can be disallowed at compile time, preventing logical and mathematical errors.

  * **Static Polymorphism**: The engines designed for frame transformations (e.g. `frame_cast`) can use the axis type traits to deduce the correct component transformations at compile time with zero runtime cost, and assert the invalid transformations between objects of incompatible frames.

This architecture creates a system where the C++ compiler can validate type identity and the underlying physical and mathematical contracts of the frames involved, ensuring that your geometric code is provably correct by construction.

## Geometric Primitives

### Coordinate3D: Representing Positions

The `Coordinate3D<Frame, T>` class represents a position in a specific reference frame; think of it as a single, fixed location.

#### Example: A Local Cartesian Coordinate

In a local frame like *North-East-Down*, a coordinate represents a specific point relative to that frame's origin.

```cpp
//include header for geometry entities
#include <refx/geometry.h>

// A coordinate representing a point 100m North, 50m East, and 10m deep
// relative to the local origin.
refx::Coordinate3D<refx::ned> vehicle_position(100.0, 50.0, 10.0);

// Access its components with clear, semantic names
double north_pos = vehicle_position.north(); // returns 100.0
double east_pos  = vehicle_position.east();  // returns 50.0
double down_pos  = vehicle_position.down();  // returns 10.0
```

#### Example: A Geodetic Coordinate

In a geodetic frame like *Latitude-Longitude-Altitude*, a coordinate represents an *absolute* position on the globe. The constructor expects angles in degrees, however the factory `Coordinate3D<lla, T>::from_radians` and the accessors provide flexibility.

```cpp
//include header for geometry entities
#include <refx/geometry.h>

// A GPS coordinate for a location in Turin, Italy.
refx::Coordinate3D<refx::lla> turin_gps_fix(45.0703, 7.6869, 239.0);

// OR, Can pass the same coordinates in radians:
// refx::Coordinate3D<refx::lla> turin_gps_fix = 
//  refx::Coordinate3D<refx::lla>::from_radians(0.78662, 0.13416, 239.0);

// Access components in the default unit (degrees)
double lat_deg = turin_gps_fix.latitude();  // returns 45.0703
double lon_deg = turin_gps_fix.longitude(); // returns 7.6869

// Access the same components in radians for use with standard math libraries
double lat_rad = turin_gps_fix.latitude(refx::AngleUnit::Rad);
double lon_rad = turin_gps_fix.longitude(refx::AngleUnit::Rad);
```

### Vector3D: Representing Displacement

The `Vector3D<Frame, T>` class represents a displacement, such as a change in position, a velocity, an acceleration, or a force. It is a quantity that has both magnitude and direction.

#### Accessor Naming: The Core Semantic Distinction

The naming convention for `Vector3D` accessors is intentionally different for linear and non-linear frames to reinforce their mathematical meaning.

**Case 1: Linear Frames (NED, ENU, FRD, etc.)**
In a linear, Euclidean space, a vector's components directly correspond to a displacement along that frame's axes. Therefore, direct semantic names are intuitive and correct.

```cpp
// A velocity vector representing movement of 5 m/s North and 2 m/s West.
refx::Vector3D<refx::ned> vehicle_velocity(5.0, -2.0, 0.0);

// The accessor `.north()` correctly returns the "rate of change along the North axis".
double north_speed = vehicle_velocity.north(); // returns 5.0
double east_speed  = vehicle_velocity.east();  // returns -2.0
```


**Case 2: Non-Linear Frames (LLA, LLD, AER)**
In a non-linear space like a sphere, a "vector" does not represent a straight line. A `Vector3D<lla>` is a differential quantity or an error term: its components are `{Δlat, Δlon, Δalt}`. To prevent the dangerous semantic confusion of treating this as a geometric vector, all accessors are prefixed with `delta_`.

```cpp
// An error term from a GPS filter, representing a needed correction.
refx::Vector3D<refx::lla> gps_error(-0.0001, 0.0002, -1.5);

// The accessor `.delta_latitude()` correctly returns the "change in latitude".
double lat_correction_deg = gps_error.delta_latitude();  // returns -0.0001
double lon_correction_deg = gps_error.delta_longitude(); // returns 0.0002
double alt_correction_m   = gps_error.delta_altitude();   // returns -1.5
```

### Mathematical Operations

A key feature of the library is its strict enforcement of mathematically correct operations between coordinates and vectors.

| Operation | Result | Mathematical Justification |
| :--- | :--- | :--- |
| `Coordinate - Coordinate` | `Vector` | The difference between two coordinates is a displacement vector. |
| `Coordinate + Vector` | `Coordinate` | Adding a displacement to a coordinate yields a new coordinate. |
| `Vector + Vector` | `Vector` | Adding two displacements yields a net displacement. |
| `Coordinate + Coordinate` | Compile-Time Error | This operation is considered physically meaningless and is deleted. |


> [!NOTE]
> **On the Choice for Coordinate summation**
>
>The choice to disallow the summation of two Coordinates takes direct inspiration from the properties of positions in geographic frames: for instance, summming two GPS positions is generally considered meaningless, because they represent specific points on a spherical surface (the Earth): for example, adding the coordinates of the Empire State Building to those of the Eiffel Tower, although mathematically possible, would not yield to a location with any practical significance. Moreover, while mathematical operations can be performed on the underlying 3D Cartesian coordinates (x, y, z) derived from latitude and longitude, the result of summing two such points would again not correspond to a real-world location and would not be a valid GPS position. Instead of disabling the operator only for specific Coordinates defined on certain frames, with the resulting risk of introducing confusion in the library usage, we decided to extend this feature to *all* the Coordinates types, referred to *all* the reference frame.

### Rotations and Orientations

The library provides a clear separation between raw data containers for orientation and the main, frame-aware `Rotation` class.

#### Orientation Representations

**1. `UnitQuaternion<T>`:**
a four-dimensional number `(w, x, y, z)` that provides a complete, unambiguous representation of a 3D rotation. It is the internal gold standard for all rotation calculations within the library due to its efficiency and lack of singularities and gimbal lock.

**2. `EulerAngles<Seq, T>`:**
Euler angles represent an orientation as a sequence of three rotations. The library solves the ambiguity problem by making the rotation sequence a compile-time template parameter (e.g., `EulerAngles<refx::EulerSequence::ZYX, T>`).

#### The Geometric Operator: `Rotation<ToFrame, FromFrame, T>`

This is the primary, type-safe class for representing a 3D rotation. It is templated on the source and destination frames, enabling the compiler to validate entire chains of transformations.

```cpp
// 1. Create a frame-agnostic EulerAngles object (angles in radians)
// refx::YawPitchRoll<T> is an alias for refx::EulerAngles<refx::EulerSequence::ZYX, T>
refx::YawPitchRoll<double> ypr(refx::deg2rad(45.0), 0.0, 0.0); // 45-degree yaw

// 2. Create a type-safe Rotation object from the data container.
auto R_ned_frd = refx::Rotation<refx::ned, refx::frd>(ypr);

// 3. Use the Rotation object to transform a Vector3D
refx::Vector3D<refx::frd> forward_thrust(10.0, 0.0, 0.0);
refx::Vector3D<refx::ned> forward_thrust_ned = R_ned_frd * forward_thrust;

// Rotation works with Coordinate3D objects also
refx::Coordinate3D<refx::frd> p_body(10.0, 0.0, 0.0);
refx::Coordinate3D<refx::ned> p_ned = R_ned_frd * p_body;

//These are compile-time errors:
// refx::Vector3D<refx::flu> forward_thrust_flu(10.0, 0.0, 0.0);
// refx::Vector3D<refx::ned> forward_thrust_ned = R_ned_frd * forward_thrust_flu; //ERROR: source vector must be frd
// refx::Vector3D<refx::enu> forward_thrust_enu = R_ned_frd * forward_thrust; //ERROR: target vector must be ned
```

#### Frame-agnostic Intermediate Rotations

While refx's strict frame awareness is its core safety feature, some advanced algorithms require complex, intermediate calculations where frames are not yet finalized. Forcing frame safety on every single mathematical step can be cumbersome. To handle this, the library promotes a philosophy that separates raw **mathematical data containers** (`UnitQuaternion` or `EulerAngles<Seq, T>`) from its final geometric meaning, that is the **geometrically-aware types** (`Rotation`). This gives you the flexibility to perform complex calculations on a *scratchpad* before stamping the *final report* with a type-safe frame.

  * **The *Scratchpad***: refx provides the raw mathematical type `UnitQuaternion<T>` as your scratchpad. It is a pure data container with all the necessary mathematical operators (for composition, inversion, etc.), but it carry **no frame information**: you have complete freedom to multiply and manipulate it as needed for your calculations. To ease building complex rotations, the `UnitQuaternion` class provides three static factory functions for creating the fundamental building blocks:
      * **`from_rotation_x(T angle)`**
      * **`from_rotation_y(T angle)`**
      * **`from_rotation_z(T angle)`**

      Each one of these functions returns the quaternion corresponding to an elementary rotation around the X, Y or Z axis, respectively.

  * **The Final Report**: Once your complex, multi-step calculation is complete, you take the final result from your scratchpad (the final `UnitQuaternion`) and give it a clear, geometric meaning. You do this by constructing a `Rotation<ToFrame, FromFrame>` from it. From this point on, the compiler's safety checks are in full effect. Note that if the [Eigen support](#appendix-eigen3-integration) is enabled, a `Rotation<ToFrame, FromFrame>` can be contructed starting from an Eigen rotation matrix defined by the user, which augment the power of representation of the library.

This two-step process gives you the best of both worlds:
1.  **Flexibility**: You can perform complex, multi-step calculations with raw mathematical types without the constraints of the frame system getting in your way.
2.  **Safety**: Once the calculation is done, you wrap the result in a geometrically-aware type. This ensures that the result cannot be misused in the rest of your application, preserving the core safety benefits of the library.

> [!NOTE]
> **On the Rotation Convention**
> 
> The rotation convention used from the `from_rotation_*` factories transforms the coordinates of a point from a rotated frame back to the original reference frame. Therefore, the standard elementary rotation matrices for this convention are:
>- Rx(θ) = [[1, 0, 0], [0, cosθ, -sinθ], [0, sinθ, cosθ]]
>- Ry(θ) = [[cosθ, 0, sinθ], [0, 1, 0], [-sinθ, 0, cosθ]]
>- Rz(θ) = [[cosθ, -sinθ, 0], [sinθ, cosθ, 0], [0, 0, 1]]
> <table>
>   <tr>
>     <td><img src="imgs/Rotation_of_coordinates.svg" alt="Rotation of coordinates" width="700"/></td>
>     <td>
>       <b>Figure 1:</b> Example (positive) rotation around the z-axis. The resulting matrix Rz(theta) transforms vectors from the rotated frame (x′, y′) back to the original reference frame (x, y): P(x, y) = Rz(theta) P(x′, y′). For instance, the point x' = 1, y' = 0 in the frame (x,y) is [cos(theta), sin(theta)]
>     </td>
>   </tr>
> </table>

#### Practical Example: Geodetic to Local Rotation

Let's use a typical example in the inertial navigation context of creating the rotation from an ECEF (Earth-Centered, Earth-Fixed) frame to a local NED (North-East-Down) frame at a given latitude and longitude. This is typically done by composing three separate rotations: one around the y-axis of pi/2, one around the y-axis involving the latitude and another rotation around the z-axis involving the (negative) longitude, to align the resulting axes.

Here is how you would do it the refx way:

```cpp
#include <refx/geometry.h>
#include <cmath>

// A function to calculate the ECEF to NED rotation.
// It returns a fully type-safe object.
//
// The following computations are taken by Fig 3.1, page 50, and Eq. (3.12), page 57
// Book: "Applied Mathematics in Integrated Navigation Systems" 3rd Edition, Robert M. Rogers
refx::Rotation<refx::ned, refx::ecef> create_R_ned_from_ecef(double lat_rad, double lon_rad) {
    // --- Step 1: Frame-Agnostic Computation (The "Scratchpad") ---
    // We perform all intermediate calculations using the raw `UnitQuaternion` type.
    // These objects have no concept of "To" or "From" frames.

    // Rotation about the Y-axis by pi/2 (NED->Intermediate F1)
    refx::UnitQuaternion<double> q_pi_2 = refx::UnitQuaternion<double>::from_rotation_y(M_PI_2);
    // Rotation about the Y-axis by latitude (Intermediate F1 -> Intermediate F2)
    refx::UnitQuaternion<double> q_lat = refx::UnitQuaternion<double>::from_rotation_y(lat_rad);
    // Rotation about the Z-axis by -longitude (Intermediate F2 -> ECEF)
    refx::UnitQuaternion<double> q_lon = refx::UnitQuaternion<double>::from_rotation_z(-lon_rad);
    // Compose the rotations. This is a pure mathematical operation.
    refx::UnitQuaternion<double> final_q = q_pi_2 * q_lat * q_lon;

    // --- Step 2: Assign Geometric Meaning (The "Final Report") ---
    // Now that our calculation is complete, we construct the final, type-safe
    // ECEF-to-NED Rotation object from our result.
    return refx::Rotation<refx::ned, refx::ecef>(final_q);;
}
```


### Representing Full Pose (SE(3))

The `refx::Transformation` represents the complete **pose**—that is, the combined **position and orientation**—of one coordinate frame relative to another. Mathematically, this object models a **rigid body transformation** in 3D space, in the classical mathematical formulation of the **Special Euclidean group, SE(3)**, which is the standard mathematical tool for describing pose in robotics, 3D graphics, and mechanics. It's the primary tool for representing concepts like:

  * The complete pose of a vehicle in the navigation frame.
  * The extrinsic calibration of a sensor on a robot's body.
  * The relationship between a robot's end-effector and its base.

#### Structure and Members

The `Transformation` struct is composed of two key members:

1.  **`rotation`**: A `refx::Rotation<ToFrame, FromFrame, T>`

      * This member describes the **orientation** of the source frame (`FromFrame`)'s axes as viewed from the destination frame (`ToFrame`).

2.  **`translation`**: A `refx::Vector3D<ToFrame, T>`

      * This member describes the **displacement** of the `FromFrame`'s origin relative to the `ToFrame`'s origin; this translation vector is expressed in the coordinate system of the **`ToFrame`**.

#### Mathematical Operation

The `Transformation` class provides the pose transform operator, which accepts both `Vector3D` and `Coordinate3D`:
```cpp
VecType<FrameA, T> operator*(const Transformation<FrameA, FrameB, T>&, const VecType<FrameC, T>&);
```
which implements the mathematical formula:

`p_in_ToFrame = (rotation * p_in_FromFrame) + translation`

and the pose composition operator

```cpp
Transformation<FrameA, FrameC, T> operator*(const Transformation<FrameA, FrameB, T>&, const Transformation<FrameB, FrameC, T>&)
```

which implements the mathematical formula: 

`T_A_C = {T_A_B.rotation * T_B_C.rotation; T_A_B.rotation * T_B_C.translation + T_A_B.translation}`

#### Example: Defining a Vehicle's Pose

Here's how to construct a `Transformation` object to represent a vehicle's complete pose in a local-tangent `ned` frame.

```cpp
// 1. Define the vehicle's known state in the local-tangent (navigation) frame.
auto vehicle_position = refx::Coordinate3D<refx::ned>(500.0, 250.0, -10.0);
auto vehicle_orientation = refx::Rotation<refx::ned, refx::frd>(
    refx::YawPitchRoll<double>(refx::deg2rad(90.0), 0.0, 0.0) // Pointing East
);

// 2. Construct the Transformation object.
//    This single object now encapsulates the vehicle's entire pose.
auto T_ned_from_frd = refx::Transformation<refx::ned, refx::frd>{
    vehicle_orientation,
    // When assigned to the Transformation translation component,
    // `vehicle_position` is to be intended as a displacement vector
    // from the NED origin to the vehicle's location. In this way,
    // it can be consistently used as algebraic operator with Coordinates.
    vehicle_position.as_vector() //move to Vector3D
};

// 3. This Transformation object is now the key input for functions that
//    need to convert data from the vehicle's frame to the navigation frame,
//    such as `refx::frame_transform`.
```



## Geodetic Models

For high-accuracy navigation, simply treating the world as a flat plane or a perfect sphere isn't enough. refx provides a set of classes that model the Earth's true shape and physical properties, which are essential for tasks like gravity compensation in IMUs and accurate transformations between geodetic and Cartesian coordinates.

These models are layered, building from a basic geometric definition to a unified, easy-to-use context object.

### `ReferenceEllipsoid`

The foundation of all geodetic calculations is a model of the Earth's shape. The **`ReferenceEllipsoid`** class stores the defining constants of an oblate spheroid that approximates the Earth's geoid.

  * **Purpose**: Provides the fundamental geometric datum (size, shape, flattening) for all calculations.
  * **Standards**: The library includes pre-defined models for the most common standards:
      * **`ReferenceEllipsoidWGS84`**: The World Geodetic System 1984, which is the standard for GPS. This is the recommended default.
      * **`ReferenceEllipsoidGRS80`**: The Geodetic Reference System 1980, widely used in surveying and geodesy.

### `GravityModel`

Built upon a specific `ReferenceEllipsoid`, the **`GravityModel`** class provides a mathematical model of the Earth's theoretical "normal" gravity field. The model provides two modes for height compensation of the gravity norm, the **Free-Air** correction, reasonably accurate for altitudes typical of most robotics and aerospace applications, and the **Ellipsoidal** (Moritz, 1980) correction, which provides higher accuracy by accounting for the non-spherical ellipsoidal) shape of the Earth.
  * **Purpose**: To calculate the magnitude of the gravity vector at any given latitude and altitude.
  * **Primary Use Case**: **Gravity compensation** for inertial navigation systems (INS). An accelerometer measures both motion and gravity; this model allows you to subtract the gravity component to isolate the true acceleration.
  * **Standards**: Includes the **`GravityModelWGS84`** which corresponds to the WGS-84 ellipsoid.

### `EarthModel`

The **`EarthModel`** is the primary high-level class that users will typically interact with. It bundles the `ReferenceEllipsoid`, `GravityModel`, and an optional `MagneticFieldModel` into a single, convenient context object.

  * **Purpose**: To provide a single, authoritative source for all Earth-related physical constants and derived values.
  * **Features**: It offers simple methods to get critical navigation parameters like:
      * Local gravity at a specific position (`.gravity(...)`).
      * The Earth's rotation rate for Coriolis correction.
      * The meridian and normal radii of curvature, needed for converting velocities into changes in latitude and longitude.


### Usage Example: Gravity Computation

An example use for these models is to calculate the local gravity vector for correcting IMU data. The `EarthModel` makes this straightforward.

```cpp
//include header for models entities
#include <refx/models.h>
//include header for geometry entities
#include <refx/geometry.h>

// 1. Instantiate the pre-configured WGS-84 Earth model.
refx::EarthModelWGS84<double> earth_model;

// 2. Define the vehicle's current position using a geodetic coordinate.
auto current_position = refx::Coordinate3D<refx::lla>(44.39, 7.58, 534.0); // Cuneo, Italy

// 3. Get the precise magnitude of gravity at that location.
//    The model automatically accounts for both latitude and altitude.
double local_gravity = earth_model.gravity(current_position, 
                                           refx::GravityHeightCorrection::FreeAir);

// `local_gravity` can now be used to create a gravity vector and subtract it
// from a raw accelerometer reading to get the vehicle's true linear acceleration.
```



## Frame Conversions and Transformations

### `frame_cast`: Co-Origin Conversions

The `frame_cast<To>(from)` function is a lightweight, highly-efficient utility for simple conversions between frames that are **co-located** — that is, they share the same origin but have different axis conventions. The conversion mechanism is resolved entirely at compile time and employs C++ metaprogramming to deduce the correct transformation logic directly from the `DirectionalAxis` types that define the frames, via compile-time shuffling engine.

The mechanism works as follows:
1.  **Compile-Time Reflection**: The engine reflects on the template parameters of the source and target `DirectionalAxis` types (e.g., `<Forw, Right, Down>` for `axis_frd`).
2.  **Component Projection**: To construct the new vector, it determines the value for each of its components: it infers the component of the source vector (`x`, `y`, or `z`) which aligns with the direction of each of the target axis, and with what sign.

The entire logic is resolved by the compiler, which then generates the minimal, optimal machine code to perform the component shuffling and sign changes. This automatic mechanism works for all the 24 right-handed frame axis that the library defines, and so no additional work has to be made when an user adds new custom **Cartesian** reference frames, which necessarily are based on one of the standard axis semantics.

#### Common Use Cases

**1. Navigation Frame Conversion (NED ↔ ENU)**

The most common use case is converting between local-tangent frames, e.g. between **N**orth-**E**ast-**D**own (NED) and **E**ast-**N**orth-**U**p (ENU) frames, which are both centered on the same global origin.

```cpp
//include header for geometry entities
#include <refx/geometry.h>
//include header for frame transformations functions
#include <refx/transformations.h>

// A vector representing a velocity of 10 m/s North, -5 m/s East, and 2 m/s Down.
refx::Vector3D<refx::ned> velocity_ned(10.0, -5.0, 2.0);

// Cast it to the ENU frame. The library knows how to remap the axes.
auto velocity_enu = refx::frame_cast<refx::enu>(velocity_ned);

// The components are automatically shuffled and flipped:
// ENU.X = NED.Y  -> -5.0
// ENU.Y = NED.X  -> 10.0
// ENU.Z = -NED.Z -> -2.0
// Result: velocity_enu is [-5.0, 10.0, -2.0]
```

**2. Robotics Body Frame Conversion (FRD ↔ FLU)**

Different robotics domains use different conventions for the vehicle's body frame. `frame_cast` makes it trivial to switch between the aerospace standard (**F**orward-**R**ight-**D**own) and the common robotics/ROS standard (**F**orward-**L**eft-**U**p).

```cpp
// A force vector measured in the common robotics FLU frame.
// 20N forward, 5N left, 10N up.
refx::Vector3D<refx::flu> force_flu(20.0, 5.0, 10.0);

// Cast it to the aerospace FRD frame.
auto force_frd = refx::frame_cast<refx::frd>(force_flu);

// The components are remapped:
// FRD.X = FLU.X  -> 20.0
// FRD.Y = -FLU.Y -> -5.0
// FRD.Z = -FLU.Z -> -10.0
// Result: force_frd is [20.0, -5.0, -10.0]
```

#### Important Limitations

`frame_cast` will produce a **compile-time error** if you attempt to cast between frames that are not co-located or whose relationship is not a simple, fixed rotation.

```cpp
// The following will FAIL TO COMPILE because the frames have different origins
// and require a full transformation.

/*
// ERROR: Cannot cast from a body frame to a navigation frame.
// Their origins are different, and/or in general a simple axes shuffle is not sufficient for alignment.
// Use frame_transform.
refx::Vector3D<refx::frd> body_vec;
auto nav_vec = refx::frame_cast<refx::ned>(body_vec);

// ERROR: Cannot cast from a geodetic frame to a Cartesian one.
// This is a complex projection. Use frame_transform.
refx::Coordinate3D<refx::lla> geodetic_coord;
auto cartesian_coord = refx::frame_cast<refx::ned>(geodetic_coord);
*/
```

> [!NOTE]
>**more on the Category Safety Check**
>
>A key architectural decision enforced by `frame_cast` is the compile-time check on the frame category; this assertion is not arbitrary. The `frame_cast` API is designed exclusively for re-expressing a vector between frames that are **co-located** (i.e., share the same origin). This is a purely rotational or axis-shuffling operation. Frames within the same category, such as `body` or `sensor`, are assumed to be co-located by default, making `frame_cast` the appropriate tool.
>
>Transformations between different categories, such as from a `body` frame to a `sensor` frame, typically represent a physical mounting or calibration. These transformations almost always involve not just a rotation but also a translation due to a lever arm, which are mathematically represented by a full SE(3) Pose. For these more complex transformations, the correct tool is the **`Transformation<frameTo, frameFrom>`** class, which is designed to handle the complete pose (both orientation and position).
>

### `frame_transform`: Context-Dependent Transformations

The `frame_transform(...)` function is designed for complex transformations that are not simple axis shuffles. It is required whenever a transformation involves a change of origin or a conversion between fundamentally different types of coordinate systems (e.g., geodetic and Cartesian). 

A primary application of the `frame_transform` function is to perform transformations between frames of fundamentally different natures: from a `DirectionalAxis` (Cartesian) to a `SemanticAxis` (e.g., spherical or geodetic), and vice versa. Unlike the case of `frame_cast`, such transformation does not imply a simple axis shuffle, being the two reference frames described in a different geometric space. The function does not require any other context than the object to be converted:
```cpp
/// @brief Transforms a vector/coordinate between two co-located frames, with different axis semantics (axis_aer <-> cartesian)
template <typename frameTo, typename frameFrom, template <class, class> class VecType, typename T>
VecType<frameTo, T> frame_transform(const VecType<frameFrom, T>&);
```
Some additional context is necessary for the other overloadings of the `frame_transform(...)` function:
1.  **`EarthModel`**: An `EarthModel` (like `EarthModelWGS84<double>`) is required for any transformation involving geodetic coordinates like `lla`. The model provides the precise shape of the Earth (the WGS-84 ellipsoid), which is essential for correctly converting latitude and longitude into Cartesian `(x, y, z)` coordinates and vice-versa.
```cpp
/// @brief Transforms a coordinate between two **geocentric** frames (e.g., ECEF ↔ LLA).
template <typename frameTo, typename frameFrom, typename T>
Coordinate3D<frameTo, T> frame_transform(const Coordinate3D<frameFrom, T>&,
                                         const EarthModel<T>&);
```
2.  **`local_origin`**: When converting from a geodetic frame like `lla` to a local frame like `ned`, the function needs to know where on Earth the local tangent plane is anchored. This coordinate acts as the origin for the local frame.
```cpp
/// @brief Transforms (projects) a coordinate between a geocentric frame and a local tangent frame, and vice-versa.
template <typename frameTo, typename frameFrom, typename frameOrigin, typename T>
Coordinate3D<frameTo, T> frame_transform(const Coordinate3D<frameFrom, T>&,
                                         const Coordinate3D<frameOrigin, T>&,
                                         const EarthModel<T>&);
```
3.  **`Transformation` (Full Pose Context)**: This is just an alias API for the Transformation's product operator:  
`operator*(const Transformation<FrameA, FrameB, T>&, const VecType<FrameC, T>&)`.
```cpp
/// @brief Transforms a vector between two Cartesian frames using a full SE(3) pose
template <typename frameTo, typename frameFrom, template <class, class> class VecType, typename T>
VecType<frameTo, T> frame_transform(const Transformation<frameTo, frameFrom, T>&,
                                    const VecType<frameFrom, T>&);
```


#### Use Case 1: Global-to-Global Transformation (`lla` ↔ `ecef`)

This is the most direct type of transformation. It converts geodetic coordinates to the Earth-Centered, Earth-Fixed (ECEF) Cartesian frame. This is useful for getting a true 3D position in a global Cartesian system. The only context required is the `EarthModel`.

```cpp
// Required context: just the Earth model
refx::EarthModelWGS84<double> earth;

// A GPS coordinate for a location near the Mole Antonelliana in Turin
auto turin_lla = refx::Coordinate3D<refx::lla>(45.0688, 7.6872, 240.0);

// Transform the geodetic coordinate into the ECEF frame
auto turin_ecef = refx::frame_transform<refx::ecef>(turin_lla, earth);

// turin_ecef now holds the position as a 3D vector from the center of the Earth.
// We can also transform back.
auto turin_lla_again = refx::frame_transform<refx::lla>(turin_ecef, earth);
```

#### Use Case 2: Global-to-Local Transformation (`lla` → `ned`)

This is a common task in vehicle navigation: converting a global GPS fix into a local, flat coordinate system where motion can be more easily modeled. This requires both the `EarthModel` and the global coordinate of the local frame's origin.

```cpp
// Required context
refx::EarthModelWGS84<double> earth;

// Define the origin of our local NED frame. Let's place it at Piazza Castello.
auto local_origin_lla = refx::Coordinate3D<refx::lla>(45.0721, 7.6846, 238.0);

// A new GPS fix arrives for the car.
auto car_gps_fix = refx::Coordinate3D<refx::lla>(45.0725, 7.6850, 239.0);

// Transform the GPS fix into the local NED frame using the origin's full transform as context.
auto car_pos_in_ned = refx::frame_transform<refx::ned>(car_gps_fix, local_origin_lla, earth);

// car_pos_in_ned now holds the car's position in meters relative to Piazza Castello.
// e.g., [44.4, 27.5, -1.0], meaning 44.4m North, 27.5m East, and 1m up.
```



## Common Workflows for Developers

### Workflow 1: INS Gravity Compensation

An accelerometer measures specific force (motion + gravity). To get true acceleration, you must subtract the local gravity vector expressed in the proper (body/sensor) frame.

```cpp
//include header for geometry entities
#include <refx/geometry.h>
//include header for model entities
#include <refx/models.h>

// 1. Get the vehicle's current state
refx::Coordinate3D<refx::lla> current_position(45.07, 7.68, 250.0);
auto R_ned_frd = refx::Rotation<refx::ned, refx::frd>(
    refx::YawPitchRoll<double>(0.0, refx::deg2rad(-10.0), refx::deg2rad(5.0))
);

// 2. Instantiate the Earth model
refx::EarthModelWGS84<double> earth;

// 3. Calculate the gravity vector in the navigation (NED) frame
refx::Vector3D<refx::ned> gravity_vector_ned(0.0, 0.0, earth.gravity(current_position));

// 4. Rotate the gravity vector into the body frame to align with the sensor
// We assume that the body frame does coincide with the imu sensor frame
refx::Vector3D<refx::frd> gravity_vector_frd = R_ned_frd.inverse() * gravity_vector_ned;

// 5. This vector can now be subtracted from the accelerometer's measurement.
```


### Workflow 2: Calibrating Sensor Data with Extrinsics

A sensor mounted on a vehicle (like a camera, lidar, or radar) measures objects in its own coordinate system. To find the position of a detected object relative to the vehicle's center, it is needed to transform it into the vehicle's main reference frame (`frd`).

```cpp
//include header for geometry entities
#include <refx/geometry.h>
//include header for frame transformations functions
#include <refx/transformations.h>

// 1. Define the Extrinsic Calibration as a Transformation
// This camera is mounted 2m forward, 0.5m right, and 1.2m up from the vehicle's center.
// It is also pitched down by 15 degrees.

// The position of the camera's origin in the vehicle's frame (FRD)
auto camera_position_in_vehicle = refx::Vector3D<refx::frd>(2.0, 0.5, -1.2);

// The orientation of the camera relative to the vehicle's frame (FRD)
auto camera_orientation_in_vehicle = refx::Rotation<refx::frd, refx::camera>(
    refx::YawPitchRoll<double>(0.0, refx::deg2rad(-15.0), 0.0)
);

// The complete extrinsic calibration is a single Transformation object.
// This defines the pose of the camera relative to the vehicle.
auto T_vehicle_from_camera = refx::Transformation<refx::frd, refx::camera>{
    camera_orientation_in_vehicle,
    camera_position_in_vehicle
};

// 2. Get a Raw Sensor Measurement
// The camera detects an object 30 meters directly in front of itself.
// This is a coordinate in the camera's own reference frame.
auto detection_in_camera_frame = refx::Coordinate3D<refx::camera>(0.0, 0.0, 30.0);

// 3. Apply the Transformation to Calibrate the Measurement
// Use frame_transform to convert the raw detection into the vehicle's frame.
auto detection_in_vehicle_frame = refx::frame_transform(
    detection_in_camera_frame, // The raw measurement
    T_vehicle_from_camera      // The extrinsic calibration transform
);

// The result is the object's position relative to the vehicle's center,
// ready for use in path planning or sensor fusion.
// Expected output will account for the camera's offset and downward pitch.
```

### The refx Cookbook
The workflows above provide a quick look at how to solve common, isolated tasks with refx. They are the building blocks for creating robust navigation systems.

Real-world applications often involve combining these concepts to solve more complex, multi-step problems. For in-depth guides that tackle these challenges and explore the library's philosophy through practical examples, we highly recommend [**exploring our dedicated Cookbook**](docs/md/cookbook/main.md).

## Types Customization

While `refx` provides a set of standard frames, some of its power comes from its extensibility: you are encouraged to define custom frames that precisely describe your specific robot, sensors, and environment. The library's architecture makes defining custom frames a simple, declarative process. In particular for Cartesian frames, all the conversions are automatically compatible with all the new frame types that the user can define, meaning there is no need to write any custom conversion code.

### Step 1: Define Your Frame Struct

A custom frame is a simple `struct` that composes the three semantic primitives: a name, an axis system, and a frame tag. It's best practice to group your custom frames in a dedicated header file.

You need to define:

1.  A `static constexpr auto name`: A string identifier for debugging or interoperability.
2.  A `static constexpr FrameTag tag`: The conceptual category (`Body`, `Sensor`, etc.) the frame belongs to.
3.  A `using axis`: An alias to one of the 24 pre-defined `DirectionalAxis` types (e.g., `axis_frd`, `axis_rfu`) that matches the physical orientation of your frame's axes.

Let's define a frame for a `laser_scanner` sensor. Assume its axes are oriented according to the `axis_flu` (Forward-Left-Up) convention and it's part of the `Sensor` category.

**`my_robot_frames.h`**

```cpp
// Include 'developer-level' headers
#include <refx/frames/axis.h>
#include <refx/frames/tags.h>

namespace my_robot {

// A custom frame for a 2D laser scanner.
struct laser_scanner {
    static constexpr auto name = "laser_scanner";
    using axis = refx::axis_flu;  // Use a standard DirectionalAxis
    static constexpr refx::FrameTag tag = refx::FrameTag::Sensor;
};

}  // namespace my_robot
```

**That's it.** Your `laser_scanner` frame is now a first-class citizen in the `refx` ecosystem. Because its axis system is a known `DirectionalAxis`, the library's `frame_cast` engine can automatically deduce how to convert it to and from any other Cartesian frame in the same category.

### Step 2 (Optional): Specialize `Vector3D` for Semantic Accessors

By default, you can create vectors in your new frame using the standard `.x()`, `.y()`, and `.z()` accessors, `refx::Vector3D<my_robot::laser_scanner> vec;`. However, for improved code clarity, you can provide an optional `Vector3D` specialization to add meaningful accessors that correspond to your frame's axes.

**`my_robot_vectors.h`**

```cpp
#include <refx/geometry/vector.h>
#include "my_robot_frames.h"

namespace refx {  // Specialization must be done in the `refex` namespace
/**
 * @brief Specialization of Vector3D for our custom laser_scanner frame.
 * @details Provides semantic accessors matching the underlying axis_flu system.
 */
template <typename T>
struct Vector3D<my_robot::laser_scanner, T> : public internal::VectorContainer3D<T> {
    // Standard boilerplate for a specialization
    using frame = my_robot::laser_scanner;
    using internal::VectorContainer3D<T>::VectorContainer3D;

    // Optional constructor for convenience
    Vector3D(T forward = 0, T left = 0, T up = 0)
        : internal::VectorContainer3D<T>({forward, left, up}) {}

    // --- Semantic Accessors ---
    T& laser_front() { return this->x(); }
    T& laser_side() { return this->y(); }
    T& laser_up() { return this->z(); }

    T laser_front() const { return this->x(); }
    T laser_side() const { return this->y(); }
    T laser_up() const { return this->z(); }
};

}  // namespace refx
```

### Putting It All Together: Using Your Custom Frame

With the frame defined, it integrates with the entire library. The compile-time shuffling engine automatically handles `frame_cast`.

```cpp
#include "my_robot_frames.h"
#include "my_robot_vectors.h" // For semantic accessors
#include <refx/geometry.h>

int main() {    
    // 1. Create a vector in our custom frame.
    auto vec_laser = refx::Vector3D<my_robot::laser_scanner>(10.0, 5.0, 2.0);
    // double fwd_distance = vec_laser.x(); // Works, but...
    double fwd_distance = vec_laser.laser_front();  // ...is much clearer with the specialization.
    // Expected result: 10
    std::cout << "Fwd distance: " << fwd_distance << std::endl;

    refx::Rotation<refx::frd, my_robot::laser_scanner> R_laser_to_body(
        refx::YawPitchRoll<double>(M_PI / 2.0, 0.0, 0.0));

    // For example you can rotate from the sensor frame to the body frame, ensuring type-safety
    // the resulting vector is still in the origin of the sensor frame, but aligned with the body
    // axis
    Vector3D<frd> vec_laser_in_body = R_laser_to_body * vec_laser;
    // Then you can add a displacement vector in the body frame, ensuring type-safety, for example
    // the translation calibration
    refx::Vector3D<refx::frd> T_laser_calib_body(2.0, 0.8, 1.5);
    // THIS RESULTS IN A COMPILE ERROR!
    // refx::Vector3D<refx::frd> vec_body = vec_laser + T_laser_calib_body;

    // The resulting (calibrated) mesaurement from laser to body
    // now this sum is safe to do
    refx::Vector3D<refx::frd> vec_body = vec_laser_in_body + T_laser_calib_body;

    // Expected result: [-3, 10.8, 3.5]
    std::cout << "Vector in body frame (frd): " << vec_body << std::endl;

    // 2. Use `frame_cast` to convert to a standard library frame.
    // This works automatically with ZERO custom conversion code!
    // The engine deduces the transform from axis_flu to axis_rdf.
    // remember that with such function, you can transfrom between
    // frames that belongs to the same FrameTag (Body, Sensor, ...)
    auto vec_camera = refx::frame_cast<refx::camera>(vec_laser);

    // Expected result: [-5, -2, 10]
    std::cout << "Vector in camera frame (frd): " << vec_camera << std::endl;

    // 3. Use a Transformation object, to calibrate the sensor measuement in one shot.
    // Define the pose of the laser scanner relative to the vehicle's body.
    auto T_body_from_laser = Transformation<refx::frd, my_robot::laser_scanner>(
        // Assign the rotation calibration
        R_laser_to_body,
        // And the translation calibration
        T_laser_calib_body);

    // The custom frame is a first-class citizen in all type-safe operations.
    vec_body = T_body_from_laser * vec_laser;

    // Expected result: [-3, 10.8, 3.5]
    std::cout << "Vector in body frame (via Transform): " << vec_body << std::endl;
}
```

---

## Appendix: Eigen3 Integration

  * **Version**: latest stable (3.4.0)
  * **Detection**: Enabled automatically if `find_package(Eigen3)` succeeds.
  * **Functionality**: If found, the `REFX_ENABLE_EIGEN_SUPPORT` preprocessor macro is defined. This activates:
    * **Constructors** to create refx types directly from Eigen vectors, quaternions, and rotation matrices.
    * `.to_eigen()`, `.to_eigen_quat()` and `.to_eigen_matrix()` methods to convert refx types into their Eigen counterparts.

## Appendix: Default Frame Tags Reference

| Tag | Description |
| :--- | :--- |
| **Geocentric / Global Frames** | |
| `ecef` | Earth-Centered, Earth-Fixed. A global Cartesian frame. |
| `lla` | Geodetic Latitude, Longitude, Altitude. |
| `lld` | Geodetic Latitude, Longitude, Down. |
| **Local Tangent Frames** | |
| `ned` | North-East-Down. A right-handed local Cartesian frame. |
| `enu` | East-North-Up. A right-handed local Cartesian frame. |
| `nwu` | North-West-Up. A right-handed local Cartesian frame. |
| `wa_generic` | Generic Wander Azimuth. A generic local Cartesian frame, templated wrt axis orientation. |
| `wa_ned` | Wander Azimuth. A standard generic local Cartesian frame (underlying NED). |
| `wa` | Alias for `wa_ned`. |
| `wa_enu` | Wander Azimuth. A generic local Cartesian frame (underlying ENU). |
| `aer` | Azimuth, Elevation, Range. A local spherical frame. |
| **Body-Fixed Frames** | |
| `frd` | Forward-Right-Down. A right-handed body frame (aerospace standard). |
| `flu` | Forward-Left-Up. A right-handed body frame (robotics standard, ROS). |
| `rfu` | Right-Forward-Up. A right-handed body frame (vision standard). |
| `ssh` | Surge-Sway-Heave. A velocity frame for marine robotics. |
| **Sensor-Fixed Frames** | |
| `imu` | [Forward-Left-Up]. A right-handed frame usually related to Inertial Sensor suites. |
| `camera` | [Right-Down-Forward]. A right-handed camera (pinhole) frame. |

## Appendix: Default Frame Axis Reference

### Directional Axis (Cartesian)
| Axis | Description |
| :--- | :--- |
| **X: Forward or Backward** | |
| `axis_frd` | `DirectionalAxis<AxisDirection::Forw, AxisDirection::Right, AxisDirection::Down>`|
| `axis_flu` | `DirectionalAxis<AxisDirection::Forw, AxisDirection::Left, AxisDirection::Up>`|
| `axis_fur` | `DirectionalAxis<AxisDirection::Forw, AxisDirection::Up, AxisDirection::Right>`|
| `axis_fdl` | `DirectionalAxis<AxisDirection::Forw, AxisDirection::Down, AxisDirection::Left>`|
| `axis_bru` | `DirectionalAxis<AxisDirection::Back, AxisDirection::Right, AxisDirection::Up>`|
| `axis_bld` | `DirectionalAxis<AxisDirection::Back, AxisDirection::Left, AxisDirection::Down>`|
| `axis_bul` | `DirectionalAxis<AxisDirection::Back, AxisDirection::Up, AxisDirection::Left>`|
| `axis_bdr` | `DirectionalAxis<AxisDirection::Back, AxisDirection::Down, AxisDirection::Right>`|
| **X: Right or Left** | |
| `axis_rfu` | `DirectionalAxis<AxisDirection::Right, AxisDirection::Forw, AxisDirection::Up>`|
| `axis_rbd` | `DirectionalAxis<AxisDirection::Right, AxisDirection::Back, AxisDirection::Down>`|
| `axis_rub` | `DirectionalAxis<AxisDirection::Right, AxisDirection::Up, AxisDirection::Back>`|
| `axis_rdf` | `DirectionalAxis<AxisDirection::Right, AxisDirection::Down, AxisDirection::Forw>`|
| `axis_lfd` | `DirectionalAxis<AxisDirection::Left, AxisDirection::Forw, AxisDirection::Down>`|
| `axis_lbu` | `DirectionalAxis<AxisDirection::Left, AxisDirection::Back, AxisDirection::Up>`|
| `axis_luf` | `DirectionalAxis<AxisDirection::Left, AxisDirection::Up, AxisDirection::Forw>`|
| `axis_ldb` | `DirectionalAxis<AxisDirection::Left, AxisDirection::Down, AxisDirection::Back>`|
| **X: Up or Down** | |
| `axis_urf` | `DirectionalAxis<AxisDirection::Up, AxisDirection::Right, AxisDirection::Forw>`|
| `axis_ulb` | `DirectionalAxis<AxisDirection::Up, AxisDirection::Left, AxisDirection::Back>`|
| `axis_ufl` | `DirectionalAxis<AxisDirection::Up, AxisDirection::Forw, AxisDirection::Left>`|
| `axis_ubr` | `DirectionalAxis<AxisDirection::Up, AxisDirection::Back, AxisDirection::Right>`|
| `axis_drb` | `DirectionalAxis<AxisDirection::Down, AxisDirection::Right, AxisDirection::Back>`|
| `axis_dlf` | `DirectionalAxis<AxisDirection::Down, AxisDirection::Left, AxisDirection::Forw>`|
| `axis_dfr` | `DirectionalAxis<AxisDirection::Down, AxisDirection::Forw, AxisDirection::Right>`|
| `axis_dbl` | `DirectionalAxis<AxisDirection::Down, AxisDirection::Back, AxisDirection::Left>`|

### Semantic Axis (Mixed Spherical/Linear)

| Axis | Description |
| :--- | :--- |
| `axis_lla` | `SemanticAxis<AxisSemantic::Latitude, AxisSemantic::Longitude, AxisSemantic::Altitude>`|
| `axis_lld` | `SemanticAxis<AxisSemantic::Latitude, AxisSemantic::Longitude, AxisSemantic::Down>`|
| `axis_aer` | `SemanticAxis<AxisSemantic::Azimuth, AxisSemantic::Elevation, AxisSemantic::Range>`|

---

[^ Return to ToC](#documentation)
