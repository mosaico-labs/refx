← [Back to Cookbook](main.md)

---

## From Aerodynamic Angles to Aircraft Pose ✈️

This recipe demonstrates how to calculate an aircraft's complete orientation in a global frame by composing multiple, intermediate rotations derived from aerodynamic angles. It showcases the "scratchpad" philosophy for handling complex calculations.

### The Goal

We know an aircraft's orientation relative to its airflow, defined by its **angle of attack (α)** and **sideslip angle (β)**. We also know the orientation of the airflow (the "wind") relative to the world. Our goal is to find the aircraft's final, absolute orientation (its "pose") in the global **`ned`** frame.

### The Challenge

The transformation from the aircraft's body frame to the world frame is not direct. It involves intermediate coordinate systems like the "stability frame." These intermediate rotations are pure mathematical steps based on aerodynamic angles. Forcing a `frameo::Rotation<To, From>` type onto each intermediate step is cumbersome and conceptually awkward.

This is a perfect scenario for using the frame-agnostic **`frameo::UnitQuaternion`** as a computational scratchpad, only creating the final, type-safe **`frameo::Rotation`** at the very end.

### Ingredients

  * `frameo::UnitQuaternion`: Our frame-agnostic "scratchpad" for intermediate math.
  * `frameo::Rotation`: Our type-safe "final report" for the resulting orientation.
  * **Frames**:
      * `frameo::ned`: The global **N**orth-**E**ast-**D**own navigation frame.
      * `frameo::frd`: The aircraft's body-fixed **F**orward-**R**ight-**D**own frame.
      * `wind`: A temporary frame tag for the intermediate "wind frame."

-----

### Step-by-Step Instructions

#### Step 1: The "Scratchpad" - Define Intermediate Rotations

First, we calculate the individual rotations from the aerodynamic angles. We use `frameo::UnitQuaternion` for these steps, as they are pure mathematical operations without a final, fixed geometric context.

  * **Sideslip Angle (β)**: This is the rotation from the body frame to the "stability frame," performed around the body's Z-axis (Down).
  * **Angle of Attack (α)**: This is the rotation from the stability frame to the "wind frame," performed around the stability frame's Y-axis (Right).

<!-- end list -->

```cpp
#include <frameo/frameo.h>
#include <cmath>

// Known aerodynamic angles
double angle_of_attack_alpha = frameo::deg2rad(5.0); // 5 degrees alpha
double sideslip_angle_beta = frameo::deg2rad(-2.0); // -2 degrees beta

// --- Scratchpad Computations ---

// Create a frame-agnostic quaternion for the rotation from body to stability frame.
// This is a rotation around the Z-axis by the sideslip angle.
auto q_stability_from_body = frameo::UnitQuaternion<double>::from_rotation_z(sideslip_angle_beta);

// Create a frame-agnostic quaternion for the rotation from stability to wind frame.
// This is a rotation around the Y-axis by the angle of attack.
auto q_wind_from_stability = frameo::UnitQuaternion<double>::from_rotation_y(angle_of_attack_alpha);
```

#### Step 2: Compose the Frame-Agnostic Rotations

Still working on our scratchpad, we multiply these quaternions to get the total rotation from the aircraft's body to the wind frame.

```cpp
// This is a pure mathematical composition, free of any frame constraints.
frameo::UnitQuaternion<double> q_wind_from_body = q_wind_from_stability * q_stability_from_body;
```

#### Step 3: Define the Known Geometric Rotation

Now, we introduce the part of the problem that has a clear, geometric meaning. We know the orientation of the wind itself relative to the world. This is a known relationship between two well-defined frames, so we represent it with a type-safe `frameo::Rotation`.

```cpp
// Let's say the wind is coming from the North (0-degree heading) and the aircraft
// has a bank angle of 10 degrees relative to the wind.
// This is our known, type-safe "anchor" to the real world.
auto R_ned_from_wind = frameo::Rotation<frameo::ned, frameo::wind>::from_euler_angles(
    frameo::YawPitchRoll<double>(frameo::deg2rad(10.0), 0.0, 0.0)
);
```

#### Step 4: Final Composition and "Assigning Meaning"

This is the final step where we leave the scratchpad and publish our final report. We take our calculated `q_wind_from_body` quaternion, give it a geometric meaning, and then compose it with our other type-safe `Rotation`.

```cpp
// 1. Convert the frame-agnostic quaternion into a type-safe Rotation.
// We are now formally stating that this rotation goes FROM the body (frd) TO the wind frame.
auto R_wind_from_frd = frameo::Rotation<frameo::wind, frameo::frd>(q_wind_from_body);

// 2. Perform the final, type-safe composition.
// The compiler checks that the inner frames (`wind`) match.
auto R_ned_from_frd = R_ned_from_wind * R_wind_from_frd;

// This is our final result: the complete orientation of the aircraft in the world.
// It can now be used safely throughout the rest of the application.
std::cout << "Final Aircraft Orientation (NED from FRD): " << R_ned_from_frd << std::endl;
```

-----

### The Main Takeaway

This recipe demonstrates the power of `frameo`'s two-level design:

1.  **Flexibility**: Use raw data containers like `frameo::UnitQuaternion` as a "scratchpad" for complex, multi-step, or frame-agnostic calculations.
2.  **Safety**: When the calculation is complete, construct a final, geometrically-aware type like `frameo::Rotation` or `frameo::Transformation`. This "stamps" the result with a clear meaning and engages the compiler's type-safety checks, protecting the rest of your application from misuse.