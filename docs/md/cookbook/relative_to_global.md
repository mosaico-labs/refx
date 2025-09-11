← [Back to Cookbook](main.md)

---

## From Relative Sensor Data to Global Position

This recipe shows how to take a sensor measurement from a vehicle's body-fixed frame and correctly determine its absolute position in a global navigation frame. This is a fundamental task in robotics, autonomous driving, and aviation.

### The Goal

An ego-vehicle, equipped with a sensor like a radar, detects another vehicle (a target). We know our own absolute position and orientation in the world. Our goal is to find the **absolute world position of the target vehicle**.

-----

### Ingredients

  * `refx::Coordinate3D`: To represent specific points in a frame.
  * `refx::Rotation`: To represent orientation.
  * `refx::frame_transform`: The function that correctly changes a coordinate's frame of reference.
  * **Frames**:
      * `refx::lla`: A global geocentric frame for absolute localization.
      * `refx::ned`: A local **N**orth-**E**ast-**D**own frame as intermediate navigation frame.
      * `refx::frd`: A body-fixed **F**orward-**R**ight-**D**own frame attached to our ego-vehicle.

-----

### Step-by-Step Instructions

#### Step 1: Define the Ego-Vehicle's State

First, we need to know our own state in the world. This consists of our position and orientation.

  * Our **absolute position** is a specific point in the `lla` frame, so it's a `Coordinate3D<lla>`.
  * Our **orientation** describes how our `frd` frame is rotated relative to the local `ned` frame, so it's a `Rotation<ned, frd>`.

<!-- end list -->

```cpp
#include <refx/geometry.h>

using namespace refx;

// Our ego-vehicle is at a known global position.
auto ego_position_global = Coordinate3D<lla>(44.3040729, 11.9530427);

// It's pointing -45 degrees off the North (a -45-degree yaw).
auto ego_orientation_global =
    Rotation<ned, frd>(YawPitchRoll<double>(deg2rad(-45.0), 0.0, 0.0));
```

#### Step 2: Define the Sensor Measurement

Our radar provides the position of the target vehicle; this measurement is relative to the **origin of our ego-vehicle's `frd` frame**. It answers the question, "Where is the target *relative to me*?" Therefore, it is a `Coordinate3D<frd>`.

```cpp
// The radar detects the target 75 meters directly in front of us
// and 10 meters to the left. 
// In the FRD frame, "left" is negative on the "Right" axis.
auto target_position_relative = Coordinate3D<frd>(75.0, -10.0, 0.0);
```

#### Step 3: Convert and Transform

To obtain the absolute (geodetic) position of the target vehicle detected by the radar, we must perform a two-step transformation: first, from the sensor's frame to the local navigation frame, and second, from the local navigation frame to the global geodetic frame.

```cpp
// This transformation finds the position of the target vehicle in the ned frame attached to our ego-vehicle
Coordinate3D<ned> target_position_relative_ned = ego_orientation_global * target_position_relative;
```

Then this coordinate expressed in a local tangent reference frame **rigidly attached to our ego-vehicle** can be used in a `frame_transform`, by employing our global position as origin. What we need, is an `EarthModel`:

```cpp
// This transform fully defines the target vehicle global position.
Coordinate3D<lla> target_position_global = frame_transform<lla>(
    target_position_relative_ned, ego_position_global, EarthModelWGS84<double>());

// We can now use target_position_global for navigation, tracking, etc.
//expected: [44.3045, 11.9523, 0]
std::cout << "Target's Absolute Position (LLA): " << target_position_global << std::endl;

```

-----

### The Main Takeaway

This recipe demonstrates the core philosophy of refx:

  * A **`Coordinate`** is a point relative to a specific frame's origin. The radar measurement is a coordinate in the `frd` frame.
  * To find a point's coordinates in a different frame, you must apply a **`frame_transform`**. This avoids ambiguity and prevents common mathematical errors, like adding a relative position to a global one without rotating it first.