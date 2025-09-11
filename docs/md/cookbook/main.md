← [Back to README](../../../README.md)

---

# The refx Cookbook

Welcome to the refx cookbook! This is a collection of practical, real-world recipes designed to help you solve common problems in robotics and navigation. While the main documentation explains *what* each component does, this cookbook shows you *how* to put them together to build robust, safe, and readable software.

Each recipe is more than just a code snippet; it's a handout in the refx philosophy of "thinking in frames." You'll learn how to leverage the refx type system to prevent bugs before they happen and write code that is a clear reflection of the underlying mathematics.

---

## Table of Contents

Here you will find a list of available recipes, each tackling a specific challenge. The list is constantly updated.

### 1. [From Relative Sensor Data to Global Position](relative_to_global.md)
* **Goal**: Take a sensor detection from a vehicle's local frame and find its absolute position in a global navigation frame.
* **Concepts Covered**: `Coordinate3D`, `frame_transform`.

### 2\. [IMU Measurements Compensation for Navigation Filters](imu_comp.md)

  * **Goal**: Correct raw IMU (accelerometer and gyroscope) measurements for the effects of gravity and the Earth's rotation (Coriolis effect) to prepare them for integration in a navigation filter.
  * **Concepts Covered**: Using high-fidelity `EarthModel`s, transforming physical vectors (`gravity`, `earth_rate`) between frames, practical application of `Rotation` and `Vector` for sensor fusion.
