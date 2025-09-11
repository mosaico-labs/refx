#ifndef _REFX_FRAMES_TAGS_
#define _REFX_FRAMES_TAGS_

/**
 * @file tags.h
 * @brief Defines the compile-time frame-tags for all refx coordinate frames.
 *
 * @details This header contains a Enum that serve as "tags" for different families
 * of coordinate frames. By embedding these frame tags within a frame's definition
 * (e.g., `struct ned { using tag = FrameTag::LocalTangent; };`),
 * we can use C++ type traits to perform powerful compile-time checks.
 *
 * This architectural pattern is the foundation for ensuring that operations
 * are only performed between frames of the same conceptual type, preventing a
 * wide range of common errors in robotics and navigation software.
 */

namespace refx {

enum class FrameTag {
    /**
     * @brief frame-tag for geocentric (Earth-centered) coordinate frames.
     *
     * @details Geocentric frames have their origin at the center of mass of the Earth.
     * They are used for absolute, planet-wide positioning.
     *
     * Examples of frames in this frame-tag include:
     * - **`ecef`**: Earth-Centered, Earth-Fixed
     * - **`lla`**: Geodetic Latitude, Longitude, Altitude
     * - ...
     */
    Geocentric,

    /**
     * @brief frame-tag for local tangent plane coordinate frames.
     *
     * @details Local tangent frames are Cartesian systems anchored to a specific
     * point on the Earth's surface or on a moving platform. They represent a
     * "flat Earth" approximation that is valid for local-area navigation.
     * All frames within this frame-tag are co-located (share the same origin).
     *
     * Examples of frames in this frame-tag include:
     * - **`ned`**: North-East-Down
     * - **`enu`**: East-North-Up
     * - **`aer`**: Azimurh-Elevation-Range
     * - ...
     */
    LocalTangent,

    /**
     * @brief frame-tag for frames rigidly attached to a moving body.
     *
     * @details Body-fixed frames have their origin at a reference point on a vehicle
     * or object (e.g., its center of gravity) and move and rotate with it.
     *
     * Examples of frames in this frame-tag include:
     * - **`frd`**: Forward-Right-Down (aerospace standard)
     * - **`flu`**: Forward-Left-Up (robotics/ROS standard)
     * - ...
     */
    Body,

    /**
     * @brief frame-tag for frames rigidly attached to a sensor mounted on a body.
     *
     * @details Sensor frames have their origin at a specific point on a sensor
     * (e.g., the optical center of a camera or the measurement origin of a lidar).
     * They are a specialized type of body-fixed frame. The relationship between a
     * `sensor` frame and a `body` frame is defined by an extrinsic calibration
     * `Transformation`.
     *
     * Examples of user-defined frames in this frame-tag could include:
     * - `imu`
     * - `camera`
     * - ...
     */
    Sensor
};

}  // namespace refx

#endif /* _REFX_FRAMES_TAGS_ */
