#ifndef _REFX_FRAMES_FRAMES_
#define _REFX_FRAMES_FRAMES_

#include "axis.h"
#include "tags.h"
/**
 * @file frames.h
 * @brief Defines the tag structs used to identify coordinate reference frames.
 * @details This file is the central registry for all coordinate frames supported by
 * the library. It contains a collection of empty structs, each representing a unique
 * reference frame.
 *
 * These tags are the base types of the library's compile-time safety. By using
 * them as template parameters for types like `Vector3D` and `Coordinate3D`, the
 * C++ compiler can enforce that operations are only performed between objects
 * belonging to the same, compatible frame. This prevents a wide class of common
 * and difficult-to-debug errors in robotics and navigation software.
 *
 * The frames are organized into logical groups: geocentric, local tangent,
 * body(sensor)-fixed.
 */

namespace refx {

/*==========================================================================*/
/*====================== Geocentric / Global Frames ========================*/
/*==========================================================================*/

/**
 * @brief **Earth-Centered, Earth-Fixed (ECEF)** reference frame.
 * @details A global, right-handed Cartesian frame whose origin is at the Earth's
 * center of mass. It rotates with the Earth.
 * - **+X-axis**: Points to the intersection of the equator and the prime meridian.
 * - **+Y-axis**: Points to 90° East longitude on the equator.
 * - **+Z-axis**: Points to the North Pole, aligned with the Earth's rotation axis.
 */
struct ecef {
    static constexpr auto name = "ecef";
    using axis = axis_flu;
    static constexpr FrameTag tag = FrameTag::Geocentric;
};

/**
 * @brief Geodetic **Latitude, Longitude, and Altitude (LLA)** frame.
 * @details A geographic coordinate system for specifying a point on or near the
 * Earth's surface using angular coordinates and a height.
 * - **Latitude**: Angle from the equatorial plane (-90° to +90°).
 * - **Longitude**: Angle from the prime meridian (-180° to +180°).
 * - **Altitude**: Height in meters above a reference ellipsoid (e.g., WGS84).
 * @note This is not a Cartesian frame. Arithmetic on `Coordinate3D<lla>` is
 * specialized to handle angular wrap-around.
 */
struct lla {
    static constexpr auto name = "lla";
    using axis = axis_lla;
    static constexpr FrameTag tag = FrameTag::Geocentric;
};

/**
 * @brief Geodetic **Latitude, Longitude, and Down (LLD)** frame.
 * @details A variant of LLA where the third component is a positive-downward
 * distance (depth) from the reference ellipsoid.
 * @note This frame is often more convenient than LLA when working with systems
 * that use a North-East-Down (NED) local frame.
 * @note This is not a Cartesian frame. Arithmetic on `Coordinate3D<lld>` is
 * specialized to handle angular wrap-around.
 */
struct lld {
    static constexpr auto name = "lld";
    using axis = axis_lld;
    static constexpr FrameTag tag = FrameTag::Geocentric;
};

/*==========================================================================*/
/*================== Local Tangent Frames (LTP) ============================*/
/*==========================================================================*/

/**
 * @brief **North-East-Down (NED)** local tangent frame.
 * @details A right-handed Cartesian frame defined on a plane tangent to the
 * Earth's surface at a specific origin.
 * - **+X-axis**: Points to geographic (true) North.
 * - **+Y-axis**: Points to geographic East.
 * - **+Z-axis**: Points Down, towards the Earth's center (parallel to gravity).
 */
struct ned {
    static constexpr auto name = "ned";
    using axis = axis_frd;
    static constexpr FrameTag tag = FrameTag::LocalTangent;
};

/**
 * @brief **East-North-Up (ENU)** local tangent frame.
 * @details A right-handed Cartesian frame defined on a plane tangent to the
 * Earth's surface at a specific origin.
 * - **+X-axis**: Points to geographic East.
 * - **+Y-axis**: Points to geographic North.
 * - **+Z-axis**: Points Up, away from the Earth's center (opposite to gravity).
 */
struct enu {
    static constexpr auto name = "enu";
    using axis = axis_rfu;
    static constexpr FrameTag tag = FrameTag::LocalTangent;
};

/**
 * @brief **North-West-Up (NWU)** local tangent frame.
 * @details A right-handed Cartesian frame defined on a plane tangent to the
 * Earth's surface.
 * - **+X-axis**: Points to geographic North.
 * - **+Y-axis**: Points to geographic West.
 * - **+Z-axis**: Points Up.
 */
struct nwu {
    static constexpr auto name = "nwu";
    using axis = axis_flu;
    static constexpr FrameTag tag = FrameTag::LocalTangent;
};

/**
 * @brief A generic template for creating Wander-Azimuth (WA) frame types.
 * @details This class template acts as a factory for Wander-Azimuth frames.
 * A WA frame is a local, right-handed Cartesian frame whose orientation is not
 * necessarily aligned with geographic North.
 *
 * The specific axis orientation is not hard-coded but is provided as a template parameter
 * (`Axis`). This allows for the creation of different WA frame variants that are rotationally
 * consistent with other primary local tangent frames like `ned` or `enu`.
 *
 * @tparam Axis The specific `DirectionalAxis` configuration for this WA frame.
 */
template <typename Axis>
struct wa_generic {
    static constexpr auto name = "wa";
    using axis = Axis;
    static constexpr FrameTag tag = FrameTag::LocalTangent;
};

/**
 * @brief The standard Wander-Azimuth frame related with an underlying navigation frame
 * in the NED convention.
 * @details This type alias defines a Wander-Azimuth frame whose axis system
 * is `axis_frd` (Forward-Right-Down). This makes it rotationally equivalent
 * and directly compatible with the standard `ned` frame. It should be used in
 * systems where NED is the primary local tangent frame.
 * @note Aligning this frame with the underlying local-tangent frame cannot be done
 * using the `frame_cast` API, since the relative orientation is generally time-varying
 * and dependent on the Wander angle `alpha`
 */
using wa_ned = wa_generic<axis_frd>;
/**
 * @brief The standard alias for Wander-Azimuth frame (NED related).
 */
using wa = wa_ned;

/**
 * @brief A Wander-Azimuth frame related with an underlying navigation frame
 * in the ENU convention.
 * @details This type alias defines a Wander-Azimuth frame whose axis system
 * is `axis_rfu` (Right-Forward-Up). This makes it rotationally equivalent
 * and directly compatible with the standard `enu` frame. It should be used in
 * systems where ENU is the primary local tangent frame.
 * @note Aligning this frame with the underlying local-tangent frame cannot be done
 * using the `frame_cast` API, since the relative orientation is generally time-varying
 * and dependent on the Wander angle `alpha`
 */
using wa_enu = wa_generic<axis_rfu>;

/**
 * @brief **Azimuth, Elevation, Range (AER)** spherical frame.
 * @details A spherical coordinate system for specifying a point relative to a local origin.
 * - **Azimuth**: Horizontal angle, typically clockwise from North.
 * - **Elevation**: Vertical angle from the horizontal plane.
 * - **Range**: Straight-line distance from the origin to the point.
 */
struct aer {
    static constexpr auto name = "aer";
    using axis = axis_aer;
    static constexpr FrameTag tag = FrameTag::LocalTangent;
};

/*==========================================================================*/
/*====================== Body / Vehicle-Fixed Frames =======================*/
/*==========================================================================*/

/**
 * @brief **Forward-Right-Down (FRD)** body-fixed frame.
 * @details A right-handed Cartesian frame attached to the moving vehicle.
 * - **+X-axis**: Points out the front of the vehicle (roll axis).
 * - **+Y-axis**: Points out the right side of the vehicle (pitch axis).
 * - **+Z-axis**: Points down through the vehicle (yaw axis).
 */
struct frd {
    static constexpr auto name = "frd";
    using axis = axis_frd;
    static constexpr FrameTag tag = FrameTag::Body;
};

/**
 * @brief **Forward-Left-Up (FLU)** body-fixed frame.
 * @details A right-handed Cartesian frame attached to the moving vehicle.
 * - **+X-axis**: Points out the front of the vehicle.
 * - **+Y-axis**: Points out the left side of the vehicle.
 * - **+Z-axis**: Points up through the vehicle.
 */
struct flu {
    static constexpr auto name = "flu";
    using axis = axis_flu;
    static constexpr FrameTag tag = FrameTag::Body;
};

/**
 * @brief **Right-Forward-Up (RFU)** body-fixed frame.
 * @details A body-fixed frame used in computer vision and some robotics libraries.
 * - **+X-axis**: Points out the right side of the vehicle.
 * - **+Y-axis**: Points out the front of the vehicle.
 * - **+Z-axis**: Points up through the vehicle.
 */
struct rfu {
    static constexpr auto name = "rfu";
    using axis = axis_rfu;
    static constexpr FrameTag tag = FrameTag::Body;
};

/**
 * @brief **Surge-Sway-Heave (SSH)** velocity frame.
 * @details A frame used in marine robotics to describe body-fixed linear velocities.
 * Its axes are typically aligned with the FRD frame.
 * - **+X-axis**: Forward/backward motion (Surge).
 * - **+Y-axis**: Right/left motion (Sway).
 * - **+Z-axis**: Down/up motion (Heave).
 */
struct ssh {
    static constexpr auto name = "ssh";
    using axis = axis_frd;
    static constexpr FrameTag tag = FrameTag::Body;
};

/**
 * @brief Defines a standard reference frame for an Inertial Measurement Unit (IMU).
 * @details This frame is rigidly attached to the IMU sensor. It's used to express
 * measurements of angular velocity and linear acceleration. The axis conventions
 * are defined to be right-handed and typically follow common standards for robotics.
 * The standard convention is:
 * - **+X**: Points forward out of the sensor.
 * - **+Y**: Points to the left.
 * - **+Z**: Points up.
 */
struct imu {
    static constexpr auto name = "imu";
    using axis = axis_frd;
    static constexpr FrameTag tag = FrameTag::Sensor;
};

/**
 * @brief Defines a standard reference frame for a camera or image sensor.
 * @details This frame is rigidly attached to the camera, with its origin at the
 * optical center. It's the reference frame in which 3D points are projected
 * onto the image plane.
 * The standard convention in robotics and computer vision is:
 * - **+Z**: Points forward, out of the lens, along the optical axis.
 * - **+X**: Points to the right from the camera's perspective.
 * - **+Y**: Points down from the camera's perspective.
 * @note Verify this against your specific camera's documentation.
 */
struct camera {
    static constexpr auto name = "camera";
    using axis = axis_rdf;
    static constexpr FrameTag tag = FrameTag::Sensor;
};

}  // namespace refx

#endif /* _REFX_FRAMES_FRAMES_ */
