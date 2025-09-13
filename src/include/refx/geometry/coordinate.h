#ifndef _REFX_GEOMETRY_COORDINATE_
#define _REFX_GEOMETRY_COORDINATE_

#include "../frames/frames.h"
#include "../frames/internal/validators.h"
#include "../geometry/vector.h"
#include "../math/angles.h"

/**
 * @file coordinate.h
 * @brief Defines the Coordinate3D class for representing type-safe 3D positions.
 * @details This file introduces `refx::Coordinate3D`, a class designed to represent
 * a point or position in a specific reference frame. It is the conceptual counterpart
 * to `refx::Vector3D`.
 *
 * The key distinction is semantic and enforced by the type system:
 * - `Vector3D`: Represents quantities in a vector space (e.g., displacement,
 * velocity, force). Standard vector arithmetic (addition, scaling) is always valid.
 * - `Coordinate3D`: Represents a position in an affine space (a point). Vector
 * arithmetic is not always meaningful (e.g., adding two GPS coordinates).
 */

namespace refx {

/**
 * @brief A type-safe representation of a 3D position in a specific reference frame.
 * @tparam Frame The reference frame tag type (e.g., `ned`, `enu`, `lla`).
 * @tparam T The underlying scalar type for the components (e.g., `float`, `double`).
 *
 * @details This is the primary template for a 3D coordinate. For frames that form a
 * linear vector space (like all standard Cartesian frames).
 */
template <typename Frame, typename T = double>
struct Coordinate3D : public Vector3D<Frame, T> {
    // accept without specialization only if a Cartesian frame
    static_assert(
        is_directional_axis_v<typename Frame::axis>,
        "Non-specialized Coordinate3D is only possible for DirectionalAxis (Cartesian) frames");

    typedef Frame frame;
    typedef T scalar_type;
    /// @brief Inherits all constructors from the base Vector3D class.
    using Vector3D<Frame, T>::Vector3D;

    Vector3D<Frame, T> as_vector() const { return *this; }
};

/* =========================================================================
   SPECIALIZATIONS FOR NON-LINEAR FRAMES
   =========================================================================
   The following specializations are for coordinate frames that are not
   vector spaces. By inheriting from VectorContainer3D instead of Vector3D,
   we prevent invalid arithmetic operations (e.g., cross/dot products on two
   LLA coordinates) from compiling, thus ensuring type safety.
   ========================================================================= */

/**
 * @brief A 3D geographic position in the **Latitude, Longitude, Altitude (LLA)** frame.
 * @tparam T The scalar type, defaults to `double`.
 *
 * @details This specialization represents a precise point on Earth using geodetic
 * coordinates. It is fundamentally a positional type, not a vector. As such, it does
 * not support standard vector arithmetic.
 *
 * @note This type inherit from internal::VectorContainer3D<T> because we do not want it to be
 * usable with Vector3D<Frame, T> operators, that whould be meaningless for mixed
 * (spherical-cartesian) coordinates
 *
 * Internally, angular components (latitude, longitude) are stored in **degrees**.
 */
template <typename T>
struct Coordinate3D<lla, T> : public internal::VectorContainer3D<T> {
    typedef lla frame;  ///< The frame tag type for this specialization.
    using internal::VectorContainer3D<T>::VectorContainer3D;
    typedef T scalar_type;
    using container_type =
        typename internal::VectorContainer3D<T>::container_type;  ///< The underlying data
                                                                  ///< container.

    /**
     * @brief Constructs a coordinate in the LLA frame from degree/meter values.
     * @param lat Latitude in **degrees** (default: 0). Range: [-90, 90].
     * @param lon Longitude in **degrees** (default: 0). Range: [-180, 180).
     * @param alt Altitude in meters relative to the reference ellipsoid (default: 0).
     */
    Coordinate3D(T lat = 0, T lon = 0, T alt = 0)
        : internal::VectorContainer3D<T>({lat, lon, alt}) {}

    /**
     * @brief Constructs an LLA coordinate from radian/meter values.
     * @param lat_rad Latitude in radians.
     * @param lon_rad Longitude in radians.
     * @param alt Altitude in meters.
     * @return A new Coordinate3D<lla, T> object.
     */
    static Coordinate3D<lla, T> from_radians(T lat_rad, T lon_rad, T alt) {
        return Coordinate3D<lla, T>(rad2deg(lat_rad), rad2deg(lon_rad), alt);
    }
    // --- Mutable Accessors (always in degrees) ---
    /// @brief Provides mutable access to the Latitude component in **degrees**.
    T& latitude() { return this->x(); }
    /// @brief Provides mutable access to the Longitude component in **degrees**.
    T& longitude() { return this->y(); }
    /// @brief Provides mutable access to the Altitude component in **meters**.
    T& altitude() { return this->z(); }

    // --- Const Accessors (with unit conversion) ---
    /**
     * @brief Provides const access to the Latitude component.
     * @param unit The desired angular unit (`AngleUnit::Deg` or `AngleUnit::Rad`).
     * @return The latitude value, converted to the specified unit. Defaults to degrees.
     */
    T latitude(AngleUnit unit = AngleUnit::Deg) const {
        return (unit == AngleUnit::Rad) ? deg2rad(this->x()) : this->x();
    }

    /**
     * @brief Provides const access to the Longitude component.
     * @param unit The desired angular unit (`AngleUnit::Deg` or `AngleUnit::Rad`).
     * @return The longitude value, converted to the specified unit. Defaults to degrees.
     */
    T longitude(AngleUnit unit = AngleUnit::Deg) const {
        // BUG FIX: Was incorrectly returning this->x() (data[0]). Corrected to this->y() (data[1]).
        return (unit == AngleUnit::Rad) ? deg2rad(this->y()) : this->y();
    }

    /**
     * @brief Provides const access to the Altitude component.
     * @return The altitude value in **meters**.
     */
    const T& altitude() const { return this->z(); }

    /**
     * @brief Provides const access to the underlying data container.
     * @return A const reference to the internal `std::array` storing components as
     * {Latitude (deg), Longitude (deg), Altitude (m)}.
     */
    const container_type& data() const { return internal::VectorContainer3D<T>::data; }
};

/**
 * @brief A 3D geographic position in the **Latitude, Longitude, Down (LLD)** frame.
 * @tparam T The scalar type, defaults to `double`.
 *
 * @details Similar to LLA, but the third component represents depth or a positive-downward
 * displacement from the reference ellipsoid. This is common in marine and sub-surface navigation.
 * Internally, angular components are stored in **degrees**.
 *
 * @note This type inherit from internal::VectorContainer3D<T> because we do not want it to be
 * usable with Vector3D<Frame, T> operators, that whould be meaningless for mixed
 * (spherical-cartesian) coordinates
 */
template <typename T>
struct Coordinate3D<lld, T> : public internal::VectorContainer3D<T> {
    typedef lld frame;  ///< The frame tag type for this specialization.
    using internal::VectorContainer3D<T>::VectorContainer3D;
    typedef T scalar_type;
    using container_type =
        typename internal::VectorContainer3D<T>::container_type;  ///< The underlying data
                                                                  ///< container.

    /**
     * @brief Constructs a coordinate in the LLD frame from degree/meter values.
     * @param lat Latitude in **degrees** (default: 0).
     * @param lon Longitude in **degrees** (default: 0).
     * @param dw Down/Depth in **meters** (positive downward, default: 0).
     */
    Coordinate3D(T lat = 0, T lon = 0, T dw = 0) : internal::VectorContainer3D<T>({lat, lon, dw}) {}

    /**
     * @brief Constructs an LLA coordinate from radian/meter values.
     * @param lat_rad Latitude in radians.
     * @param lon_rad Longitude in radians.
     * @param dow Down in meters.
     * @return A new Coordinate3D<lld, T> object.
     */
    static Coordinate3D<lld, T> from_radians(T lat_rad, T lon_rad, T dow) {
        return Coordinate3D<lld, T>(rad2deg(lat_rad), rad2deg(lon_rad), dow);
    }

    // --- Mutable Accessors (always in degrees) ---
    /// @brief Provides mutable access to the Latitude component in **degrees**.
    T& latitude() { return this->x(); }
    /// @brief Provides mutable access to the Longitude component in **degrees**.
    T& longitude() { return this->y(); }
    /// @brief Provides mutable access to the Down/Depth component in **meters**.
    T& down() { return this->z(); }

    // --- Const Accessors (with unit conversion) ---
    /**
     * @brief Provides const access to the Latitude component.
     * @param unit Desired angular unit (`AngleUnit::Deg` or `AngleUnit::Rad`).
     * @return The latitude value in the specified unit (defaults to degrees).
     */
    T latitude(AngleUnit unit = AngleUnit::Deg) const {
        return (unit == AngleUnit::Rad) ? deg2rad(this->x()) : this->x();
    }

    /**
     * @brief Provides const access to the Longitude component.
     * @param unit Desired angular unit (`AngleUnit::Deg` or `AngleUnit::Rad`).
     * @return The longitude value in the specified unit (defaults to degrees).
     */
    T longitude(AngleUnit unit = AngleUnit::Deg) const {
        return (unit == AngleUnit::Rad) ? deg2rad(this->y()) : this->y();
    }

    /**
     * @brief Provides const access to the Down/Depth component.
     * @return The down value in **meters**.
     */
    const T& down() const { return this->z(); }

    /**
     * @brief Provides const access to the underlying data container.
     * @return A const reference to the internal `std::array` storing components as
     * {Latitude (deg), Longitude (deg), Down (m)}.
     */
    const container_type& data() const { return internal::VectorContainer3D<T>::data; }
};

/**
 * @brief A 3D position in the **Azimuth, Elevation, Range (AER)** spherical frame.
 * @tparam T The scalar type, defaults to `double`.
 *
 * @details Represents a point in a local spherical coordinate system, commonly used for
 * sensor measurements (e.g., radar, LiDAR).
 * - **Azimuth**: Horizontal angle from a reference direction.
 * - **Elevation**: Vertical angle from the horizontal plane.
 * - **Range**: Straight-line distance from the origin.
 * Internally, angular components are stored in **degrees**.
 *
 * @note This type inherit from internal::VectorContainer3D<T> because we do not want it to be
 * usable with Vector3D<Frame, T> operators, that whould be meaningless for mixed
 * (spherical-cartesian) coordinates
 */
template <typename T>
struct Coordinate3D<aer, T> : public internal::VectorContainer3D<T> {
    typedef aer frame;  ///< The frame tag type for this specialization.
    using internal::VectorContainer3D<T>::VectorContainer3D;
    typedef T scalar_type;
    using container_type =
        typename internal::VectorContainer3D<T>::container_type;  ///< The underlying data
                                                                  ///< container.

    /**
     * @brief Constructs a coordinate in the AER frame from degree/meter values.
     * @param az Azimuth in **degrees** (default: 0).
     * @param el Elevation in **degrees** (default: 0).
     * @param rg Range in **meters** (default: 0).
     */
    Coordinate3D(T az = 0, T el = 0, T rg = 0) : internal::VectorContainer3D<T>({az, el, rg}) {}

    /**
     * @brief Constructs an LLA coordinate from radian/meter values.
     * @param az_rad Azimuth in radians.
     * @param el_rad Elevation in radians.
     * @param rg Range in meters.
     * @return A new Coordinate3D<aer, T> object.
     */
    static Coordinate3D<aer, T> from_radians(T az_rad, T el_rad, T rg) {
        return Coordinate3D<aer, T>(rad2deg(az_rad), rad2deg(el_rad), rg);
    }

    // --- Mutable Accessors (always in degrees) ---
    /// @brief Provides mutable access to the Azimuth component in **degrees**.
    T& azimuth() { return this->x(); }
    /// @brief Provides mutable access to the Elevation component in **degrees**.
    T& elevation() { return this->y(); }
    /// @brief Provides mutable access to the Range component in **meters**.
    T& range() { return this->z(); }

    // --- Const Accessors (with unit conversion) ---
    /**
     * @brief Provides const access to the Azimuth component.
     * @param unit Desired angular unit (`AngleUnit::Deg` or `AngleUnit::Rad`).
     * @return The azimuth value in the specified unit (defaults to degrees).
     */
    T azimuth(AngleUnit unit = AngleUnit::Deg) const {
        return (unit == AngleUnit::Rad) ? deg2rad(this->x()) : this->x();
    }

    /**
     * @brief Provides const access to the Elevation component.
     * @param unit Desired angular unit (`AngleUnit::Deg` or `AngleUnit::Rad`).
     * @return The elevation value in the specified unit (defaults to degrees).
     */
    T elevation(AngleUnit unit = AngleUnit::Deg) const {
        return (unit == AngleUnit::Rad) ? deg2rad(this->y()) : this->y();
    }

    /**
     * @brief Provides const access to the Range component.
     * @return The range value in **meters**.
     */
    const T& range() const { return this->z(); }

    /**
     * @brief Provides const access to the underlying data container.
     * @return A const reference to the internal `std::array` storing components as
     * {Azimuth (deg), Elevation (deg), Range (m)}.
     */
    // BUG FIX: Comment was incorrect. Corrected to reflect AER components.
    const container_type& data() const { return internal::VectorContainer3D<T>::data; }
};

/* =========================================================================
   OPERATORS FOR MATHEMATICALLY CONSISTENT COORDINATE ARITHMETIC
   ========================================================================= */

/**
 * @brief Disabled operator to prevent the addition of two absolute coordinates.
 * @details In affine geometry, the addition of two points (positions) is a
 * mathematically undefined and physically meaningless operation. It prevents logical
 * errors at compile-time, such as attempting to add two GPS coordinates together.
 * @return This operator is deleted and will cause a compile-time error if used.
 */
template <typename Frame1, typename Frame2, typename T>
Coordinate3D<Frame1, T> operator+(const Coordinate3D<Frame1, T>& a,
                                  const Coordinate3D<Frame2, T>& b) {
    static_assert(false,
                  "The summation between two Coordinates3D is not allowed. Check the documentation "
                  "for details.");
}

/**
 * @brief Computes the displacement vector between two coordinates.
 * @details This operator correctly implements the mathematical principle that the
 * difference between two points (`to - from`) is a displacement vector. The
 * underlying implementation leverages the `FrameTraits` for the given `Frame`.
 *
 * - For **linear frames** (e.g., `NED`, `ECEF`), this performs a standard
 * component-wise subtraction, yielding a true Euclidean `Vector3D`.
 * - For **non-linear frames** (e.g., `LLA`), this performs a specialized
 * subtraction that correctly handles angular wrap-around (for longitude) and
 * clamping, yielding a `Vector3D<lla>` which represents a geodetic difference
 * vector `{Δlat, Δlon, Δalt}`.
 *
 * @param a The destination coordinate (`to`).
 * @param b The origin coordinate (`from`).
 * @return A `Vector3D<Frame, T>` representing the displacement from `b` to `a`.
 */
template <typename Frame1, typename Frame2, typename T>
Vector3D<Frame1, T> operator-(const Coordinate3D<Frame1, T>& a, const Coordinate3D<Frame2, T>& b) {
    // The subtraction of two Vector3D objects will correctly invoke the
    // specialized arithmetic defined by FramedVecOperator, which uses FrameTraits.
    if constexpr (internal::FrameValidator<Frame1, Frame2>::validate()) {
        return Vector3D<Frame1, T>(a.data()) - Vector3D<Frame2, T>(b.data());
    }
}

/**
 * @brief Translates a coordinate by a displacement vector.
 * @details This operator correctly implements the mathematical principle that adding
 * a displacement vector to a point yields a new point. The underlying implementation
 * leverages the `FrameTraits` for the given `Frame`.
 *
 * - For **linear frames**, this is a standard component-wise addition.
 * - For **non-linear frames**, this correctly applies the differential quantities
 * from the vector to the coordinate, handling angular wrap-around and clamping
 * as needed.
 *
 * @param a The starting coordinate (point).
 * @param b The displacement vector to apply.
 * @return A new `Coordinate3D<Frame, T>` representing the translated position.
 */
template <typename Frame1, typename Frame2, typename T>
Coordinate3D<Frame1, T> operator+(const Coordinate3D<Frame1, T>& a, const Vector3D<Frame2, T>& b) {
    // The addition will correctly invoke the specialized arithmetic from FramedVecOperator.
    if constexpr (internal::FrameValidator<Frame1, Frame2>::validate()) {
        const auto& res = Vector3D<Frame1, T>(a.data()) + b;
        return {res.x(), res.y(), res.z()};
    }
}

/**
 * @brief Translates a coordinate by a displacement vector (commutative version).
 * @details Provides a commutative version of the `Coordinate + Vector` operation for
 * convenience and completeness.
 * @param a The displacement vector to apply.
 * @param b The starting coordinate (point).
 * @return A new `Coordinate3D<Frame, T>` representing the translated position.
 */
template <typename Frame1, typename Frame2, typename T>
Coordinate3D<Frame1, T> operator+(const Vector3D<Frame1, T>& a, const Coordinate3D<Frame2, T>& b) {
    if constexpr (internal::FrameValidator<Frame1, Frame2>::validate()) {
        // Forward the call to the canonical version.
        return b + a;
    }
}

}  // namespace refx

#endif /* _REFX_GEOMETRY_COORDINATE_ */
