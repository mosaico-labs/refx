#ifndef _REFX_GEOMETRY_VECTOR_
#define _REFX_GEOMETRY_VECTOR_

#include <array>

#include "../frames/frames.h"
#include "../frames/internal/traits.h"  //has_axis_and_tag_v
#include "../frames/internal/validators.h"
#include "../math/angles.h"
#include "internal/operators.h"
#include "internal/vector_base.h"

/**
 * @file vector.h
 * @brief Defines a compile-time, frame-aware 3D vector class and its specializations.
 * @details This file provides the core `refx::Vector3D` class. Its key feature is
 * **frame awareness**, which leverages the C++ type system to prevent common and often
 * hard-to-debug errors arising from mixing vectors from different coordinate frames.
 *
 * The primary template `Vector3D<Frame, T>` is not intended for direct use. Instead,
 * users should instantiate one of its specializations, which correspond to common
 * reference frames in robotics and navigation (e.g., `Vector3D<ned>`, `Vector3D<frd>`).
 * These specializations provide semantic accessors (e.g., `.north()`, `.forward()`)
 * to improve code readability and correctness.
 *
 * All arithmetic operators are overloaded to enforce that operations are only performed
 * between vectors of the same frame, failing at compile-time otherwise.
 */

namespace refx {

/**
 * @brief A type-safe, 3D vector defined in a specific reference frame.
 * @tparam Frame A tag type (e.g., `refx::ned`, `refx::enu`) representing the vector's
 * coordinate frame.
 * @tparam T The underlying scalar type for the vector's components (e.g., `float`, `double`).
 *
 * @details This is the primary class template for a frame-aware 3D vector and provides
 * simple accessors .x(), .y(), .z()
 *
 * The benefit of this class is that it encodes the reference frame into the
 * type itself. This allows the compiler to catch logical errors, such as adding a
 * vector in the robot's body frame (`frd`) to a vector in a global navigation
 * frame (`ned`).
 */
template <typename Frame, typename T = double>
struct Vector3D : public internal::VectorContainer3D<T> {
    static_assert(internal::has_axis_and_tag_v<Frame>,
                  "Frame must be a valid refx frame type (must provide typedefs 'axis' and 'tag')");
    typedef Frame frame;  ///< The frame type for this specialization.
    using internal::VectorContainer3D<T>::VectorContainer3D;
    using scalar_type = T;
    using container_type =
        typename internal::VectorContainer3D<T>::container_type;  ///< The underlying data
                                                                  ///< container.

    /**
     * @brief Constructs a vector in the generic frame.
     * @param x X-axis component (default: 0).
     * @param y Y-axis component (default: 0).
     * @param z Z-axis component (default: 0).
     */
    Vector3D(T x = 0, T y = 0, T z = 0) : internal::VectorContainer3D<T>({x, y, z}) {}

    /**
     * @brief Provides const access to the underlying data container.
     * @return A const reference to the internal `std::array` storing the components in {X, Y, Z}
     * order.
     */
    const container_type& data() const { return internal::VectorContainer3D<T>::data; }
};

/**
 * @brief Specialization of `Vector3D` for the **North-East-Down (NED)** reference frame.
 * @tparam T The scalar type, defaults to `double`.
 *
 * @details The NED frame is a local tangent plane coordinate system. Its axes are defined as:
 * - **+X (north())**: Points towards the geographic North Pole.
 * - **+Y (east())**: Points towards the geographic East.
 * - **+Z (down())**: Points downwards, towards the Earth's center (aligned with gravity).
 */
template <typename T>
struct Vector3D<ned, T> : public internal::VectorContainer3D<T> {
    typedef ned frame;  ///< The frame tag type for this specialization.
    using internal::VectorContainer3D<T>::VectorContainer3D;
    using scalar_type = T;
    using container_type =
        typename internal::VectorContainer3D<T>::container_type;  ///< The underlying data
                                                                  ///< container.

    /**
     * @brief Constructs a vector in the NED frame.
     * @param n North component in meters (default: 0).
     * @param e East component in meters (default: 0).
     * @param d Down component in meters (default: 0).
     */
    Vector3D(T n = 0, T e = 0, T d = 0) : internal::VectorContainer3D<T>(n, e, d) {}

    // --- Semantic Accessors ---

    /// @brief Provides mutable access to the North component.
    T& north() { return this->x(); }
    /// @brief Provides mutable access to the East component.
    T& east() { return this->y(); }
    /// @brief Provides mutable access to the Down component.
    T& down() { return this->z(); }

    /// @brief Provides const access to the North component.
    T north() const { return this->x(); }
    /// @brief Provides const access to the East component.
    T east() const { return this->y(); }
    /// @brief Provides const access to the Down component.
    T down() const { return this->z(); }

    /**
     * @brief Provides const access to the underlying data container.
     * @return A const reference to the internal `std::array` storing the components in {North,
     * East, Down} order.
     */
    const container_type& data() const { return internal::VectorContainer3D<T>::data; }
};

/**
 * @brief Specialization of `Vector3D` for the **East-North-Up (ENU)** reference frame.
 * @tparam T The scalar type, defaults to `double`.
 *
 * @details The ENU frame is another local tangent plane system. Its axes are:
 * - **+X (east())**: Points towards the geographic East.
 * - **+Y (north())**: Points towards the geographic North Pole.
 * - **+Z (up())**: Points upwards, away from the Earth's center (opposite to gravity).
 */
template <typename T>
struct Vector3D<enu, T> : public internal::VectorContainer3D<T> {
    typedef enu frame;  ///< The frame tag type for this specialization.
    using internal::VectorContainer3D<T>::VectorContainer3D;
    using scalar_type = T;
    using container_type =
        typename internal::VectorContainer3D<T>::container_type;  ///< The underlying data
                                                                  ///< container.

    /**
     * @brief Constructs a vector in the ENU frame.
     * @param e East component in meters (default: 0).
     * @param n North component in meters (default: 0).
     * @param u Up component in meters (default: 0).
     */
    Vector3D(T e = 0, T n = 0, T u = 0) : internal::VectorContainer3D<T>({e, n, u}) {}

    // --- Semantic Accessors ---

    /// @brief Provides mutable access to the East component.
    T& east() { return this->x(); }
    /// @brief Provides mutable access to the North component.
    T& north() { return this->y(); }
    /// @brief Provides mutable access to the Up component.
    T& up() { return this->z(); }

    /// @brief Provides const access to the East component.
    T east() const { return this->x(); }
    /// @brief Provides const access to the North component.
    T north() const { return this->y(); }
    /// @brief Provides const access to the Up component.
    T up() const { return this->z(); }

    /**
     * @brief Provides const access to the underlying data container.
     * @return A const reference to the internal `std::array` storing the components in {East,
     * North, Up} order.
     */
    const container_type& data() const { return internal::VectorContainer3D<T>::data; }
};

/**
 * @brief Specialization of `Vector3D` for the **North-West-Up (NWU)** reference frame.
 * @tparam T The scalar type, defaults to `double`.
 *
 * @details The NWU frame is another local tangent plane system. Its axes are:
 * - **+X (north())**: Points towards the geographic North Pole.
 * - **+Y (west())**: Points towards the geographic West.
 * - **+Z (up())**: Points upwards, away from the Earth's center.
 * It forms a right-handed coordinate system.
 */
template <typename T>
struct Vector3D<nwu, T> : public internal::VectorContainer3D<T> {
    typedef nwu frame;  ///< The frame tag type for this specialization.
    using internal::VectorContainer3D<T>::VectorContainer3D;
    using scalar_type = T;
    using container_type =
        typename internal::VectorContainer3D<T>::container_type;  ///< The underlying data
                                                                  ///< container.

    /**
     * @brief Constructs a vector in the NWU frame.
     * @param n North component in meters (default: 0).
     * @param w West component in meters (default: 0).
     * @param u Up component in meters (default: 0).
     */
    Vector3D(T n = 0, T w = 0, T u = 0) : internal::VectorContainer3D<T>({n, w, u}) {}

    // --- Semantic Accessors ---

    /// @brief Provides mutable access to the North component.
    T& north() { return this->x(); }
    /// @brief Provides mutable access to the West component.
    T& west() { return this->y(); }
    /// @brief Provides mutable access to the Up component.
    T& up() { return this->z(); }

    /// @brief Provides const access to the North component.
    T north() const { return this->x(); }
    /// @brief Provides const access to the West component.
    T west() const { return this->y(); }
    /// @brief Provides const access to the Up component.
    T up() const { return this->z(); }

    /**
     * @brief Provides const access to the underlying data container.
     * @return A const reference to the internal `std::array` storing the components in {North,
     * West, Up} order.
     */
    const container_type& data() const { return internal::VectorContainer3D<T>::data; }
};

/**
 * @brief A geodetic difference vector in the LLA (Latitude, Longitude, Altitude) space.
 * @tparam T The scalar type, defaults to `double`.
 *
 * @details This class is a highly specialized type and does **not** represent a standard
 * Euclidean vector. Instead, it represents a **differential quantity** or a "geodetic
 * difference vector" whose components are `{Δlatitude, Δlongitude, Δaltitude}`.
 *
 * Its primary purpose is to hold the result of subtracting two `Coordinate3D<lla>`
 * positions. The subtraction operation that creates this vector is specialized:
 * - The latitude difference (`Δlat`) is calculated as the **shortest angle**,
 * correctly handling the wrap-around at ±90°.
 * - The longitude difference (`Δlon`) is calculated as the **shortest angle**,
 * correctly handling the wrap-around at ±180°.
 * - The altitude difference is simple linear subtractions.
 *
 * The components have mixed units of **degrees** and **meters**. Consequently,
 * standard vector operations like magnitude, dot/cross products, or arbitrary scaling
 * are not physically meaningful for this type. Its main use is for applying small,
 * incremental updates to a `Coordinate3D<lla>` or for representing the innovation
 * (error term) in a state estimator like an EKF.
 *
 * @warning This is NOT a geometric vector in 3D space. To get a true Euclidean
 * displacement vector between two LLA points, you must first convert them to a
 * Cartesian frame (e.g., ECEF) and then subtract them.
 */
template <typename T>
struct Vector3D<lla, T> : public internal::VectorContainer3D<T> {
    typedef lla frame;  ///< The frame tag type for this specialization.
    using internal::VectorContainer3D<T>::VectorContainer3D;
    using scalar_type = T;
    using container_type =
        typename internal::VectorContainer3D<T>::container_type;  ///< The underlying data
                                                                  ///< container.

    /**
     * @brief Constructs a geodetic difference vector.
     * @details While this constructor is public, this object is typically created by
     * subtracting two `Coordinate3D<lla>` objects.
     * @param d_lat The difference in latitude in **degrees**.
     * @param d_lon The difference in longitude in **degrees**.
     * @param d_alt The difference in altitude in **meters**.
     */
    Vector3D(T d_lat = 0, T d_lon = 0, T d_alt = 0)
        : internal::VectorContainer3D<T>({d_lat, d_lon, d_alt}) {}

    /**
     * @brief Constructs a geodetic difference vector from an existing data container.
     * @param v A container (e.g., `std::array<T, 3>`) with {Δlat, Δlon, Δalt} components.
     */
    Vector3D(const container_type& v) : internal::VectorContainer3D<T>(v) {}

    /**
     * @brief Constructs a Delta LLA vector from radian/meter values.
     * @param delta_lat_rad Delta Latitude in radians.
     * @param delta_lon_rad Delta Longitude in radians.
     * @param delta_alt Delta Altitude in meters.
     * @return A new Coordinate3D<lla, T> object.
     */
    static Vector3D<lla, T> from_radians(T delta_lat_rad, T delta_lon_rad, T delta_alt) {
        return Vector3D<lla, T>(rad2deg(delta_lat_rad), rad2deg(delta_lon_rad), delta_alt);
    }

    // --- Semantic Accessors for Differential Quantities ---

    // --- Mutable Accessors (always in degrees) ---
    /// @brief Provides mutable access to the Latitude component in **degrees**.
    T& delta_latitude() { return this->x(); }
    /// @brief Provides mutable access to the Longitude component in **degrees**.
    T& delta_longitude() { return this->y(); }
    /// @brief Provides mutable access to the Down/Depth component in **meters**.
    T& delta_altitude() { return this->z(); }

    /**
     * @brief Provides const access to the Latitude difference component.
     * @param unit The desired angular unit (`AngleUnit::Deg` or `AngleUnit::Rad`).
     * @return The latitude difference value, converted to the specified unit. Defaults to degrees.
     */
    T delta_latitude(AngleUnit unit = AngleUnit::Deg) const {
        return (unit == AngleUnit::Rad) ? deg2rad(this->x()) : this->x();
    }
    /**
     * @brief Provides const access to the Longitude difference component.
     * @param unit The desired angular unit (`AngleUnit::Deg` or `AngleUnit::Rad`).
     * @return The Longitude difference value, converted to the specified unit. Defaults to degrees.
     */
    T delta_longitude(AngleUnit unit = AngleUnit::Deg) const {
        return (unit == AngleUnit::Rad) ? deg2rad(this->y()) : this->y();
    }
    /// @brief Provides const access to the altitude difference component in **meters**.
    T delta_altitude() const { return this->z(); }

    /**
     * @brief Provides const access to the underlying data container.
     * @return A const reference to the internal `std::array` storing the components in
     * {Δlatitude (deg), Δlongitude (deg), Δaltitude (m)} order.
     */
    const container_type& data() const { return internal::VectorContainer3D<T>::data; }
};

/**
 * @brief A geodetic difference vector in the LLD (Latitude, Longitude, Down) space.
 * @tparam T The scalar type, defaults to `double`.
 *
 * @details This class is a highly specialized type and does **not** represent a standard
 * Euclidean vector. Instead, it represents a **differential quantity** or a "geodetic
 * difference vector" whose components are `{Δlatitude, Δlongitude, Δdown}`.
 *
 * Its primary purpose is to hold the result of subtracting two `Coordinate3D<lld>`
 * positions. The subtraction operation that creates this vector is specialized:
 * - The latitude difference (`Δlat`) is calculated as the **shortest angle**,
 * correctly handling the wrap-around at ±90°.
 * - The longitude difference (`Δlon`) is calculated as the **shortest angle**,
 * correctly handling the wrap-around at ±180°.
 * - The down difference is simple linear subtraction.
 *
 * The components have mixed units of **degrees** and **meters**. Consequently,
 * standard vector operations like magnitude, dot/cross products, or arbitrary scaling
 * are not physically meaningful for this type. Its main use is for applying small,
 * incremental updates to a `Coordinate3D<lld>` or for representing the innovation
 * (error term) in a state estimator like an EKF.
 *
 * @warning This is NOT a geometric vector in 3D space. To get a true Euclidean
 * displacement vector between two LLA points, you must first convert them to a
 * Cartesian frame (e.g., ECEF) and then subtract them.
 */
template <typename T>
struct Vector3D<lld, T> : public internal::VectorContainer3D<T> {
    typedef lld frame;  ///< The frame tag type for this specialization.
    using internal::VectorContainer3D<T>::VectorContainer3D;
    using scalar_type = T;
    using container_type =
        typename internal::VectorContainer3D<T>::container_type;  ///< The underlying data
                                                                  ///< container.

    /**
     * @brief Constructs a geodetic difference vector.
     * @details While this constructor is public, this object is typically created by
     * subtracting two `Coordinate3D<lld>` objects.
     * @param d_lat The difference in latitude in **degrees**.
     * @param d_lon The difference in longitude in **degrees**.
     * @param d_dow The difference in down in **meters**.
     */
    Vector3D(T d_lat = 0, T d_lon = 0, T d_dow = 0)
        : internal::VectorContainer3D<T>({d_lat, d_lon, d_dow}) {}

    /**
     * @brief Constructs a geodetic difference vector from an existing data container.
     * @param v A container (e.g., `std::array<T, 3>`) with {Δlat, Δlon, Δalt} components.
     */
    Vector3D(const container_type& v) : internal::VectorContainer3D<T>(v) {}

    /**
     * @brief Constructs a Delta LLD vector from radian/meter values.
     * @param delta_lat_rad Delta Latitude in radians.
     * @param delta_lon_rad Delta Longitude in radians.
     * @param delta_dow Delta Down in meters.
     * @return A new Coordinate3D<lla, T> object.
     */
    static Vector3D<lld, T> from_radians(T delta_lat_rad, T delta_lon_rad, T delta_dow) {
        return Vector3D<lld, T>(rad2deg(delta_lat_rad), rad2deg(delta_lon_rad), delta_dow);
    }

    // --- Semantic Accessors for Differential Quantities ---

    // --- Mutable Accessors (always in degrees) ---
    /// @brief Provides mutable access to the Latitude component in **degrees**.
    T& delta_latitude() { return this->x(); }
    /// @brief Provides mutable access to the Longitude component in **degrees**.
    T& delta_longitude() { return this->y(); }
    /// @brief Provides mutable access to the Down/Depth component in **meters**.
    T& delta_down() { return this->z(); }

    // --- Const Accessors (with unit conversion) ---
    /**
     * @brief Provides const access to the Latitude difference component.
     * @param unit The desired angular unit (`AngleUnit::Deg` or `AngleUnit::Rad`).
     * @return The latitude difference value, converted to the specified unit. Defaults to degrees.
     */
    T delta_latitude(AngleUnit unit = AngleUnit::Deg) const {
        return (unit == AngleUnit::Rad) ? deg2rad(this->x()) : this->x();
    }
    /**
     * @brief Provides const access to the Longitude difference component.
     * @param unit The desired angular unit (`AngleUnit::Deg` or `AngleUnit::Rad`).
     * @return The Longitude difference value, converted to the specified unit. Defaults to degrees.
     */
    T delta_longitude(AngleUnit unit = AngleUnit::Deg) const {
        return (unit == AngleUnit::Rad) ? deg2rad(this->y()) : this->y();
    }
    /// @brief Provides const access to the down difference component in **meters**.
    T delta_down() const { return this->z(); }

    /**
     * @brief Provides const access to the underlying data container.
     * @return A const reference to the internal `std::array` storing the components in
     * {Δlatitude (deg), Δlongitude (deg), Δdown (m)} order.
     */
    const container_type& data() const { return internal::VectorContainer3D<T>::data; }
};

/**
 * @brief A geodetic difference vector in the AER (Azimuth, Elevation, Range) space.
 * @tparam T The scalar type, defaults to `double`.
 *
 * @details This class is a highly specialized type and does **not** represent a standard
 * Euclidean vector. Instead, it represents a **differential quantity** or a "geodetic
 * difference vector" whose components are `{Δazimuth, Δlongitude, Δdown}`.
 *
 * Its primary purpose is to hold the result of subtracting two `Coordinate3D<aer>`
 * positions. The subtraction operation that creates this vector is specialized:
 * - The azimuth difference (`Δaz`) is calculated as the **shortest angle**,
 * correctly handling the wrap-around at 0...360°°.
 * - The elevation difference (`Δele`) is calculated as the **shortest angle**,
 * correctly handling the wrap-around at ±180°.
 * - The altitude difference is simple linear subtraction.
 *
 * The components have mixed units of **degrees** and **meters**. Consequently,
 * standard vector operations like magnitude, dot/cross products, or arbitrary scaling
 * are not physically meaningful for this type. Its main use is for applying small,
 * incremental updates to a `Coordinate3D<aer>` or for representing the innovation
 * (error term) in a state estimator like an EKF.
 *
 * @warning This is NOT a geometric vector in 3D space. To get a true Euclidean
 * displacement vector between two AER points, you must first convert them to a
 * Cartesian frame (e.g., NED) and then subtract them.
 */
template <typename T>
struct Vector3D<aer, T> : public internal::VectorContainer3D<T> {
    typedef aer frame;  ///< The frame tag type for this specialization.
    using internal::VectorContainer3D<T>::VectorContainer3D;
    using scalar_type = T;
    using container_type =
        typename internal::VectorContainer3D<T>::container_type;  ///< The underlying data
                                                                  ///< container.

    /**
     * @brief Constructs a geodetic difference vector.
     * @details While this constructor is public, this object is typically created by
     * subtracting two `Coordinate3D<aer>` objects.
     * @param d_az The difference in azimuth in **degrees**.
     * @param d_ele The difference in elevation in **degrees**.
     * @param d_rg The difference in range in **meters**.
     */
    Vector3D(T d_az = 0, T d_ele = 0, T d_rg = 0)
        : internal::VectorContainer3D<T>({d_az, d_ele, d_rg}) {}

    /**
     * @brief Constructs a geodetic difference vector from an existing data container.
     * @param v A container (e.g., `std::array<T, 3>`) with {Δlat, Δlon, Δalt} components.
     */
    Vector3D(const container_type& v) : internal::VectorContainer3D<T>(v) {}

    /**
     * @brief Constructs a Delta LLD vector from radian/meter values.
     * @param delta_az_rad Delta Azimiuth in radians.
     * @param delta_el_rad Delta Elevation in radians.
     * @param delta_rg Delta Range in meters.
     * @return A new Coordinate3D<lla, T> object.
     */
    static Vector3D<aer, T> from_radians(T delta_az_rad, T delta_el_rad, T delta_rg) {
        return Vector3D<aer, T>(rad2deg(delta_az_rad), rad2deg(delta_el_rad), delta_rg);
    }

    // --- Semantic Accessors for Differential Quantities (with unit conversion) ---

    // --- Mutable Accessors (always in degrees) ---
    /// @brief Provides mutable access to the Latitude component in **degrees**.
    T& delta_azimuth() { return this->x(); }
    /// @brief Provides mutable access to the Longitude component in **degrees**.
    T& delta_elevation() { return this->y(); }
    /// @brief Provides mutable access to the Down/Depth component in **meters**.
    T& delta_range() { return this->z(); }

    /**
     * @brief Provides const access to the azimuth difference component.
     * @param unit The desired angular unit (`AngleUnit::Deg` or `AngleUnit::Rad`).
     * @return The azimuth difference value, converted to the specified unit. Defaults to degrees.
     */
    T delta_azimuth(AngleUnit unit = AngleUnit::Deg) const {
        return (unit == AngleUnit::Rad) ? deg2rad(this->x()) : this->x();
    }
    /**
     * @brief Provides const access to the elevation difference component.
     * @param unit The desired angular unit (`AngleUnit::Deg` or `AngleUnit::Rad`).
     * @return The elevation difference value, converted to the specified unit. Defaults to degrees.
     */
    T delta_elevation(AngleUnit unit = AngleUnit::Deg) const {
        return (unit == AngleUnit::Rad) ? deg2rad(this->y()) : this->y();
    }
    /// @brief Provides const access to the range difference component in **meters**.
    T delta_range() const { return this->z(); }

    /**
     * @brief Provides const access to the underlying data container.
     * @return A const reference to the internal `std::array` storing the components in
     * {Δazimuth (deg), Δelevation (deg), Δrange (m)} order.
     */
    const container_type& data() const { return internal::VectorContainer3D<T>::data; }
};

/**
 * @brief Specialization of `Vector3D` for the **Forward-Right-Down (FRD)** body frame.
 * @tparam T The scalar type, defaults to `double`.
 *
 * @details The FRD frame is a body-fixed frame. Its origin is at the vehicle's center of gravity.
 * Its axes are:
 * - **+X (forward())**: Points out the front of the vehicle.
 * - **+Y (right())**: Points out the right side of the vehicle.
 * - **+Z (down())**: Points downwards relative to the vehicle's orientation.
 */
template <typename T>
struct Vector3D<frd, T> : public internal::VectorContainer3D<T> {
    typedef frd frame;  ///< The frame tag type for this specialization.
    using internal::VectorContainer3D<T>::VectorContainer3D;
    using scalar_type = T;
    using container_type =
        typename internal::VectorContainer3D<T>::container_type;  ///< The underlying data
                                                                  ///< container.

    /**
     * @brief Constructs a vector in the FRD frame.
     * @param f Forward component in meters (default: 0).
     * @param r Right component in meters (default: 0).
     * @param d Down component in meters (default: 0).
     */
    Vector3D(T f = 0, T r = 0, T d = 0) : internal::VectorContainer3D<T>({f, r, d}) {}

    // --- Semantic Accessors ---

    /// @brief Provides mutable access to the Forward component.
    T& forward() { return this->x(); }
    /// @brief Provides mutable access to the Right component.
    T& right() { return this->y(); }
    /// @brief Provides mutable access to the Down component.
    T& down() { return this->z(); }

    /// @brief Provides const access to the Forward component.
    T forward() const { return this->x(); }
    /// @brief Provides const access to the Right component.
    T right() const { return this->y(); }
    /// @brief Provides const access to the Down component.
    T down() const { return this->z(); }

    /**
     * @brief Provides const access to the underlying data container.
     * @return A const reference to the internal `std::array` storing the components in {Forward,
     * Right, Down} order.
     */
    const container_type& data() const { return internal::VectorContainer3D<T>::data; }
};

/**
 * @brief Specialization of `Vector3D` for the **Surge-Sway-Heave (SSH)** body frame.
 * @tparam T The scalar type, defaults to `double`.
 *
 * @details The SSH frame is a body-fixed frame used in marine robotics (ships, AUVs, ROVs)
 * to describe linear velocities. Its axes align with the FRD frame but use nautical terminology:
 * - **+X (surge())**: Forward/backward motion.
 * - **+Y (sway())**: Starboard (right)/port (left) motion.
 * - **+Z (heave())**: Downward/upward motion.
 */
template <typename T>
struct Vector3D<ssh, T> : public internal::VectorContainer3D<T> {
    typedef ssh frame;  ///< The frame tag type for this specialization.
    using internal::VectorContainer3D<T>::VectorContainer3D;
    using scalar_type = T;
    using container_type =
        typename internal::VectorContainer3D<T>::container_type;  ///< The underlying data
                                                                  ///< container.

    /**
     * @brief Constructs a vector in the SSH frame.
     * @param sg Surge component (forward velocity) in meters/sec (default: 0).
     * @param sw Sway component (rightward velocity) in meters/sec (default: 0).
     * @param hv Heave component (downward velocity) in meters/sec (default: 0).
     */
    Vector3D(T sg = 0, T sw = 0, T hv = 0) : internal::VectorContainer3D<T>({sg, sw, hv}) {}

    // --- Semantic Accessors ---

    /// @brief Provides mutable access to the Surge (forward) component.
    T& surge() { return this->x(); }
    /// @brief Provides mutable access to the Sway (rightward) component.
    T& sway() { return this->y(); }
    /// @brief Provides mutable access to the Heave (downward) component.
    T& heave() { return this->z(); }

    /// @brief Provides const access to the Surge (forward) component.
    T surge() const { return this->x(); }
    /// @brief Provides const access to the Sway (rightward) component.
    T sway() const { return this->y(); }
    /// @brief Provides const access to the Heave (downward) component.
    T heave() const { return this->z(); }

    /**
     * @brief Provides const access to the underlying data container.
     * @return A const reference to the internal `std::array` storing the components in {Surge,
     * Sway, Heave} order.
     */
    const container_type& data() const { return internal::VectorContainer3D<T>::data; }
};

/**
 * @brief Specialization of `Vector3D` for the **Right-Forward-Up (RFU)** body frame.
 * @tparam T The scalar type, defaults to `double`.
 *
 * @details The RFU frame is a body-fixed frame. Its axes are:
 * - **+X (right())**: Points out the right side of the vehicle/camera.
 * - **+Y (forward())**: Points out the front of the vehicle/camera.
 * - **+Z (up())**: Points upwards relative to the vehicle/camera.
 */
template <typename T>
struct Vector3D<rfu, T> : public internal::VectorContainer3D<T> {
    typedef rfu frame;  ///< The frame tag type for this specialization.
    using internal::VectorContainer3D<T>::VectorContainer3D;
    using scalar_type = T;
    using container_type =
        typename internal::VectorContainer3D<T>::container_type;  ///< The underlying data
                                                                  ///< container.

    /**
     * @brief Constructs a vector in the RFU frame.
     * @param r Right component in meters (default: 0).
     * @param f Forward component in meters (default: 0).
     * @param u Up component in meters (default: 0).
     */
    Vector3D(T r = 0, T f = 0, T u = 0) : internal::VectorContainer3D<T>({r, f, u}) {}

    // --- Semantic Accessors ---

    /// @brief Provides mutable access to the Right component.
    T& right() { return this->x(); }
    /// @brief Provides mutable access to the Forward component.
    T& forward() { return this->y(); }
    /// @brief Provides mutable access to the Up component.
    T& up() { return this->z(); }

    /// @brief Provides const access to the Right component.
    T right() const { return this->x(); }
    /// @brief Provides const access to the Forward component.
    T forward() const { return this->y(); }
    /// @brief Provides const access to the Up component.
    T up() const { return this->z(); }

    /**
     * @brief Provides const access to the underlying data container.
     * @return A const reference to the internal `std::array` storing the components in {Right,
     * Forward, Up} order.
     */
    const container_type& data() const { return internal::VectorContainer3D<T>::data; }
};

/**
 * @brief Specialization of `Vector3D` for the **Forward-Left-Up (FLU)** body frame.
 * @tparam T The scalar type, defaults to `double`.
 *
 * @details The FLU frame is a right-handed body-fixed frame. Its axes are:
 * - **+X (forward())**: Points out the front of the vehicle.
 * - **+Y (left())**: Points out the left side of the vehicle.
 * - **+Z (up())**: Points upwards relative to the vehicle.
 * This is a common convention for ground robots.
 */
template <typename T>
struct Vector3D<flu, T> : public internal::VectorContainer3D<T> {
    typedef flu frame;  ///< The frame tag type for this specialization.
    using internal::VectorContainer3D<T>::VectorContainer3D;
    using scalar_type = T;
    using container_type =
        typename internal::VectorContainer3D<T>::container_type;  ///< The underlying data
                                                                  ///< container.

    /**
     * @brief Constructs a vector in the FLU frame.
     * @param f Forward component in meters (default: 0).
     * @param l Left component in meters (default: 0).
     * @param u Up component in meters (default: 0).
     */
    Vector3D(T f = 0, T l = 0, T u = 0) : internal::VectorContainer3D<T>({f, l, u}) {}

    // --- Semantic Accessors ---

    /// @brief Provides mutable access to the Forward component.
    T& forward() { return this->x(); }
    /// @brief Provides mutable access to the Left component.
    T& left() { return this->y(); }
    /// @brief Provides mutable access to the Up component.
    T& up() { return this->z(); }

    /// @brief Provides const access to the Forward component.
    T forward() const { return this->x(); }
    /// @brief Provides const access to the Left component.
    T left() const { return this->y(); }
    /// @brief Provides const access to the Up component.
    T up() const { return this->z(); }

    /**
     * @brief Provides const access to the underlying data container.
     * @return A const reference to the internal `std::array` storing the components in {Forward,
     * Left, Up} order.
     */
    const container_type& data() const { return internal::VectorContainer3D<T>::data; }
};

/* ========================================================================== */
/* =================== USER-FACING VECTOR OPERATORS ========================= */
/* ========================================================================== */

/**
 * @brief Performs compile-time safe, trait-aware vector addition.
 * @details This operator serves as the user-facing entry point for vector addition.
 * It enforces that both operands belong to the exact same reference frame, failing
 * at compile-time if they do not match.
 *
 * The actual calculation is delegated to the `internal::FramedVecOperator`, which performs a
 * component-wise addition that respects the `FrameTraits` of the frame.
 *
 * @tparam Frame The shared reference frame of the vectors.
 * @tparam T The scalar type of the vectors.
 * @param a The left-hand side vector.
 * @param b The right-hand side vector.
 * @return A new `Vector3D<Frame, T>` containing the trait-aware sum.
 * @warning A compile-time error will occur if `a` and `b` have different frame types.
 */

template <typename Frame1, typename Frame2, typename T>
Vector3D<Frame1, T> operator+(const Vector3D<Frame1, T>& a, const Vector3D<Frame2, T>& b) {
    internal::FrameValidator<Frame1, Frame2>::validate();
    return internal::FramedVecOperator<Vector3D<Frame1, T>>::add(a, b);
}

/**
 * @brief Performs compile-time safe, trait-aware vector sign change.
 * @details This operator serves as the user-facing entry point for vector sign change.
 *
 * @tparam Frame The shared reference frame of the vectors.
 * @tparam T The scalar type of the vectors.
 * @param v The vector to be sign-changed.
 * @return A new `Vector3D<Frame, T>` containing the trait-aware opposite vector.
 */
template <typename Frame, typename T>
Vector3D<Frame, T> operator-(const Vector3D<Frame, T>& v) {
    return static_cast<T>(-1.0) * v;
}

/**
 * @brief Performs compile-time safe, trait-aware vector subtraction.
 * @details This operator serves as the user-facing entry point for vector subtraction.
 * It enforces that both operands belong to the same reference frame.
 *
 * The calculation is delegated to the `internal::FramedVecOperator`, which will apply
 * specialized logic (e.g., shortest-angle difference for longitude) for non-linear
 * axes as defined in the frame's `FrameTraits`.
 *
 * @tparam Frame The shared reference frame of the vectors.
 * @tparam T The scalar type of the vectors.
 * @param a The minuend (vector to subtract from).
 * @param b The subtrahend (vector to subtract).
 * @return A new `Vector3D<Frame, T>` containing the trait-aware difference.
 * @warning A compile-time error will occur if `a` and `b` have different frame types.
 */
template <typename Frame1, typename Frame2, typename T>
Vector3D<Frame1, T> operator-(const Vector3D<Frame1, T>& a, const Vector3D<Frame2, T>& b) {
    internal::FrameValidator<Frame1, Frame2>::validate();
    return internal::FramedVecOperator<Vector3D<Frame1, T>>::sub(a, b);
}

/**
 * @brief Performs trait-aware scalar multiplication (vector * scalar).
 * @details Scales a vector by a scalar value. The operation is handled by the
 * `internal::FramedVecOperator`, which may apply normalization to angular components
 * after scaling.
 * @tparam Frame The reference frame of the vector.
 * @tparam T The scalar type.
 * @param a The vector to be scaled.
 * @param b The scalar multiplier.
 * @return A new `Vector3D<Frame, T>` representing the scaled vector.
 */
template <typename Frame, typename T>
Vector3D<Frame, T> operator*(const Vector3D<Frame, T>& a, T b) {
    return internal::FramedVecOperator<Vector3D<Frame, T>>::scalar_mul(a, b);
}

/**
 * @brief Performs trait-aware scalar multiplication (scalar * vector).
 * @details Scales a vector by a scalar value. This is the commutative version of
 * the `vector * scalar` operator.
 * @tparam Frame The reference frame of the vector.
 * @tparam T The scalar type.
 * @param a The scalar multiplier.
 * @param b The vector to be scaled.
 * @return A new `Vector3D<Frame, T>` representing the scaled vector.
 */
template <typename Frame, typename T>
Vector3D<Frame, T> operator*(T a, const Vector3D<Frame, T>& b) {
    return internal::FramedVecOperator<Vector3D<Frame, T>>::scalar_mul(a, b);
}

/**
 * @brief Performs trait-aware scalar division (vector / scalar).
 * @details Divides each component of the vector by a scalar value. The operation
 * is handled by the `internal::FramedVecOperator`.
 * @tparam Frame The reference frame of the vector.
 * @tparam T The scalar type.
 * @param a The vector dividend.
 * @param b The scalar divisor.
 * @return A new `Vector3D<Frame, T>` with each component divided by `b`.
 * @note This function does not check for division by zero.
 */
template <typename Frame, typename T>
Vector3D<Frame, T> operator/(const Vector3D<Frame, T>& a, T b) {
    return internal::FramedVecOperator<Vector3D<Frame, T>>::scalar_div(a, b);
}

/**
 * @brief Performs trait-aware, element-wise scalar division (scalar / vector).
 * @details This is an element-wise operation: `result[i] = a / b[i]`.
 * @tparam Frame The reference frame of the vector.
 * @tparam T The scalar type.
 * @param a The scalar dividend.
 * @param b The vector divisor.
 * @return A new `Vector3D<Frame, T>` containing the element-wise result.
 * @note This function does not check for division by zero in any of the vector's components.
 */
template <typename Frame, typename T>
Vector3D<Frame, T> operator/(T a, const Vector3D<Frame, T>& b) {
    return internal::FramedVecOperator<Vector3D<Frame, T>>::scalar_div(a, b);
}

/**
 * @brief Computes the geometric cross product of two vectors.
 * @details The cross product produces a new vector that is orthogonal to both input
 * vectors, following the right-hand rule.
 *
 * @tparam Frame The shared reference frame of the vectors.
 * @tparam T The scalar type.
 * @param a The left-hand side vector in the cross product (`a x b`).
 * @param b The right-hand side vector in the cross product (`a x b`).
 * @return `a x b`, a new `Vector3D<Frame, T>` representing the orthogonal vector.
 * @warning This operation is only physically meaningful for vectors in Cartesian
 * reference frames (e.g., `NED`, `FRD`, `ECEF`). Using it on non-Cartesian types
 * like `Vector3D<lla>` will compile but produce a mathematically nonsensical result.
 * A compile-time error will occur if `a` and `b` have different frame types.
 */
template <typename Frame1, typename Frame2, typename T>
Vector3D<Frame1, T> cross(const Vector3D<Frame1, T>& a, const Vector3D<Frame2, T>& b) {
    internal::FrameValidator<Frame1, Frame2>::validate();
    internal::FrameDirectionalAxisValidator<Frame1>::validate();
    return Vector3D<Frame1, T>(a.y() * b.z() - a.z() * b.y(), a.z() * b.x() - a.x() * b.z(),
                               a.x() * b.y() - a.y() * b.x());
}

/**
 * @brief Computes the geometric dot product of two vectors.
 *
 * @tparam Frame The shared reference frame of the vectors.
 * @tparam T The scalar type.
 * @param a The left-hand side vector in the cross product (`a x b`).
 * @param b The right-hand side vector in the cross product (`a x b`).
 * @return a scalar representing the dot product `a . b`.
 * @warning This operation is only physically meaningful for vectors in Cartesian
 * reference frames (e.g., `NED`, `FRD`, `ECEF`). Using it on non-Cartesian types
 * like `Vector3D<lla>` will compile but produce a mathematically nonsensical result.
 * A compile-time error will occur if `a` and `b` have different frame types.
 */
template <typename Frame1, typename Frame2, typename T>
Vector3D<Frame1, T> dot(const Vector3D<Frame1, T>& a, const Vector3D<Frame2, T>& b) {
    internal::FrameValidator<Frame1, Frame2>::validate();
    internal::FrameDirectionalAxisValidator<Frame1>::validate();
    return T(a.x() * b.x() + a.y() * b.y() + a.z() * b.z());
}

}  // namespace refx

#endif /* _REFX_GEOMETRY_VECTOR_ */
