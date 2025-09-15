#ifndef _REFX_FRAMES_INTERNAL_VALIDATORS_
#define _REFX_FRAMES_INTERNAL_VALIDATORS_

#include "../internal/traits.h"  // For is_semantic_axis_v, is_directional_axis_v
#include "../tags.h"

/**
 * @file validators.h
 * @brief Defines a set of internal, compile-time validation utilities.
 *
 * @details This internal header provides a collection of class templates whose
 * purpose is to enforce preconditions on frame transformations at compile time.
 * They are the core mechanism for ensuring that high-level APIs like `frame_cast`
 * and `frame_transform` are used correctly.
 *
 * The architectural pattern is to **centralize validation logic**. Instead of
 * scattering `static_assert` statements throughout the library's transformation
 * engines, these validator structs provide a single point of truth for each
 * specific precondition.
 *
 * This design improves maintainability and provides consistent, user-friendly
 * error messages that guide developers to the correct API if they make a mistake.
 * These validators are a key component of the library's "correct by construction"
 * philosophy, turning potential runtime errors or silent failures into clear
 * compile-time errors.
 */

namespace refx {
namespace internal {

enum class ValidateCondition { AND, OR, XOR };

template <typename frameA, typename frameB>
struct FrameValidator {
    // this function is evaluated at compile-time.
    static void validate() {
        static_assert(std::is_same_v<frameA, frameB>,
                      "This operation is allowed only for object in the same frame.");
    }
};

/**
 * @brief Enforces that the first two frame-tags are equal to the third.
 * @details This validator is used to ensure that a transformation is only
 * attempted between frames that are conceptually related (e.g., both are
 * `Body` frames, or both are `LocalTangent`). It prevents meaningless
 * operations like trying to cast a `Geocentric` frame to a `Body` frame
 * without a full `frame_transform`.
 */
template <FrameTag tagA, FrameTag tagB, FrameTag tagC = tagA,
          ValidateCondition C = ValidateCondition::AND>
struct FrameTagValidator;

// FrameTagValidator specializations
template <FrameTag tagA, FrameTag tagB, FrameTag tagC>
struct FrameTagValidator<tagA, tagB, tagC, ValidateCondition::AND> {
    /**
     * @brief Performs the compile-time check.
     */
    static void validate() {
        static_assert(tagA == tagC && tagB == tagC,
                      "This operation is only allowed between frames of the same FrameTag.");
    }
};

template <FrameTag tagA, FrameTag tagB, FrameTag tagC>
struct FrameTagValidator<tagA, tagB, tagC, ValidateCondition::OR> {
    /**
     * @brief Performs the compile-time check.
     */
    static void validate() {
        static_assert(tagA == tagC || tagB == tagC,
                      "This operation is only allowed when at least one among tagA and tagB is "
                      "equal to tagC.");
    }
};

template <FrameTag tagA, FrameTag tagB, FrameTag tagC>
struct FrameTagValidator<tagA, tagB, tagC, ValidateCondition::XOR> {
    /**
     * @brief Performs the compile-time check.
     */
    static void validate() {
        static_assert(
            tagA == tagC ^ tagB == tagC,
            "This operation is only allowed when only one between tagA and tagB is equal to tagC");
    }
};

/**
 * @brief Validates that two axis are equal.
 */
template <typename axisA, typename axisB>
struct AxisValidator {
    /**
     * @brief Performs the compile-time check.
     */
    static void validate() {
        static_assert(
            std::is_same_v<axisA, axisB>,
            "The two axis must be equal. Maybe you passed a frame with incompatible axis");
    }
};

/**
 * @brief Validates that a transformation involves at least one non-Cartesian
 * (`SemanticAxis`) frame.
 * @details This validator is a precondition for the `frame_transform` API.
 * `frame_transform` is designed for complex projections and conversions that
 * require a physical model (e.g., LLA -> NED). Such operations are only
 * meaningful if at least one of the frames is non-Cartesian.
 *
 * If a developer attempts to use `frame_transform` with two Cartesian frames
 * (e.g., NED -> ENU), this validator will trigger a compile-time error,
 * guiding them to use the more appropriate and efficient `frame_cast` API.
 */
template <typename frameA, typename frameB = frameA, ValidateCondition C = ValidateCondition::AND>
struct FrameSemanticAxisValidator;

// FrameSemanticAxisValidator specializations
template <typename frameA, typename frameB>
struct FrameSemanticAxisValidator<frameA, frameB, ValidateCondition::AND> {
    /**
     * @brief Performs the compile-time check.
     */
    static void validate() {
        static_assert(
            is_semantic_axis_v<typename frameA::axis> && is_semantic_axis_v<typename frameB::axis>,
            "This operation is only allowed between semantic axis frames "
            "(Non-cartesian frame). Maybe you wanted to call the frame_cast API.");
    }
};

template <typename frameA, typename frameB>
struct FrameSemanticAxisValidator<frameA, frameB, ValidateCondition::OR> {
    /**
     * @brief Performs the compile-time check.
     */
    static void validate() {
        static_assert(
            is_semantic_axis_v<typename frameA::axis> || is_semantic_axis_v<typename frameB::axis>,
            "This operation is only allowed whean at least one of the frames are semantic "
            "(Non-cartesian frame). Maybe you wanted to call the frame_cast API.");
    }
};
template <typename frameA, typename frameB>
struct FrameSemanticAxisValidator<frameA, frameB, ValidateCondition::XOR> {
    /**
     * @brief Performs the compile-time check.
     */
    static void validate() {
        static_assert(
            is_semantic_axis_v<typename frameA::axis> != is_semantic_axis_v<typename frameB::axis>,
            "This operation is only allowed when only one the frames is semantic ");
    }
};

/**
 * @brief Validates that a conversion is performed exclusively between
 * Cartesian (`DirectionalAxis`) frames.
 * @details This validator is a precondition for the `frame_cast` API,
 * that is only well-defined for Cartesian coordinate systems.
 *
 * If a developer attempts to `frame_cast` a non-Cartesian frame (e.g., LLA),
 * this validator will trigger a compile-time error, guiding them to use the
 * more appropriate `frame_transform` API which can handle the necessary
 * physical projections.
 */
template <typename frameA, typename frameB = frameA, ValidateCondition C = ValidateCondition::AND>
struct FrameDirectionalAxisValidator;

// FrameDirectionalAxisValidator specializations
template <typename frameA, typename frameB>
struct FrameDirectionalAxisValidator<frameA, frameB, ValidateCondition::AND> {
    /**
     * @brief Performs the compile-time check.
     */
    static void validate() {
        static_assert(is_directional_axis_v<typename frameA::axis> &&
                          is_directional_axis_v<typename frameB::axis>,
                      "This operation is only allowed between directional axis frames "
                      "(Cartesian frame). Maybe you wanted to call the frame_transform API.");
    }
};

template <typename frameA, typename frameB>
struct FrameDirectionalAxisValidator<frameA, frameB, ValidateCondition::OR> {
    /**
     * @brief Performs the compile-time check.
     */
    static void validate() {
        static_assert(is_directional_axis_v<typename frameA::axis> ||
                          is_directional_axis_v<typename frameB::axis>,
                      "This operation is only allowed between directional axis frames "
                      "(Cartesian frame). Maybe you wanted to call the frame_transform API.");
    }
};

template <typename frameA, typename frameB>
struct FrameDirectionalAxisValidator<frameA, frameB, ValidateCondition::XOR> {
    /**
     * @brief Performs the compile-time check.
     */
    static void validate() {
        static_assert(is_directional_axis_v<typename frameA::axis> !=
                          is_directional_axis_v<typename frameB::axis>,
                      "This operation is only allowed when only one the frames is directional "
                      "(Cartesian frame). Maybe you wanted to call the frame_transform API.");
    }
};

}  // namespace internal
}  // namespace refx

#endif /* _REFX_FRAMES_INTERNAL_VALIDATORS_ */
