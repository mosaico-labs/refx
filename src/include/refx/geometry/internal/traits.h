#ifndef _REFX_GEOMETRY_INTERNAL_TRAITS_
#define _REFX_GEOMETRY_INTERNAL_TRAITS_

#include "../../frames/axis.h"

namespace refx {
namespace internal {

template <typename ToFrame, typename FromFrame>
struct is_valid_rotation {
    static constexpr bool value = is_directional_axis_v<typename ToFrame::axis> &&
                                  is_directional_axis_v<typename FromFrame::axis>;
};

template <typename ToFrame, typename FromFrame>
using is_valid_transformation = is_valid_rotation<ToFrame, FromFrame>;

}  // namespace internal

template <typename ToFrame, typename FromFrame>
inline constexpr bool is_valid_rotation_v = internal::is_valid_rotation<ToFrame, FromFrame>::value;

template <typename ToFrame, typename FromFrame>
inline constexpr bool is_valid_transformation_v =
    internal::is_valid_transformation<ToFrame, FromFrame>::value;

}  // namespace refx

#endif /* _REFX_GEOMETRY_INTERNAL_TRAITS_ */
