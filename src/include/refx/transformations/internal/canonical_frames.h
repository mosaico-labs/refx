#ifndef _REFX_TRANSFORMATIONS_INTERNAL_CANONICAL_FRAMES_
#define _REFX_TRANSFORMATIONS_INTERNAL_CANONICAL_FRAMES_

#include "../../frames/frames.h"
#include "../../frames/tags.h"

namespace refx {
namespace internal {

template <FrameTag Tag>
struct canonical_frame;

template <>
struct canonical_frame<FrameTag::LocalTangent> {
    using frame = ned;
};
template <>
struct canonical_frame<FrameTag::Body> {
    using frame = frd;
};
template <>
struct canonical_frame<FrameTag::Sensor> {
    using frame = imu;
};

}  // namespace internal
}  // namespace refx

#endif /* _REFX_TRANSFORMATIONS_INTERNAL_CANONICAL_FRAMES_ */
