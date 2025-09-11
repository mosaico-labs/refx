#ifndef _INCLUDE_MY_ROBOT_FRAMES_
#define _INCLUDE_MY_ROBOT_FRAMES_

// Include 'developer-level' headers
#include <refx/frames/axis.h>
#include <refx/frames/tags.h>

namespace my_robot {

// A custom frame for a 2D laser scanner.
struct laser_scanner {
    static constexpr auto name = "laser_scanner";
    using axis = refx::axis_flu;  // Use a standard DirectionalAxis
    static constexpr refx::FrameTag tag = refx::FrameTag::Sensor;
};

}  // namespace my_robot

#endif /* _INCLUDE_MY_ROBOT_FRAMES_ */
