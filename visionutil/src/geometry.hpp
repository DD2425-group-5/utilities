#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include <vector>
#include <cmath>
#define _USE_MATH_DEFINES
#include <math.h>

namespace vision_geometry {

std::vector<float> getRelativePosition(float camera_offset_x,
                                     float camera_offset_y,
                                     float camera_offset_z,
                                     float camera_rotation_x,
                                     float camera_fov_h,
                                     float camera_fov_w,
                                     int camera_res_h,
                                     int camera_res_w,
                                     int pixel_row, int pixel_col, float depth);

}

#endif // GEOMETRY_HPP
