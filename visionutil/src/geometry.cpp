#include "geometry.hpp"

namespace vision_geometry {

std::vector<float> getRelativePosition(
        float camera_offset_x,
        float camera_offset_y,
        float camera_offset_z,
        float camera_rotation_x,
        float camera_fov_h,
        float camera_fov_w,
        int camera_res_h,
        int camera_res_w,
        int pixel_row, int pixel_col, float depth) {

    std::vector<float> p(2);

    //float h_omega = camera_rotation_x - camera_res_h/2 + ((float)pixel_row)*(camera_fov_h/camera_res_h);
    float w_omega = ((float) pixel_col)*(camera_fov_w/camera_res_w) - camera_res_w/2;
    //h_omega = std::abs(h_omega);
    //float d_g = std::cos((h_omega*M_PI) / 180)*depth;
    float d_g = std::sqrt(std::pow(depth,2)-std::pow(camera_offset_z,2));

    w_omega = w_omega*(M_PI/180); //convert to radians

    float epsilon = 1e-5;
    if(std::abs(w_omega) < epsilon) {
        //object straight infront
        p[0] = camera_offset_x;
        p[1] = d_g + camera_offset_y;
        return p;
    }

    bool left = false;
    if(w_omega < 0) {
        left = true;
        w_omega = std::abs(w_omega);
    }

    float x = std::sin(w_omega)*d_g;
    float y = std::cos(w_omega)*d_g;

    if(!left) {
        x = -x;
    }

    x += camera_offset_x;
    y += camera_offset_y;
    p[0] = x;
    p[1] = y;
    return p;
}

}
