#ifndef PERCIPIO_VIDEO_MODE_H_
#define PERCIPIO_VIDEO_MODE_H_

#include <cstddef>
#include <ostream>

namespace percipio_wrapper
{

// copied from OniEnums.h
typedef enum
{
        // Depth
        PIXEL_FORMAT_DEPTH_1_MM = 100,
        PIXEL_FORMAT_DEPTH_100_UM = 101,
        PIXEL_FORMAT_SHIFT_9_2 = 102,
        PIXEL_FORMAT_SHIFT_9_3 = 103,

        // Color
        PIXEL_FORMAT_RGB888 = 200,
        PIXEL_FORMAT_YUV422 = 201,
        PIXEL_FORMAT_GRAY8 = 202,
        PIXEL_FORMAT_GRAY16 = 203,
        PIXEL_FORMAT_JPEG = 204,
} PixelFormat;

struct PercipioVideoMode
{
  std::size_t x_resolution_;
  std::size_t y_resolution_;
  double frame_rate_;
  PixelFormat pixel_format_;
};

std::ostream& operator << (std::ostream& stream, const PercipioVideoMode& video_mode);

bool operator==(const PercipioVideoMode& video_mode_a, const PercipioVideoMode& video_mode_b);
bool operator!=(const PercipioVideoMode& video_mode_a, const PercipioVideoMode& video_mode_b);

}

#endif
