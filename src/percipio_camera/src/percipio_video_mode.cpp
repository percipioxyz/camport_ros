#include "percipio_camera/percipio_video_mode.h"

namespace percipio_wrapper
{


std::ostream& operator << (std::ostream& stream, const PercipioVideoMode& video_mode) {
  stream << "Resolution: " << (int)video_mode.x_resolution_ << "x" << (int)video_mode.y_resolution_ <<
                              "@" << video_mode.frame_rate_ <<
                              "Hz Format: ";

  switch (video_mode.pixel_format_)
  {
    case PIXEL_FORMAT_DEPTH_1_MM:
      stream << "Depth 1mm";
      break;
    case PIXEL_FORMAT_DEPTH_100_UM:
      stream << "Depth 100um";
      break;
    case PIXEL_FORMAT_SHIFT_9_2:
      stream << "Shift 9/2";
      break;
    case PIXEL_FORMAT_SHIFT_9_3:
      stream << "Shift 9/3";
      break;
    case PIXEL_FORMAT_RGB888:
      stream << "RGB888";
      break;
    case PIXEL_FORMAT_YUV422:
      stream << "YUV422";
      break;
    case PIXEL_FORMAT_GRAY8:
      stream << "Gray8";
      break;
    case PIXEL_FORMAT_GRAY16:
      stream << "Gray16";
      break;
    case PIXEL_FORMAT_JPEG:
      stream << "JPEG";
      break;

    default:
      break;
  }

  return stream;
}

bool operator==(const PercipioVideoMode& video_mode_a, const PercipioVideoMode& video_mode_b)
{
  return (video_mode_a.x_resolution_==video_mode_b.x_resolution_) &&
         (video_mode_a.y_resolution_==video_mode_b.y_resolution_) &&
         (video_mode_a.frame_rate_  ==video_mode_b.frame_rate_)   &&
         (video_mode_a.pixel_format_==video_mode_b.pixel_format_);
}

bool operator!=(const PercipioVideoMode& video_mode_a, const PercipioVideoMode& video_mode_b)
{
  return !(video_mode_a==video_mode_b);
}

} //namespace percipio_wrapper
