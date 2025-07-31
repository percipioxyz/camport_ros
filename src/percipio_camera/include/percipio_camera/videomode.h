

#ifndef _VIDEO_MODEL_H_
#define _VIDEO_MODEL_H_

#include "TYApi.h"
#include "stdio.h"

namespace percipio
{
  typedef enum
  {
    PIXEL_FORMAT_UNDEFINED = 0,

	  // Depth
    PIXEL_FORMAT_DEPTH_1_MM = 100,

    // Color & ir
    PIXEL_FORMAT_RGB888 = 200,
    PIXEL_FORMAT_GRAY8 = 202,
    PIXEL_FORMAT_GRAY16 = 203,
  } PixelFormat;

  static PixelFormat         get_target_ros_pixel_format(uint32_t image_mode)
  {
    TY_PIXEL_FORMAT pixel_fmt = TYPixelFormat(image_mode);
    switch (pixel_fmt)
    {
      case TY_PIXEL_FORMAT_MONO:
        return PIXEL_FORMAT_GRAY8;
      case TY_PIXEL_FORMAT_BAYER8GRBG:
      case TY_PIXEL_FORMAT_BAYER8RGGB:
      case TY_PIXEL_FORMAT_BAYER8GBRG:
      case TY_PIXEL_FORMAT_BAYER8BGGR:
      case TY_PIXEL_FORMAT_CSI_BAYER10GRBG:
      case TY_PIXEL_FORMAT_CSI_BAYER10RGGB:
      case TY_PIXEL_FORMAT_CSI_BAYER10GBRG:
      case TY_PIXEL_FORMAT_CSI_BAYER10BGGR:
      case TY_PIXEL_FORMAT_CSI_BAYER12GRBG:
      case TY_PIXEL_FORMAT_CSI_BAYER12RGGB:
      case TY_PIXEL_FORMAT_CSI_BAYER12GBRG:
      case TY_PIXEL_FORMAT_CSI_BAYER12BGGR:
      case TY_PIXEL_FORMAT_YVYU:
      case TY_PIXEL_FORMAT_YUYV:
      case TY_PIXEL_FORMAT_RGB:
      case TY_PIXEL_FORMAT_BGR:
      case TY_PIXEL_FORMAT_JPEG:
      case TY_PIXEL_FORMAT_MJPG:
        return PIXEL_FORMAT_RGB888;

      case TY_PIXEL_FORMAT_CSI_MONO10:
      case TY_PIXEL_FORMAT_CSI_MONO12:
      case TY_PIXEL_FORMAT_MONO16:
      case TY_PIXEL_FORMAT_TOF_IR_MONO16:
        return PIXEL_FORMAT_GRAY16;

      case TY_PIXEL_FORMAT_DEPTH16:
      case TY_PIXEL_FORMAT_XYZ48:
        return PIXEL_FORMAT_DEPTH_1_MM;
      default:
        printf("Undefined pixel format: %x\n", pixel_fmt);
        return PIXEL_FORMAT_UNDEFINED;
    }
  }

  class VideoMode
  {
    public:
      VideoMode() {}
      VideoMode(int32_t pixelFormat, int32_t width, int32_t height) : targetPixelFormat(pixelFormat), resolutionX(width), resolutionY(height) {}
      /*
      VideoMode(uint32_t img_mode)
      {
        image_mode  = img_mode;
	      resolutionX = TYImageWidth(img_mode);
	      resolutionY = TYImageHeight(img_mode);
        targetPixelFormat = get_target_ros_pixel_format(img_mode);
      }
      */

      VideoMode& operator=(const VideoMode& other)
      {
  	    setPixelFormat(other.getPixelFormat());
  	    setResolution(other.getResolutionX(), other.getResolutionY());
  	    setFps(other.getFps());
    		return *this;
      }

      int32_t getResolutionX() const { return resolutionX; }
      int32_t getResolutionY() const {return resolutionY;}
      int32_t getFps() const { return fps; }
      int32_t getPixelFormat() const { return this->targetPixelFormat;}
      //uint32_t getImageMode() const { return this->image_mode; }

      void setResolution(int32_t resolutionX, int32_t resolutionY)
      {
        this->resolutionX = resolutionX;
        this->resolutionY = resolutionY;
      }

      void setPixelFormat(int32_t format) { this->targetPixelFormat = format; }
      void setFps(int fps) { this->fps = fps; }
      
    private:
      int32_t resolutionX = 640;
      int32_t resolutionY = 480;
      int32_t  targetPixelFormat;
      int32_t fps = 30;
      //uint32_t image_mode;

  };
}


#endif