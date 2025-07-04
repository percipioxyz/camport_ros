/*
 * @Description: 
 * @Author: zxy
 * @Date: 2023-07-18 15:55:24
 * @LastEditors: zxy
 * @LastEditTime: 2024-05-30 15:01:39
 */

#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <map>
#include <cmath>
#include <numeric>
#include <algorithm>
//#include <math.h>
#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

#include "percipio_interface.h"
#include "TYApi.h"
#include "TYImageProc.h"
#include "TYCoordinateMapper.h"

#define MAX_DEPTH 0x10000 

static void BGRToRGB(const void* bgrFrame, int width, int height, void* rgbFrame)
{
  uint8_t* pBGR = (uint8_t*)bgrFrame;
  uint8_t* pRGB = (uint8_t*)rgbFrame;

  uint8_t r, g, b;
  for (int i = 0; i < height * width; i++)
  {
    b = *pBGR++;
    g = *pBGR++;
    r = *pBGR++;
		
    *pRGB++ = r;
    *pRGB++ = g;
    *pRGB++ = b;
  }
}

class distortion_data {
  public:
    distortion_data(const TY_CAMERA_CALIB_INFO& calib, const cv::Mat& x, const cv::Mat& y);
    distortion_data(const distortion_data& data);
    ~distortion_data();

    const int get_map_width() const;
    const int get_map_height() const;

    const std::vector<float>& get_map_x() const;
    const std::vector<float>& get_map_y() const;

    void* dataX() { return map_x.data(); }
    void* dataY() { return map_y.data(); }
    
    const TY_CAMERA_CALIB_INFO& get_calib_data() const;

  private:
    TY_CAMERA_CALIB_INFO calib_data;
    int map_size_width;
    int map_size_height;
    std::vector<float> map_x;
    std::vector<float> map_y;
};

distortion_data::distortion_data(const TY_CAMERA_CALIB_INFO& calib, const cv::Mat& x, const cv::Mat& y)
{
  if(x.size() != y.size())
  {
    ROS_ERROR("Invalid distortion map");
    return;
  }

  calib_data = calib;

  if (x.depth() != CV_32F || y.depth() != CV_32F) {
    throw std::runtime_error("Only floating-point matrices supported");
  }
  
  const int x_channels = x.channels();
  const size_t x_totalElements = x.total() * x_channels;

  const int y_channels = y.channels();
  const size_t y_totalElements = y.total() * y_channels;
    
    // 创建目标vector并直接复制内存
  map_x.resize(x_totalElements);
  memcpy(map_x.data(), x.data, x_totalElements * sizeof(float));

  map_y.resize(y_totalElements);
  memcpy(map_y.data(), y.data, y_totalElements * sizeof(float));
  
  map_size_width = x.cols;
  map_size_height = x.rows;
}

distortion_data::distortion_data(const distortion_data& data)
{
  calib_data = data.get_calib_data();
  map_x = data.get_map_x();
  map_y = data.get_map_y();

  map_size_width = data.map_size_width;
  map_size_height = data.map_size_height;
}

distortion_data::~distortion_data()
{
  //
}

const TY_CAMERA_CALIB_INFO& distortion_data::get_calib_data() const
{
  return calib_data;
}

const std::vector<float>& distortion_data::get_map_x() const
{
  return map_x;
}

const std::vector<float>& distortion_data::get_map_y() const
{
  return map_y;
}

const int distortion_data::get_map_width() const
{
  return map_size_width;
}

const int distortion_data::get_map_height() const
{
  return map_size_height;
}

class ImgProc 
{
public:
    static inline void parseXYZ48(int16_t* src, int16_t* dst, int width, int height, float f_scale_unit)
    {
      for (int pix = 0; pix < width*height; pix++) {
          dst[pix] =(int16_t)(*(src + 3*pix + 2) * f_scale_unit + 0.5f);
      }
    }
    static inline int decodeCsiRaw10(uint8_t* src, unsigned short* dst, int width, int height)
    {
        if(width & 0x3) {
            return -1;
        }
    
        int raw10_line_size = 5 * width / 4;
        for(size_t i = 0, j = 0; i < raw10_line_size * height; i+=5, j+=4)
        {
            //[A2 - A9] | [B2 - B9] | [C2 - C9] | [D2 - D9] | [A0A1-B0B1-C0C1-D0D1]
            dst[j + 0] = ((uint16_t)src[i + 0] << 2) | ((src[i + 4] & 0x3)  >> 0);
            dst[j + 1] = ((uint16_t)src[i + 1] << 2) | ((src[i + 4] & 0xc)  >> 2);
            dst[j + 2] = ((uint16_t)src[i + 2] << 2) | ((src[i + 4] & 0x30) >> 4);
            dst[j + 3] = ((uint16_t)src[i + 3] << 2) | ((src[i + 4] & 0xc0) >> 6);
        }
        return 0;
    }

    static inline int decodePacketRaw10(unsigned char* src, unsigned char* dst, int width, int height)
    {
        if(width & 0x3) {
            return -1;
        }

        int raw10_line_size = 5 * width / 4;
        //    byte0  -        byte1    -      byte2     -     byte3      -  byte4
        // | A7 - A0 | B5 - B0 A9 - A8 | C3 -C0 B9 - B6 | D1 - D0 C9 -C4 | D9 -D2|
        for(size_t i = 0; i < raw10_line_size * height; i+=5)
        {
            //high 8bit
            dst[0] = (src[1] << 6) | (src[0] >> 2);
            dst[1] = (src[2] << 4) | (src[1] >> 4);
            dst[2] = (src[3] << 2) | (src[2] >> 6);
            dst[3] = src[4];

            src+=5;
            dst+=4;
        }
        return 0;
    }

    static inline int decodeCsiRaw12(uint8_t* src, uint16_t* dst, int width, int height)
    {
        if(width & 0x1) {
            return -1;
        }
        int raw12_line_size = 3 * width / 2;
        for(size_t i = 0, j = 0; i < raw12_line_size * height; i+=3, j+=2)
        {
            //[A4 - A11] | [B4 - B11] | [A0A1A2A3-B0B1B2B3]
            dst[j + 0] = ((uint16_t)src[i + 0] << 4) | ((src[i + 2] & 0x0f)  >> 0);
            dst[j + 1] = ((uint16_t)src[i + 1] << 4) | ((src[i + 2] & 0xf0)  >> 4);
        }
        return 0;
    }

    static inline int parseCsiRaw10(uint8_t* src, uint16_t* dst, int width, int height)
    {
        decodeCsiRaw10(src, dst, width, height);
        return 0;
    }

    static inline int parsePacketRaw10(unsigned char* src, cv::Mat &dst, int width, int height)
    {
        dst = cv::Mat(height, width, CV_8U);
        decodePacketRaw10(src, dst.data, width, height);
        return 0;
    }

    static inline int parseCsiRaw12(uint8_t* src, uint16_t* dst, int width, int height)
    {
        decodeCsiRaw12(src, dst, width, height);
        return 0;
    }

    static int doDepthUndistortion(const TY_CAMERA_CALIB_INFO* depth_calib,
                                percipio::VideoFrameData* src, 
                                percipio::VideoFrameData* dst)
    {
      if(src->getPixelFormat() != TYPixelFormatCoord3D_C16) {
        printf("%s <%d> Invalid pixel format!\n", __FILE__, __LINE__);
        return-1;
      }

      dst->setTimestamp(src->getTimestamp());
      dst->setFrameIndex(src->getFrameIndex());
      dst->setComponentID(src->getComponentID());
      dst->setWidth(src->getWidth());
      dst->setHeight(src->getHeight());
      dst->setPixelFormat(src->getPixelFormat());
      dst->Resize(src->getDataSize());

      TY_IMAGE_DATA src_image, dst_image;
      src_image.width  = src->getWidth();
      src_image.height = src->getHeight();
      src_image.pixelFormat = src->getPixelFormat();
      src_image.buffer = src->getData();

      dst_image.width  = dst->getWidth();
      dst_image.height = dst->getHeight();
      dst_image.pixelFormat = dst->getPixelFormat();
      dst_image.buffer = dst->getData();

      TY_STATUS err = TYUndistortImage(depth_calib, &src_image, nullptr, &dst_image);
      if(err) {
        ROS_WARN("TYUndistortImage fail!");
      }
      return err;
    }

    static int doRGBUndistortion(const TY_CAMERA_CALIB_INFO* color_calib, 
                                percipio::VideoFrameData* src, 
                                percipio::VideoFrameData* dst)
    {
      if(src->getPixelFormat() != TYPixelFormatRGB8 &&
         src->getPixelFormat() != TYPixelFormatMono8) {
        printf("%s <%d> Invalid pixel format!\n", __FILE__, __LINE__);
        return-1;
      }

      dst->setTimestamp(src->getTimestamp());
      dst->setFrameIndex(src->getFrameIndex());
      dst->setComponentID(src->getComponentID());
      dst->setWidth(src->getWidth());
      dst->setHeight(src->getHeight());
      dst->setPixelFormat(src->getPixelFormat());
      dst->Resize(src->getDataSize());

      TY_IMAGE_DATA src_image, dst_image;
      src_image.width  = src->getWidth();
      src_image.height = src->getHeight();
      src_image.pixelFormat = src->getPixelFormat();
      src_image.buffer = src->getData();

      dst_image.width  = dst->getWidth();
      dst_image.height = dst->getHeight();
      dst_image.pixelFormat = dst->getPixelFormat();
      dst_image.buffer = dst->getData();

      TY_STATUS err = TYUndistortImage(color_calib, &src_image, nullptr, &dst_image);
      if(err) {
        ROS_WARN("TYUndistortImage fail!");
      }
      return err;
    }

    static int MapDepthImageToColorCoordinate(const TY_CAMERA_CALIB_INFO* depth_calib, 
                                              const TY_CAMERA_CALIB_INFO* color_calib, 
                                              int target_width,
                                              int target_height,
                                              percipio::VideoFrameData* src, 
                                              percipio::VideoFrameData* dst,
                                              float f_scale_unit)
    {
      if(src->getPixelFormat() != TYPixelFormatCoord3D_C16) {
        printf("%s <%d> Invalid pixel format!\n", __FILE__, __LINE__);
        return-1;
      }

      dst->setTimestamp(src->getTimestamp());
      dst->setFrameIndex(src->getFrameIndex());
      dst->setPixelFormat(src->getPixelFormat());
      if((target_width != 0) && (target_height != 0)) {
        dst->setWidth(target_width);
        dst->setHeight(target_height);
        dst->Resize(2 * target_width * target_height);
      } else {
        dst->setWidth(src->getWidth());
        dst->setHeight(src->getHeight());
        dst->Resize(2 * src->getWidth() * src->getHeight());
      }

      TY_IMAGE_DATA src_image, dst_image;
      src_image.width  = src->getWidth();
      src_image.height = src->getHeight();
      src_image.pixelFormat = src->getPixelFormat();
      src_image.buffer = src->getData();

      dst_image.width  = dst->getWidth();
      dst_image.height = dst->getHeight();
      dst_image.pixelFormat = dst->getPixelFormat();
      dst_image.buffer = dst->getData();

      return  TYMapDepthImageToColorCoordinate(
                  depth_calib,
                  src_image.width, src_image.height, (const uint16_t*)src_image.buffer,
                  color_calib,
                  dst_image.width, dst_image.height, (uint16_t* )dst_image.buffer, 
                  f_scale_unit);
    }

    static int MapXYZ48ToColorCoordinate(const TY_CAMERA_CALIB_INFO* depth_calib, 
                                              const TY_CAMERA_CALIB_INFO* color_calib, 
                                              int target_width,
                                              int target_height,
                                              percipio::VideoFrameData* src, 
                                              percipio::VideoFrameData* dst,
                                              float f_scale_unit)
    {
      //TODO
    }

    static int cvtColor(percipio::VideoFrameData& src, percipio::VideoFrameData& dst)
    {
      int i, j;
      dst.setTimestamp(src.getTimestamp());
      dst.setWidth(src.getWidth());
      dst.setHeight(src.getHeight());
      dst.setFrameIndex(src.getFrameIndex());
      dst.setComponentID(src.getComponentID());
      dst.setPixelFormat(TYPixelFormatRGB8);
      dst.Resize(3 * src.getWidth() * src.getHeight());

      int width = src.getWidth();
      int height = src.getHeight();
      int size = src.getDataSize();
      void* src_buffer = src.getData();
      void* dst_buffer = dst.getData();

      cv::Mat src_mat, dst_mat;
      switch (src.getPixelFormat()) {
        case TYPixelFormatYUV422_8: {
          src_mat = cv::Mat(height, width, CV_8UC2, src_buffer);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_YUV2RGB_YUYV);
          break;
        }
        case TYPixelFormatYUV422_8_UYVY: {
          src_mat = cv::Mat(height, width, CV_8UC2, src_buffer);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_YUV2RGB_UYVY);
          break;
        }    
        case TYPixelFormatJPEG: {
          std::vector<uchar> _v((uchar*)src_buffer, (uchar*)src_buffer + size);
          cv::Mat bgr = cv::imdecode(_v, cv::IMREAD_COLOR);
          cv::cvtColor(bgr, dst_mat, cv::COLOR_BGR2RGB);
          if(!dst_mat.empty() && (width == dst_mat.cols) && (height == dst_mat.rows)){
            memcpy(dst_buffer, dst_mat.data, 3 * width * height);
          } else {
            ROS_WARN("JPEG decode error!");
          }
          break;
        }
        case TYPixelFormatBayerGBRG8: {
          src_mat = cv::Mat(height, width, CV_8U, src_buffer);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BayerGR2RGB);
          break;
        }
        case TYPixelFormatBayerBGGR8: {
          src_mat = cv::Mat(height, width, CV_8U, src_buffer);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BayerRG2RGB);
          break;
        }
        case TYPixelFormatBayerGRBG8: {
          src_mat = cv::Mat(height, width, CV_8U, src_buffer);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BayerGB2RGB);
          break;
        }
        case TYPixelFormatBayerRGGB8: {
          src_mat = cv::Mat(height, width, CV_8U, src_buffer);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BayerBG2RGB);
          break;
        }
        case TYPixelFormatBGR8: {
          src_mat = cv::Mat(height, width, CV_8UC3, src_buffer);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BGR2RGB);
          break;
        }
        case TYPixelFormatRGB8: {
          memcpy(dst_buffer, src_buffer, 3*width*height);
        }
        case TYPixelFormatMono8: {
          src_mat = cv::Mat(height, width, CV_8U, src_buffer);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_GRAY2RGB);
          break;
        }
        case TYPixelFormatMono10: {
          std::vector<uint16_t> mono10(width * height);
          std::vector<uint8_t> mono8(width * height);
          parseCsiRaw10((uint8_t*)src_buffer, &mono10[0], width, height);
          for(size_t idx = 0; idx < mono10.size(); idx++) {
            mono8[idx] = static_cast<uint8_t>(mono10[idx] >> 2);
          }
          src_mat = cv::Mat(height, width, CV_8U, &mono8[0]);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_GRAY2RGB);
          break;
        }
        case TYPixelFormatBayerGBRG10: {
          std::vector<uint16_t> bayer10(width * height);
          std::vector<uint8_t> bayer8(width * height);
          parseCsiRaw10((uint8_t*)src_buffer, &bayer10[0], width, height);
          for(size_t idx = 0; idx < bayer10.size(); idx++) {
            bayer8[idx] = static_cast<uint8_t>(bayer10[idx] >> 2);
          }
          src_mat = cv::Mat(height, width, CV_8U, &bayer8[0]);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BayerGR2RGB);
          break;
        }
        case TYPixelFormatBayerBGGR10: {
          std::vector<uint16_t> bayer10(width * height);
          std::vector<uint8_t> bayer8(width * height);
          parseCsiRaw10((uint8_t*)src_buffer, &bayer10[0], width, height);
          for(size_t idx = 0; idx < bayer10.size(); idx++) {
            bayer8[idx] = static_cast<uint8_t>(bayer10[idx] >> 2);
          }
          src_mat = cv::Mat(height, width, CV_8U, &bayer8[0]);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BayerRG2RGB);
          break;
        }
        case TYPixelFormatBayerGRBG10: {
          std::vector<uint16_t> bayer10(width * height);
          std::vector<uint8_t> bayer8(width * height);
          parseCsiRaw10((uint8_t*)src_buffer, &bayer10[0], width, height);
          for(size_t idx = 0; idx < bayer10.size(); idx++) {
            bayer8[idx] = static_cast<uint8_t>(bayer10[idx] >> 2);
          }
          src_mat = cv::Mat(height, width, CV_8U, &bayer8[0]);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BayerGB2RGB);
          break;
        }
        case TYPixelFormatBayerRGGB10: {
          std::vector<uint16_t> bayer10(width * height);
          std::vector<uint8_t> bayer8(width * height);
          parseCsiRaw10((uint8_t*)src_buffer, &bayer10[0], width, height);
          for(size_t idx = 0; idx < bayer10.size(); idx++) {
            bayer8[idx] = static_cast<uint8_t>(bayer10[idx] >> 2);
          }
          src_mat = cv::Mat(height, width, CV_8U, &bayer8[0]);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BayerBG2RGB);
          break;
        }
        case TYPixelFormatPacketMono10:
        {
          parsePacketRaw10((uchar*)src_buffer, src_mat, width, height);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_GRAY2BGR);
          break;
        }
        case TYPixelFormatPacketBayerGBRG10:
        {
          parsePacketRaw10((uchar*)src_buffer, src_mat, width, height);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BayerGR2RGB);
          break;
        }
        case TYPixelFormatPacketBayerBGGR10:
        {
          parsePacketRaw10((uchar*)src_buffer, src_mat, width, height);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BayerRG2RGB);
          break;
        }
        case TYPixelFormatPacketBayerGRBG10:
        {
          parsePacketRaw10((uchar*)src_buffer, src_mat, width, height);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BayerGB2RGB);
          break;
        }
        case TYPixelFormatPacketBayerRGGB10:
        {
          parsePacketRaw10((uchar*)src_buffer, src_mat, width, height);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BayerBG2RGB);
          break;
        }
        case TYPixelFormatMono12: {
          std::vector<uint16_t> mono12(width * height);
          std::vector<uint8_t> mono8(width * height);
          parseCsiRaw12((uint8_t*)src_buffer, &mono12[0], width, height);
          for(size_t idx = 0; idx < mono12.size(); idx++) {
            mono8[idx] = static_cast<uint8_t>(mono12[idx] >> 4);
          }
          src_mat = cv::Mat(height, width, CV_8U, &mono8[0]);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_GRAY2RGB);
          break;
        }
        case TYPixelFormatBayerGBRG12: {
          std::vector<uint16_t> bayer12(width * height);
          std::vector<uint8_t> bayer8(width * height);
          parseCsiRaw12((uint8_t*)src_buffer, &bayer12[0], width, height);
          for(size_t idx = 0; idx < bayer12.size(); idx++) {
            bayer8[idx] = static_cast<uint8_t>(bayer12[idx] >> 4);
          }
          src_mat = cv::Mat(height, width, CV_8U, &bayer8[0]);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BayerGR2RGB);
          break;
        }
        case TYPixelFormatBayerBGGR12: {
          std::vector<uint16_t> bayer12(width * height);
          std::vector<uint8_t> bayer8(width * height);
          parseCsiRaw12((uint8_t*)src_buffer, &bayer12[0], width, height);
          for(size_t idx = 0; idx < bayer12.size(); idx++) {
            bayer8[idx] = static_cast<uint8_t>(bayer12[idx] >> 4);
          }
          src_mat = cv::Mat(height, width, CV_8U, &bayer8[0]);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BayerRG2RGB);
          break;
        }
        case TYPixelFormatBayerGRBG12: {
          std::vector<uint16_t> bayer12(width * height);
          std::vector<uint8_t> bayer8(width * height);
          parseCsiRaw12((uint8_t*)src_buffer, &bayer12[0], width, height);
          for(size_t idx = 0; idx < bayer12.size(); idx++) {
            bayer8[idx] = static_cast<uint8_t>(bayer12[idx] >> 4);
          }
          src_mat = cv::Mat(height, width, CV_8U, &bayer8[0]);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BayerGB2RGB);
          break;
        }
        case TYPixelFormatBayerRGGB12: {
          std::vector<uint16_t> bayer12(width * height);
          std::vector<uint8_t> bayer8(width * height);
          parseCsiRaw12((uint8_t*)src_buffer, &bayer12[0], width, height);
          for(size_t idx = 0; idx < bayer12.size(); idx++) {
            bayer8[idx] = static_cast<uint8_t>(bayer12[idx] >> 4);
          }
          src_mat = cv::Mat(height, width, CV_8U, &bayer8[0]);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BayerBG2RGB);
          break;
        }    
        case TYPixelFormatMono16:
        case TYPixelFormatTofIRFourGroupMono16: {
          uint16_t* ptr = (uint16_t*)src_buffer;
          std::vector<uint8_t> mono8(width * height);
          for(size_t idx = 0; idx < width * height; idx++) {
            mono8[idx] = ptr[idx] >> 8;
          }
          src_mat = cv::Mat(height, width, CV_8U, &mono8[0]);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_GRAY2RGB);
          break;
        }
        case TYPixelFormatYCbCr420_8_YY_CbCr_Semiplanar: {
          src_mat = cv::Mat(height + height / 2, width, CV_8UC1, src_buffer);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_YUV2RGB_NV12);
          break;
        }
        case TYPixelFormatYCbCr420_8_YY_CrCb_Semiplanar: {
          src_mat = cv::Mat(height + height / 2, width, CV_8UC1, src_buffer);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_YUV2RGB_NV21);
          break;
        }
        case TYPixelFormatYCbCr420_8_YY_CbCr_Planar: {
          src_mat = cv::Mat(height + height / 2, width, CV_8UC1, src_buffer);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_YUV2RGB_I420);
          break;
        }
        case TYPixelFormatYCbCr420_8_YY_CrCb_Planar: {
          src_mat = cv::Mat(height + height / 2, width, CV_8UC1, src_buffer);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_YUV420p2RGB);
          break;
        }
        default:
          ROS_WARN("Source stream pixel format: 0x%08x", src.getPixelFormat());
          break;
      }

      return 0;
    }

};