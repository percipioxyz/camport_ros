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

class ImgProc 
{
public:
    static inline void parseXYZ48(int16_t* src, int16_t* dst, int width, int height, float f_scale_unit)
    {
      for (int pix = 0; pix < width*height; pix++) {
          dst[pix] =(int16_t)(*(src + 3*pix + 2) * f_scale_unit + 0.5f);
      }
    }

    static int doDepthUndistortion(const TY_CAMERA_CALIB_INFO* depth_calib,
                                percipio::VideoFrameData* src, 
                                percipio::VideoFrameData* dst)
    {
      if(src->getPixelFormat() != TYPixelFormatCoord3D_C16) {
        ROS_ERROR("%s <%d> Invalid pixel format!\n", __FILE__, __LINE__);
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
        ROS_ERROR("%s <%d> Invalid pixel format!\n", __FILE__, __LINE__);
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
        ROS_ERROR("%s <%d> Invalid pixel format!\n", __FILE__, __LINE__);
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

      return TYMapDepthImageToColorCoordinate(
                  depth_calib,
                  src->getWidth(), src->getHeight(), (const uint16_t*)src->getData(),
                  color_calib,
                  dst->getWidth(), dst->getHeight(), (uint16_t* )dst->getData(),
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
      if(src->getPixelFormat() != TYPixelFormatCoord3D_ABC16) {
        ROS_ERROR("%s <%d> Invalid pixel format!\n", __FILE__, __LINE__);
        return-1;
      }

      uint32_t points_cnt = src->getWidth() * src->getHeight();
      int16_t* xyz48 = static_cast<int16_t*>(src->getData());

      std::vector<TY_VECT_3F> p3d(points_cnt);
      for(int i = 0; i < points_cnt; i++) {
        if(xyz48[3 * i + 2]) {
          p3d[i].x = static_cast<float>(xyz48[3 * i + 0]);
          p3d[i].y = static_cast<float>(xyz48[3 * i + 1]);
          p3d[i].z = static_cast<float>(xyz48[3 * i + 2]);
        } else {
          p3d[i].x = std::numeric_limits<float>::quiet_NaN();
          p3d[i].y = std::numeric_limits<float>::quiet_NaN();
          p3d[i].z = std::numeric_limits<float>::quiet_NaN();
        }        
      }

      TY_CAMERA_EXTRINSIC extri_inv;
      TYInvertExtrinsic(&color_calib->extrinsic, &extri_inv);
      TYMapPoint3dToPoint3d(&extri_inv, &p3d[0], points_cnt, &p3d[0]);
      
      dst->setTimestamp(src->getTimestamp());
      dst->setFrameIndex(src->getFrameIndex());
      dst->setPixelFormat(TYPixelFormatCoord3D_C16);
      if((target_width != 0) && (target_height != 0)) {
        dst->setWidth(target_width);
        dst->setHeight(target_height);
        dst->Resize(2 * target_width * target_height);
      } else {
        dst->setWidth(src->getWidth());
        dst->setHeight(src->getHeight());
        dst->Resize(2 * src->getWidth() * src->getHeight());
      }
      
      return TYMapPoint3dToDepthImage(color_calib, &p3d[0], points_cnt, dst->getWidth(), dst->getHeight(), (uint16_t*)dst->getData());
    }

    static int cvtColor(percipio::VideoFrameData& src, percipio::VideoFrameData& dst)
    {
      int i, j;
      TYImageInfo src_img;
      src_img.width  = src.getWidth();
      src_img.height = src.getHeight();
      src_img.format = src.getPixelFormat();
      src_img.dataSize = src.getDataSize();
      src_img.data = src.getData();

      uint32_t totalSize = 0;
      TYDecodeError ret = TYGetDecodeBufferSize(&src_img, &totalSize);
      if(ret != TY_DECODE_SUCCESS) {
        return ret;
      }
      
      dst.setTimestamp(src.getTimestamp());
      dst.setWidth(src.getWidth());
      dst.setHeight(src.getHeight());
      dst.setFrameIndex(src.getFrameIndex());
      dst.setComponentID(src.getComponentID());
      dst.setPixelFormat(TYPixelFormatRGB8);
      dst.Resize(totalSize);

      TYDecodeResult retInfo;
      return TYDecodeImage(&src_img,  TY_OUTPUT_FORMAT_AUTO, (void*)dst.getData(), totalSize, &retInfo);
    }

    static int IRUndistortion(percipio::VideoFrameData& IR, const TY_CAMERA_CALIB_INFO *calib_info, const TY_CAMERA_ROTATION *cameraRotation, const TY_CAMERA_INTRINSIC *cameraNewIntrinsic, const TYLensOpticalType type = TY_LENS_PINHOLE)
    {
        if (!calib_info ) {
            ROS_WARN("IRUndistortion Invalid parameters: calib_info is null");
            return TY_STATUS_INVALID_PARAMETER;
        }
        
        //Check if IR data is valid
        if (!IR.getData() || IR.getWidth() <= 0 || IR.getHeight() <= 0 || IR.getDataSize() <= 0) {
            ROS_WARN("IRUndistortion Invalid IR image data");
            return TY_STATUS_INVALID_PARAMETER;
        }
        
        //Get current image properties
        int32_t width = IR.getWidth();
        int32_t height = IR.getHeight();
        int32_t dataSize = IR.getDataSize();
        uint32_t pixelFormat = IR.getPixelFormat();
        
        //Allocate buffer for rectified image (same size as original)
        std::vector<uint8_t> rectifiedBuffer(dataSize);
        
        //Prepare source image data structure
        TY_IMAGE_DATA srcImage;
        srcImage.width = width;
        srcImage.height = height;
        srcImage.size = dataSize;
        srcImage.pixelFormat = pixelFormat;
        srcImage.buffer = IR.getData();
        
        //Prepare destination image data structure
        TY_IMAGE_DATA dstImage;
        dstImage.width = width;
        dstImage.height = height;
        dstImage.size = dataSize;
        dstImage.pixelFormat = pixelFormat;
        dstImage.buffer = rectifiedBuffer.data();
        
        //Apply distortion correction
        TY_STATUS status = TYUndistortImage2(calib_info, &srcImage, cameraRotation, 
                                            cameraNewIntrinsic, &dstImage, type);
        if (status != TY_STATUS_OK) {
            ROS_WARN("TYUndistortImage2 failed with status: %d", status);
            return status;
        }
        
        // Copy rectified data to VideoFrameData buffer
        memcpy(IR.getData(), rectifiedBuffer.data(), dataSize);
        return TY_STATUS_OK;
    }

    static int GrayIR_linearStretch(percipio::VideoFrameData& grayIR)
    {
        uint32_t pixelFormat = grayIR.getPixelFormat();
        if (!(pixelFormat == TYPixelFormatMono8 || pixelFormat == TYPixelFormatMono16)) {
            ROS_WARN("linearStretch support Mono8 or Mono16 gray, not support others type, please check grayIR pixel format");
            return -1;
        }

        int rows = grayIR.getHeight();
        int cols = grayIR.getWidth();
        
        double ratiocut = 0.1;
        int roi_x = static_cast<int>(cols * ratiocut);
        int roi_y = static_cast<int>(rows * ratiocut);
        int roi_width = static_cast<int>(cols - cols * ratiocut * 2);
        int roi_height = static_cast<int>(rows - rows * ratiocut * 2);

        if (roi_width <= 0 || roi_height <= 0 || roi_x < 0 || roi_y < 0) {
            ROS_WARN("ROI is invalid");
            return -1;
        }

        void* data = grayIR.getData();
        if (!data) {
            ROS_WARN("grayIR data is null");
            return -1;
        }

        double minVal = 0.0, maxVal = 0.0;
        
        if (pixelFormat == TYPixelFormatMono8) {
            uint8_t* buffer = static_cast<uint8_t*>(data);
            minVal = 255.0;
            maxVal = 0.0;
            
            for (int y = roi_y; y < roi_y + roi_height; ++y) {
                for (int x = roi_x; x < roi_x + roi_width; ++x) {
                    uint8_t val = buffer[y * cols + x];
                    if (val < minVal) minVal = val;
                    if (val > maxVal) maxVal = val;
                }
            }
        } else if (pixelFormat == TYPixelFormatMono16) {
            uint16_t* buffer = static_cast<uint16_t*>(data);
            minVal = 65535.0;
            maxVal = 0.0;
            
            for (int y = roi_y; y < roi_y + roi_height; ++y) {
                for (int x = roi_x; x < roi_x + roi_width; ++x) {
                    uint16_t val = buffer[y * cols + x];
                    if (val < minVal) minVal = val;
                    if (val > maxVal) maxVal = val;
                }
            }
        }

        if (maxVal - minVal <= 0.0) {
            //ROS_INFO("maxVal equals minVal, no need to stretch");
            return 0;
        }

        static std::vector<uint8_t> result_buffer;
        if(result_buffer.size() != rows * cols){
            result_buffer.resize(rows * cols);
        }

        double scale = 255.0 / (maxVal - minVal);
        double offset = -minVal * scale;
        
        if (pixelFormat == TYPixelFormatMono8) {
            uint8_t* buffer = static_cast<uint8_t*>(data);
            for (int i = 0; i < rows * cols; ++i) {
                double stretched = buffer[i] * scale + offset;

                if (stretched < 0.0) stretched = 0.0;
                if (stretched > 255.0) stretched = 255.0;
                result_buffer[i] = static_cast<uint8_t>(stretched);
            }
        } else if (pixelFormat == TYPixelFormatMono16) {
            uint16_t* buffer = static_cast<uint16_t*>(data);
            for (int i = 0; i < rows * cols; ++i) {
                double stretched = buffer[i] * scale + offset;
                if (stretched < 0.0) stretched = 0.0;
                if (stretched > 255.0) stretched = 255.0;
                result_buffer[i] = static_cast<uint8_t>(stretched);
            }
        }

        grayIR.Resize(rows * cols * sizeof(uint8_t));
        memcpy(grayIR.getData(), result_buffer.data(), rows * cols * sizeof(uint8_t));
        grayIR.setPixelFormat(TYPixelFormatMono8);
        grayIR.setWidth(cols);
        grayIR.setHeight(rows);
        
        return 0;
    }

    static int GrayIR_linearStretch_multi(percipio::VideoFrameData& grayIR, double multi_expandratio = 8)
    {
        if (multi_expandratio <= 0) {
            ROS_WARN("linearStretch_multi multi_expandratio must bigger than 0");
            return -1;
        }
        
        uint32_t pixelFormat = grayIR.getPixelFormat();
        if (!(pixelFormat == TYPixelFormatMono8 || pixelFormat == TYPixelFormatMono16)) {
            ROS_WARN("linearStretch_multi support Mono8 or Mono16 gray, not support others type, please check grayIR pixel format");
            return -1;
        }
        
        int rows = grayIR.getHeight();
        int cols = grayIR.getWidth();
        int totalPixels = rows * cols;
        
        void* data = grayIR.getData();
        if (!data) {
            ROS_WARN("grayIR data is null");
            return -1;
        }
        
        static std::vector<uint8_t> result_buffer;
        if(result_buffer.size() != totalPixels) {
            result_buffer.resize(totalPixels);
        }
        
        double eight_bit_factor = multi_expandratio;
        double sixteen_bit_factor = multi_expandratio / 255.0;
        
        if (pixelFormat == TYPixelFormatMono8) {
            uint8_t* buffer = static_cast<uint8_t*>(data);
            
            if (multi_expandratio == 1.0) {
                memcpy(&result_buffer[0], buffer, totalPixels);
            } else {
                for (int i = 0; i < totalPixels; ++i) {
                    double stretched = static_cast<double>(buffer[i]) * eight_bit_factor;
                    
                    int32_t rounded = static_cast<int32_t>(stretched + 0.5);
                    if (rounded < 0) rounded = 0;
                    if (rounded > 255) rounded = 255;
                    
                    result_buffer[i] = static_cast<uint8_t>(rounded);
                }
            }
        } 
        else if (pixelFormat == TYPixelFormatMono16) {
            uint16_t* buffer = static_cast<uint16_t*>(data);
            if (multi_expandratio == 1.0) {
                for (int i = 0; i < totalPixels; ++i) {
                    result_buffer[i] = static_cast<uint8_t>(buffer[i] >> 8); //等价于 buffer[i] / 256
                }
            } else {
                for (int i = 0; i < totalPixels; ++i) {
                    double stretched = static_cast<double>(buffer[i]) * sixteen_bit_factor;
                    
                    int32_t rounded = static_cast<int32_t>(stretched + 0.5);
                    if (rounded < 0) rounded = 0;
                    if (rounded > 255) rounded = 255;
                    
                    result_buffer[i] = static_cast<uint8_t>(rounded);
                }
            }
        }
        
        grayIR.Resize(totalPixels * sizeof(uint8_t));
        memcpy(grayIR.getData(), result_buffer.data(), totalPixels * sizeof(uint8_t));
        grayIR.setPixelFormat(TYPixelFormatMono8);
        grayIR.setWidth(cols);
        grayIR.setHeight(rows);
        
        return 0;
    }

    static int GrayIR_linearStretch_std(percipio::VideoFrameData& grayIR, double std_expandratio = 6)
    {
        if (std_expandratio <= 0) {
            ROS_WARN("GrayIR_linearStretch_std multi_expandratio must be bigger than 0");
            return -1;
        }

        //Check pixel format
        uint32_t pixelFormat = grayIR.getPixelFormat();
        if (!(pixelFormat == TYPixelFormatMono8 || pixelFormat == TYPixelFormatMono16)) {
            ROS_WARN("GrayIR_linearStretch_std supports Mono8 or Mono16 gray, not other types");
            return -1;
        }

        int rows = grayIR.getHeight();
        int cols = grayIR.getWidth();
        int totalPixels = rows * cols;
        
        void* data = grayIR.getData();
        if (!data) {
            ROS_WARN("grayIR data is null");
            return -1;
        }

        //Calculate mean and standard deviation
        double sum = 0.0;
        double sumSquares = 0.0;
        
        if (pixelFormat == TYPixelFormatMono8) {
            uint8_t* buffer = static_cast<uint8_t*>(data);
            for (int i = 0; i < totalPixels; ++i) {
                double val = static_cast<double>(buffer[i]);
                sum += val;
                sumSquares += val * val;
            }
        } else { //TYPixelFormatMono16
            uint16_t* buffer = static_cast<uint16_t*>(data);
            for (int i = 0; i < totalPixels; ++i) {
                double val = static_cast<double>(buffer[i]);
                sum += val;
                sumSquares += val * val;
            }
        }
        
        double mean = sum / totalPixels;
        double variance = (sumSquares / totalPixels) - (mean * mean);
        double stddev = (variance > 0) ? sqrt(variance) : 0.0;
        
        //Calculate normalization factor
        double use_norm = stddev * std_expandratio + 1.0;
        if (use_norm <= 0) {
            ROS_WARN("Calculated normalization factor is non-positive");
            return -1;
        }
        
        double scale = 255.0 / use_norm;
        
        //Find min and max for verification
        double minVal, maxVal;
        if (pixelFormat == TYPixelFormatMono8) {
            uint8_t* buffer = static_cast<uint8_t*>(data);
            minVal = 255.0;
            maxVal = 0.0;
            for (int i = 0; i < totalPixels; ++i) {
                double val = buffer[i];
                if (val < minVal) minVal = val;
                if (val > maxVal) maxVal = val;
            }
        } else { //TYPixelFormatMono16
            uint16_t* buffer = static_cast<uint16_t*>(data);
            minVal = 65535.0;
            maxVal = 0.0;
            for (int i = 0; i < totalPixels; ++i) {
                double val = buffer[i];
                if (val < minVal) minVal = val;
                if (val > maxVal) maxVal = val;
            }
        }
        
        //Apply linear stretch and convert to 8-bit
        static std::vector<uint8_t> result_buffer;
        if(result_buffer.size() != totalPixels) {
            result_buffer.resize(totalPixels);
        }

        if (pixelFormat == TYPixelFormatMono8) {
            uint8_t* buffer = static_cast<uint8_t*>(data);
            for (int i = 0; i < totalPixels; ++i) {
                double stretched = static_cast<double>(buffer[i]) * scale;
                if (stretched < 0.0) stretched = 0.0;
                if (stretched > 255.0) stretched = 255.0;
                result_buffer[i] = static_cast<uint8_t>(stretched);
            }
        } else { //TYPixelFormatMono16
            uint16_t* buffer = static_cast<uint16_t*>(data);
            for (int i = 0; i < totalPixels; ++i) {
                double stretched = static_cast<double>(buffer[i]) * scale;
                if (stretched < 0.0) stretched = 0.0;
                if (stretched > 255.0) stretched = 255.0;
                result_buffer[i] = static_cast<uint8_t>(stretched);
            }
        }
        
        //Update VideoFrameData
        grayIR.Resize(totalPixels * sizeof(uint8_t));
        memcpy(grayIR.getData(), result_buffer.data(), totalPixels * sizeof(uint8_t));
        grayIR.setPixelFormat(TYPixelFormatMono8);
        grayIR.setWidth(cols);
        grayIR.setHeight(rows);
        
        return 0;
    }

    static int GrayIR_nonlinearStretch_log2(percipio::VideoFrameData& grayIR, double log_expandratio = 20)
    {
        if (log_expandratio <= 0) {
            ROS_WARN("GrayIR_nonlinearStretch_log multi_expandratio must be bigger than 0");
            return -1;
        }
        
        //Check pixel format
        uint32_t pixelFormat = grayIR.getPixelFormat();
        if (!(pixelFormat == TYPixelFormatMono8 || pixelFormat == TYPixelFormatMono16)) {
            ROS_WARN("GrayIR_linearStretch_std supports Mono8 or Mono16 gray, not other types");
            return -1;
        }
        
        int rows = grayIR.getHeight();
        int cols = grayIR.getWidth();
        int totalPixels = rows * cols;
        
        void* data = grayIR.getData();
        if (!data) {
            ROS_WARN("grayIR data is null");
            return -1;
        }
        
        //Create lookup table for 16-bit values (0-65535)
        std::vector<uint8_t> lut_16bit(65536);
        for (int i = 0; i < 65536; ++i) {
            double logValue = log_expandratio * log2(static_cast<double>(i) + 1.0);
            int intValue = static_cast<int>(logValue + 0.5); //Round to nearest integer
            if (intValue > 255) intValue = 255;
            lut_16bit[i] = static_cast<uint8_t>(intValue);
        }
        
        //Create lookup table for 8-bit values
        uint8_t lut_8bit[256];
        for (int i = 0; i < 256; ++i) {
            int scaledValue = i * 256;
            double logValue = log_expandratio * log2(static_cast<double>(scaledValue) + 1.0);
            int intValue = static_cast<int>(logValue + 0.5);
            if (intValue > 255) intValue = 255;
            lut_8bit[i] = static_cast<uint8_t>(intValue);
        }
        
        //Create result buffer
        static std::vector<uint8_t> result_buffer;
        if(result_buffer.size() != totalPixels) {
            result_buffer.resize(totalPixels);
        }
        
        //Apply stretching using lookup tables
        if (pixelFormat == TYPixelFormatMono16) {
            uint16_t* buffer = static_cast<uint16_t*>(data);
            for (int i = 0; i < totalPixels; ++i) {
                result_buffer[i] = lut_16bit[buffer[i]];
            }
        } 
        else { //TYPixelFormatMono8
            uint8_t* buffer = static_cast<uint8_t*>(data);
            for (int i = 0; i < totalPixels; ++i) {
                result_buffer[i] = lut_8bit[buffer[i]];
            }
        }
        
        //Update VideoFrameData
        grayIR.Resize(totalPixels * sizeof(uint8_t));
        memcpy(grayIR.getData(), result_buffer.data(), totalPixels * sizeof(uint8_t));
        grayIR.setPixelFormat(TYPixelFormatMono8);
        grayIR.setWidth(cols);
        grayIR.setHeight(rows);
        
        return 0;
    }

    static int GrayIR_nonlinearStretch_hist(percipio::VideoFrameData& grayIR)
    {
        //Check pixel format
        uint32_t pixelFormat = grayIR.getPixelFormat();
        if (!(pixelFormat == TYPixelFormatMono8 || pixelFormat == TYPixelFormatMono16)) {
            ROS_WARN("GrayIR_linearStretch_std supports Mono8 or Mono16 gray, not other types");
            return -1;
        }
        
        int rows = grayIR.getHeight();
        int cols = grayIR.getWidth();
        int totalPixels = rows * cols;
        
        void* data = grayIR.getData();
        if (!data) {
            ROS_WARN("grayIR data is null");
            return -1;
        }
        
        //For 16-bit images, perform linear stretching first
        if (pixelFormat == TYPixelFormatMono16) {
            //Use previously implemented linear stretch function
            int result = GrayIR_linearStretch(grayIR);
            if (result != 0) {
                ROS_WARN("Linear stretch failed for 16-bit image");
                return result;
            }
            //Update data pointer after stretching
            data = grayIR.getData();
        }
        
        //Now the image should be 8-bit
        if (grayIR.getPixelFormat() != TYPixelFormatMono8) {
            ROS_WARN("Expected 8-bit image after processing");
            return -1;
        }
        
        //Calculate histogram
        uint8_t* buffer = static_cast<uint8_t*>(data);
        int histogram[256] = {0};
        
        for (int i = 0; i < totalPixels; ++i) {
            histogram[buffer[i]]++;
        }
        
        //Calculate cumulative distribution function (CDF)
        int cdf[256] = {0};
        int minCdf = totalPixels; //Initialize with maximum possible value
        
        cdf[0] = histogram[0];
        for (int i = 1; i < 256; ++i) {
            cdf[i] = cdf[i-1] + histogram[i];
        }
        
        //Find minimum non-zero CDF value
        for (int i = 0; i < 256; ++i) {
            if (cdf[i] > 0 && cdf[i] < minCdf) {
                minCdf = cdf[i];
            }
        }
        
        //Create 8-bit result buffer
        static std::vector<uint8_t> result_buffer;
        if(result_buffer.size() != totalPixels) {
            result_buffer.resize(totalPixels);
        }
        
        //Apply histogram equalization
        for (int i = 0; i < totalPixels; ++i) {
            uint8_t pixelValue = buffer[i];
            //Apply histogram equalization formula:
            //h(v) = round((cdf(v) - minCdf) / (totalPixels - minCdf) * 255)
            int equalizedValue = static_cast<int>(
                (static_cast<double>(cdf[pixelValue] - minCdf) / 
                (totalPixels - minCdf)) * 255.0 + 0.5);
            
            //Clamp to 0-255 range
            if (equalizedValue < 0) equalizedValue = 0;
            if (equalizedValue > 255) equalizedValue = 255;
            
            result_buffer[i] = static_cast<uint8_t>(equalizedValue);
        }
        
        //Update VideoFrameData
        grayIR.Resize(totalPixels * sizeof(uint8_t));
        memcpy(grayIR.getData(), result_buffer.data(), totalPixels * sizeof(uint8_t));
        grayIR.setPixelFormat(TYPixelFormatMono8);
        grayIR.setWidth(cols);
        grayIR.setHeight(rows);
        
        return 0;
    }
};