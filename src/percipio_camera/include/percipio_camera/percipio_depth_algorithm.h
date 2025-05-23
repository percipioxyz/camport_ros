#ifndef _PERCIPIO_DEPTH_ALGORITHM_H_
#define _PERCIPIO_DEPTH_ALGORITHM_H_

#include <vector>
#include <string.h>
#include <mutex>

#include "TYApi.h"
#include "TYImageProc.h"

namespace percipio
{
typedef struct image_data {
  uint64_t      timestamp;      ///< Timestamp in microseconds
  int           imageIndex;     ///< image index used in trigger mode
  int           status;         ///< Status of this buffer
  int           size;           ///< Buffer size
  void*         buffer;         ///< Pointer to data buffer
  int           width;          ///< Image width in pixels
  int           height;         ///< Image height in pixels
  int           pixelFormat;    ///< Pixel format, see TY_PIXEL_FORMAT_LIST

  image_data() {
    memset(this, 0, sizeof(image_data));
  }

  image_data(const TY_IMAGE_DATA& image) {
    timestamp = image.timestamp;
    imageIndex = image.imageIndex;
    status = image.status;
    size = image.size;
  
    width = image.width;
    height = image.height;
    pixelFormat =image.pixelFormat;

    if(size) {
      buffer = new char[size];
      memcpy(buffer, image.buffer, size);
    } else {
      buffer = NULL;
    }
  }

  TY_IMAGE_DATA toTyImage(const TY_COMPONENT_ID comp) {
    TY_IMAGE_DATA img;
    img.componentID = comp;
    img.timestamp = this->timestamp;
    img.imageIndex = this->imageIndex;
    img.status = this->status;
    img.size = this->size;
    img.buffer = this->buffer;
    img.width = this->width;
    img.height = this->height;
    img.pixelFormat = this->pixelFormat;
    return img;
  }

  image_data(const image_data & d) {
    timestamp = d.timestamp;
    imageIndex = d.imageIndex;
    status = d.status;
    size = d.size;
  
    width = d.width;
    height = d.height;
    pixelFormat =d.pixelFormat;

    if(size) {
      buffer = new char[size];
      memcpy(buffer, d.buffer, size);
    } else {
      buffer = NULL;
    }
  }

  bool resize(const int sz) {
    if(buffer) 
      delete []buffer;

    if(sz) {
      buffer = new char[sz];
      memset(buffer, 0, size);
    } else
      buffer = NULL;

    size = sz;
    return true;
  }

  image_data& operator=(const image_data& d) {
    this->timestamp = d.timestamp;
    this->imageIndex = d.imageIndex;
    this->status = d.status;
    this->size = d.size;
  
    this->width = d.width;
    this->height = d.height;
    this->pixelFormat =d.pixelFormat;

    this->resize(d.size);

    if(d.size)
      memcpy(this->buffer, d.buffer, d.size);
    return *this;
  }

  ~image_data() {
    if(buffer) {
      delete []buffer;
      buffer = NULL;
    }
  }

  void* Ptr() {
    return buffer;
  }

}image_data;


class DepthTimeDomainMgr
{
  public:
    DepthTimeDomainMgr(const int frame) : m_image_num(frame) {}
    ~DepthTimeDomainMgr(){};

    void add_frame(const TY_IMAGE_DATA& img);
    void reset(const int frame);

    bool do_time_domain_process(TY_IMAGE_DATA& img);

    const int param_image_num() { return m_image_num; }

  private:
    std::mutex _mutex;
    int m_image_num;
    std::vector<image_data> images;
};

}
#endif