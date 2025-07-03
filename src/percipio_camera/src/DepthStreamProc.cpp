#include <vector>
#include <cstring>
#include "percipio_camera/DepthStreamProc.h"

template<class T>
struct Point2_ {
    T x, y;
    Point2_() : x(0), y(0) {}
    Point2_(const T& _x, const T& _y) : x(_x), y(_y) {}
};

typedef Point2_<short>      Point2s;


template <typename T> 
T * image_ptr(T * base, int width, int y, int x = 0){
    return base + width*y + x;
}

template <typename T>
void filterSpeckles(int width, int height, T* img, int newVal, int maxSpeckleSize
        , int maxDiff, std::vector<char> &_buf)
{
    int npixels = width * height;//number of pixels
    size_t bufSize = npixels * (int)(sizeof(Point2s) + sizeof(int) + sizeof(uint8_t));//all pixel buffer
    if (_buf.size() < bufSize) {
        _buf.resize((int)bufSize);
    }

    uint8_t* buf = (uint8_t*)(&_buf[0]);
    int i, j, dstep = width;//(int)(img.step / sizeof(T));
    int* labels = (int*)buf;
    
    buf += npixels * sizeof(labels[0]);
    Point2s* wbuf = (Point2s*)buf;
    buf += npixels * sizeof(wbuf[0]);
    uint8_t* rtype = (uint8_t*)buf;
    int curlabel = 0;

    // clear out label assignments
    memset(labels, 0, npixels * sizeof(labels[0]));

    for (i = 0; i < height; i++) {
        T* ds = image_ptr(img,width,i);
        int* ls = labels + width * i;//label ptr for a row

        for (j = 0; j < width; j++) {
            if (ds[j] != newVal) { // not a bad disparity
                if (ls[j]) {   // has a label, check for bad label
                    if (rtype[ls[j]]) // small region, zero out disparity
                        ds[j] = (T)newVal;
                }
                else {
                    // no label, assign and propagate
                    Point2s* ws = wbuf; // initialize wavefront
                    Point2s p((short)j, (short)i);  // current pixel
                    curlabel++; // next label
                    
                    int count = 0;  // current region size
                    ls[j] = curlabel;

                    // wavefront propagation
                    while (ws >= wbuf) { // wavefront not empty
                        count++;

                        // put neighbors onto wavefront
                        T* dpp = image_ptr(img, width, p.y, p.x);
                        T dp = *dpp;
                        int* lpp = labels + width * p.y + p.x;

                        if (p.x < width - 1 && !lpp[+1] && dpp[+1] != newVal && std::abs(dp - dpp[+1]) <= maxDiff) {
                            lpp[+1] = curlabel;
                            *ws++ = Point2s(p.x + 1, p.y);
                        }

                        if (p.x > 0 && !lpp[-1] && dpp[-1] != newVal && std::abs(dp - dpp[-1]) <= maxDiff) {
                            lpp[-1] = curlabel;
                            *ws++ = Point2s(p.x - 1, p.y);
                        }

                        if (p.y < height - 1 && !lpp[+width] && dpp[+dstep] != newVal && std::abs(dp - dpp[+dstep]) <= maxDiff) {
                            lpp[+width] = curlabel;
                            *ws++ = Point2s(p.x, p.y + 1);
                        }

                        if (p.y > 0 && !lpp[-width] && dpp[-dstep] != newVal && std::abs(dp - dpp[-dstep]) <= maxDiff) {
                            lpp[-width] = curlabel;
                            *ws++ = Point2s(p.x, p.y - 1);
                        }

                        // pop most recent and propagate
                        // NB: could try least recent, maybe better convergence
                        p = *--ws;
                    }

                    // assign label type
                    if (count <= maxSpeckleSize) { // speckle region
                        rtype[ls[j]] = 1;   // small region label
                        ds[j] = (T)newVal;
                    } else {
                        rtype[ls[j]] = 0;   // large region label
                    }
                }
            }
        }
    }
}


static bool check_input_buff_valid(const TY_IMAGE_DATA& input, const TY_IMAGE_DATA& dst)
{
    if (!input.buffer){
        return false;
    }
    if (input.width != dst.width){
        return false;
    }
    if (input.height != dst.height){
        return false;
    }
    if (input.pixelFormat != TYPixelFormatCoord3D_C16){
        return false;
    }
    return true;
}

static float trimmed_mean(const std::vector<uint16_t> &v) 
{
    //s^2= E(x^2) -(E(x))^2
    float s1 = 0;
    int num = 0;
    if (v.size() == 0){
        return 0;
    }
    for (size_t i = 0; i < v.size(); i++){
        s1 += v[i];
        num++;
    }
    float mean = s1 / num;
    s1 = 0;
    num = 0;
    for (size_t i = 0; i < v.size(); i++){
        s1 += v[i];
        num++;
    }
    if (num == 0){
        return 0;
    }
    return s1 / num;
}

static void average_merge(const std::vector<TY_IMAGE_DATA> depth_images, TY_IMAGE_DATA& output)
{
    // NOTE: don't set output buffer to 0, the output buffer may be one of source buffers
    // memset(output->buffer, 0, output->width*output->height *sizeof(uint16_t));
    std::vector<uint16_t> buf;
    int image_num = depth_images.size();

    uint16_t *output_ptr = (uint16_t*)output.buffer;
    for (int y = 0; y < output.height; y++){
        for (int x = 0; x < output.width; x++){
            int offset = y * output.width + x;
            buf.clear();
            for (int idx = 0; idx < image_num; idx++){
                uint16_t val = ((uint16_t*)depth_images[idx].buffer)[offset];
                if (val == 0){
                    continue;
                }
                buf.push_back(val);
            }
            float mean = trimmed_mean(buf);
            output_ptr[offset] = (uint16_t)mean;
        }
    }
}

int TYDepthSpeckleFilter(TY_IMAGE_DATA& depth , const DepthSpkFilterPara& param)
{
    if(!depth.buffer){
      return TY_STATUS_NULL_POINTER;
    }

    if ((param.max_speckle_size <= 0) || (param.max_speckle_diff <= 0)) {
        return TY_STATUS_INVALID_PARAMETER;
    }

    const int new_value = 0;
    std::vector<char> label_buf;
    filterSpeckles<uint16_t>(depth.width, depth.height,(uint16_t*)depth.buffer, new_value, param.max_speckle_size, param.max_speckle_diff, label_buf);
    return TY_STATUS_OK;
}

int TYDepthEnhenceFilter(const std::vector<TY_IMAGE_DATA>& depth_images, TY_IMAGE_DATA& output)
{
#define MAX_IMAGE_NUMBER        (10)
    size_t image_num = depth_images.size();
    //parameter check
    if (!output.buffer){
        return TY_STATUS_NULL_POINTER;
    }
    if (image_num > MAX_IMAGE_NUMBER || image_num == 0){
        return TY_STATUS_INVALID_PARAMETER;
    }
    
    if ((!output.buffer) ||
        (output.width*output.height == 0) ||
        (output.pixelFormat != TYPixelFormatCoord3D_C16) ||
        (output.size < output.width*output.height*(int)sizeof(uint16_t)) ||
        (output.size == 0)){
        return TY_STATUS_OUT_OF_MEMORY;
    }

    for (int idx = 0; idx < image_num; idx++){
        if (!check_input_buff_valid(depth_images[idx], output)){
            return TY_STATUS_INVALID_PARAMETER;
        }
    }
    if(image_num == 1){
        if(depth_images[0].buffer != output.buffer){
            memcpy(output.buffer, depth_images[0].buffer, output.width*output.height*(int)sizeof(uint16_t));
        }
    } else {
        average_merge(depth_images, output);
    }

    return TY_STATUS_OK;
}

