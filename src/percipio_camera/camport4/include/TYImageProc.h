/**@file TYImageProc.h
 * @brief Image post-process API
 * @copyright  Copyright(C)2016-2018 Percipio All Rights Reserved
 **/

#ifndef TY_IMAGE_PROC_H_
#define TY_IMAGE_PROC_H_

#include "TYApi.h"

// Error code definitions
typedef enum TYDecodeError {
    TY_DECODE_SUCCESS = 0,
    TY_DECODE_NO_DECODE_NEEDED,

    TY_DECODE_ERROR_INVALID_PARAM,
    TY_DECODE_ERROR_FORMAT_MISMATCH,
    TY_DECODE_ERROR_UNSUPPORTED_FORMAT,
    TY_DECODE_ERROR_BUFFER_TOO_SMALL,
    TY_DECODE_ERROR_DATA_ERROR,

    TY_DECODE_ERROR_CREATE_WINDOW,
    TY_DECODE_ERROR_DISPLAY_IMAGE
} TYDecodeError;

// Image information structure
typedef struct TYImageInfo {
    uint32_t width;           // Image width
    uint32_t height;          // Image height
    TYPixFmt format;          // Pixel format
    uint32_t dataSize;        // Data size (bytes)
    const void* data;         // Image data pointer
} TYImageInfo;

// Output format definitions
typedef enum TYOutputFormat {
    TY_OUTPUT_FORMAT_AUTO = 0,           // Automatically select output format by priority: BGR8 > RGB8 > Mono8 > Mono16;
                                         // returns TY_DECODE_NO_DECODE_NEEDED if input format does not require decoding
    TY_OUTPUT_FORMAT_MONO8,              // Single-channel 8-bit grayscale (TYPixelFormatMono8, 1 byte/pixel);
                                         // applicable to Mono10/12/14 and PacketMono10/12/14 input formats only
    TY_OUTPUT_FORMAT_MONO16,             // Single-channel 16-bit grayscale (TYPixelFormatMono16, 2 bytes/pixel);
                                         // applicable to Mono10/12/14 and PacketMono10/12/14 input formats only
    TY_OUTPUT_FORMAT_BGR,                // 3-channel BGR 8-bit (TYPixelFormatBGR8, 3 bytes/pixel);
                                         // applicable to Bayer/YUV/JPEG input formats (demosaic or color-space conversion)
    TY_OUTPUT_FORMAT_RGB,                // 3-channel RGB 8-bit (TYPixelFormatRGB8, 3 bytes/pixel);
                                         // applicable to Bayer/YUV/JPEG input formats (demosaic or color-space conversion)
} TYOutputFormat;

// Decode result information
typedef struct TYDecodeResult {
    uint32_t width;           // Output image width
    uint32_t height;          // Output image height
    uint32_t dataSize;        // Output data size
    TYPixFmt format;          // Actual output format
} TYDecodeResult;

#define TY_DECODE_API TY_EXTC TY_EXPORT TYDecodeError TY_STDC

///@brief  Get current library version.
///@param  [out] version       Version infomation to be filled.
///@retval TY_STATUS_OK                     Succeed.
///@retval TY_STATUS_NULL_POINTER           TYGetImageAlgorithmVersion called with NULL pointer
TY_CAPI TYGetImageAlgorithmVersion(TY_VERSION_INFO* version);

/// @brief Image processing acceleration switch
/// @param  [in] en          Enable image process acceleration switch
TY_CAPI TYImageProcesAcceEnable(bool en);


/**
 * @brief Get the target pixel format for image decoding
 * 
 * This function determines the target pixel format for decoding based on the input image 
 * format and the specified output configuration. It can be used to query the pixel format
 * that will be produced by the decode operation before actually performing the decode.
 * 
 * @param input Pointer to input image information
 * @param outFmt Output parameter for the target pixel format
 * @param config Decode configuration parameters, defaults to TY_OUTPUT_FORMAT_AUTO
 * @return TYDecodeError
 *   - TY_DECODE_SUCCESS: Successfully determined target format
 *   - TY_DECODE_NO_DECODE_NEEDED: No decoding needed, output format same as input
 *   - TY_DECODE_ERROR_INVALID_PARAM: Invalid input parameters
 *   - TY_DECODE_ERROR_UNSUPPORTED_FORMAT: Unsupported output format
 *   - TY_DECODE_ERROR_FORMAT_MISMATCH: Incompatible input and output formats
 */
TY_DECODE_API TYGetDecodeTargetPixFmt(const TYImageInfo* input, TYPixFmt* outFmt,
                               const TYOutputFormat config = TY_OUTPUT_FORMAT_AUTO);

/**
 * @brief Calculate the required buffer size for image decoding
 * 
 * This function calculates the required buffer size (in bytes) for storing the decoded image
 * based on the input image format, dimensions, and the specified output configuration. 
 * The calculation takes into account the target pixel format, image dimensions, and 
 * memory alignment requirements (4-byte aligned stride).
 * 
 * @param input Pointer to input image information
 * @param totalSize Output parameter for required buffer size in bytes
 * @param config Decode configuration parameters, defaults to TY_OUTPUT_FORMAT_AUTO
 * @return TYDecodeError 
 *   - TY_DECODE_SUCCESS: Successfully calculated buffer size
 *   - TY_DECODE_ERROR_INVALID_PARAM: Invalid input parameters (null pointers, zero dimensions, etc.)
 *   - TY_DECODE_ERROR_UNSUPPORTED_FORMAT: Unsupported output format or unable to determine format
 *   - TY_DECODE_ERROR_FORMAT_MISMATCH: Incompatible input and output formats
 *   - TY_DECODE_NO_DECODE_NEEDED: No decoding needed, output format same as input
 */
TY_DECODE_API TYGetDecodeBufferSize(const TYImageInfo* input, uint32_t* totalSize,
                               const TYOutputFormat config = TY_OUTPUT_FORMAT_AUTO);

/**
 * @brief Decode an image from compressed format to specified output format
 * 
 * @param input Pointer to input image information
 * @param config Decode configuration parameters specifying output format
 * @param outputBuffer User-allocated output buffer for decoded image data
 * @param outputBufferSize Size of the output buffer in bytes
 * @param result Output parameter containing decode result information
 * @return int Error code, TY_DECODE_SUCCESS (0) indicates success
 */
TY_DECODE_API TYDecodeImage(const TYImageInfo* input, const TYOutputFormat config,
                       void* outputBuffer, uint32_t outputBufferSize, 
                       TYDecodeResult* result);


/// @brief Compute an adjusted TY_CAMERA_CALIB_INFO based on binning and crop parameters.
///
///        When an image is obtained by first applying binning to the intrinsic resolution
///        (TY_CAMERA_CALIB_INFO::intrinsicWidth / intrinsicHeight) and then cropping a
///        sub-region, the original calibration data no longer matches the actual image.
///        This function produces a new TY_CAMERA_CALIB_INFO that correctly describes the
///        binned-and-cropped image, so it can be used directly with TYUndistortImage,
///        TYUndistortImage2, TYMapDepthToPoint3d, TYMapDepthImageToPoint3d, and all other
///        coordinate mapping / undistortion functions.
///
///        Adjustment details:
///        - Intrinsic matrix:
///            fx' = fx / binningX
///            fy' = fy / binningY
///            cx' = cx / binningX - cropOffsetX
///            cy' = cy / binningY - cropOffsetY
///          Binning scales both focal length and principal point by the binning factor.
///          Cropping shifts the principal point by the crop offset (the image origin moves
///          to the top-left corner of the crop region).
///        - Intrinsic resolution:
///            intrinsicWidth'  = croppedWidth
///            intrinsicHeight' = croppedHeight
///          This ensures that the scaleX/scaleY computation inside undistortion and coordinate
///          mapping functions (imageWidth / intrinsicWidth') equals 1.0 when the source image
///          exactly covers the cropped region, so the adjusted intrinsic coordinates map
///          directly to source image pixel coordinates.
///        - Distortion and extrinsic matrices are copied unchanged (they are independent of
///          binning and crop).
///
///        How to obtain the parameters:
///        - Method 1: Read from camera GenICam nodes before capture
///            binningX     -> TYEnumGetValue read from node "BinningHorizontal"
///            binningY     -> TYEnumGetValue read from node "BinningVertical"
///            cropOffsetX  -> Select target RegionSelector, then TYIntegerGetValue read node "OffsetX"
///            cropOffsetY  -> Select target RegionSelector, then TYIntegerGetValue read node "OffsetY"
///            croppedWidth -> Select target RegionSelector, then TYIntegerGetValue read node "Width"
///            croppedHeight-> Select target RegionSelector, then TYIntegerGetValue read node "Height"
///        - Method 2: Read from TY_IMAGE_DATA after TYFetchFrame
///            binningX     -> frame.image[i].binningX
///            binningY     -> frame.image[i].binningY
///            cropOffsetX  -> frame.image[i].cropOffsetX
///            cropOffsetY  -> frame.image[i].cropOffsetY
///            croppedWidth -> frame.image[i].width
///            croppedHeight-> frame.image[i].height
///
/// @param  [in]  srcCalibInfo      Original calibration data (based on intrinsic resolution).
/// @param  [in]  binningX          Horizontal binning factor applied to the intrinsic resolution.
///                                  Must be >= 1. Typical values: 1, 2, 4.
///                                  Can be read from camera node "BinningHorizontal",
///                                  or from TY_IMAGE_DATA::binningX after TYFetchFrame.
/// @param  [in]  binningY          Vertical binning factor applied to the intrinsic resolution.
///                                  Must be >= 1. Typical values: 1, 2, 4.
///                                  Can be read from camera node "BinningVertical",
///                                  or from TY_IMAGE_DATA::binningY after TYFetchFrame.
/// @param  [in]  cropOffsetX       Horizontal offset (in pixels) of the crop region from the
///                                  top-left corner of the binned image. Must be >= 0.
///                                  Can be read from camera node "OffsetX" (after selecting
///                                  the target RegionSelector), or from TY_IMAGE_DATA::cropOffsetX
///                                  after TYFetchFrame.
/// @param  [in]  cropOffsetY       Vertical offset (in pixels) of the crop region from the
///                                  top-left corner of the binned image. Must be >= 0.
///                                  Can be read from camera node "OffsetY" (after selecting
///                                  the target RegionSelector), or from TY_IMAGE_DATA::cropOffsetY
///                                  after TYFetchFrame.
/// @param  [in]  croppedWidth      Width of the cropped image in pixels. Must be > 0.
///                                  Can be read from camera node "Width" (after selecting
///                                  the target RegionSelector), or from TY_IMAGE_DATA::width
///                                  after TYFetchFrame.
/// @param  [in]  croppedHeight     Height of the cropped image in pixels. Must be > 0.
///                                  Can be read from camera node "Height" (after selecting
///                                  the target RegionSelector), or from TY_IMAGE_DATA::height
///                                  after TYFetchFrame.
/// @param  [out] dstCalibInfo      Output adjusted calibration data. Must not be NULL.
///
/// @retval TY_STATUS_OK                Succeed.
/// @retval TY_STATUS_NULL_POINTER      srcCalibInfo or dstCalibInfo is NULL.
/// @retval TY_STATUS_INVALID_PARAMETER binningX < 1, binningY < 1, cropOffsetX < 0, cropOffsetY < 0,
///                                      croppedWidth <= 0, croppedHeight <= 0,
///                                      or the crop region exceeds the binned image bounds
///                                      (cropOffsetX + croppedWidth > intrinsicWidth / binningX, etc.).
TY_CAPI TYAdjustCalibInfoByBinningCrop (const TY_CAMERA_CALIB_INFO *srcCalibInfo
        , int32_t binningX
        , int32_t binningY
        , int32_t cropOffsetX
        , int32_t cropOffsetY
        , int32_t croppedWidth
        , int32_t croppedHeight
        , TY_CAMERA_CALIB_INFO *dstCalibInfo
        );

/// @brief Do image undistortion, only support TYPixelFormatMono8, TYPixelFormatMono16, TYPixelFormatRGB8, TYPixelFormatBGR8, TYPixelFormatCoord3D_C16.
/// @param  [in]  srcCalibInfo          Image calibration data.
/// @param  [in]  srcImage              Source image.
/// @param  [in]  cameraNewIntrinsic    Expected new image intrinsic, will use srcCalibInfo for new image intrinsic if set to NULL.
/// @param  [out] dstImage              Output image.
/// @retval TY_STATUS_OK        Succeed.
/// @retval TY_STATUS_NULL_POINTER      Any srcCalibInfo, srcImage, dstImage, srcImage->buffer, dstImage->buffer is NULL.
/// @retval TY_STATUS_INVALID_PARAMETER Invalid srcImage->width, srcImage->height, dstImage->width, dstImage->height or unsupported pixel format.
TY_CAPI TYUndistortImage (const TY_CAMERA_CALIB_INFO *srcCalibInfo
        , const TY_IMAGE_DATA *srcImage
        , const TY_CAMERA_INTRINSIC *cameraNewIntrinsic
        , TY_IMAGE_DATA *dstImage
        , const TYLensOpticalType type = TY_LENS_PINHOLE
        );

/// @brief Do image undistortion, only support TYPixelFormatMono8, TYPixelFormatMono16, TYPixelFormatRGB8, TYPixelFormatBGR8, TYPixelFormatCoord3D_C16.
/// @param  [in]  srcCalibInfo          Image calibration data.
/// @param  [in]  srcImage              Source image.
/// @param  [in]  cameraRotation        Camera rotation parameter for image orientation adjustment.
/// @param  [in]  cameraNewIntrinsic    Expected new image intrinsic, will use srcCalibInfo for new image intrinsic if set to NULL.
/// @param  [out] dstImage              Output image.
/// @retval TY_STATUS_OK        Succeed.
/// @retval TY_STATUS_NULL_POINTER      Any srcCalibInfo, srcImage, dstImage, srcImage->buffer, dstImage->buffer is NULL.
/// @retval TY_STATUS_INVALID_PARAMETER Invalid srcImage->width, srcImage->height, dstImage->width, dstImage->height or unsupported pixel format.
TY_CAPI TYUndistortImage2 (const TY_CAMERA_CALIB_INFO *calib_info,
    const TY_IMAGE_DATA *srcImage,
    const TY_CAMERA_ROTATION *cameraRotation,
    const TY_CAMERA_INTRINSIC *cameraNewIntrinsic,
    TY_IMAGE_DATA *dstImage,
    const TYLensOpticalType type = TY_LENS_PINHOLE);

// -----------------------------------------------------------
struct DepthSpeckleFilterParameters {
    int max_speckle_size; // blob size smaller than this will be removed
    int max_speckle_diff; // Maximum difference between neighbor disparity pixels
    float max_physical_size;   //Maximum Speckle Physical Size to be Filtered-Out, uint is mm^2
};

///<default parameter value definition
#define DepthSpeckleFilterParameters_Initializer {150, 64, 20}

/// @brief Remove speckles on depth image.
/// @param  [in,out]  depthImage        Depth image to be processed.
/// @param  [in]  param                 Algorithm parameters.
/// @param  [in]  calib_data            Image calibration data.
/// @retval TY_STATUS_OK        Succeed.
/// @retval TY_STATUS_NULL_POINTER      Any depth, param or depth->buffer is NULL.
/// @retval TY_STATUS_INVALID_PARAMETER param->max_speckle_size <= 0 or param->max_speckle_diff <= 0
TY_CAPI TYDepthSpeckleFilter (TY_IMAGE_DATA* depthImage
        , const DepthSpeckleFilterParameters* param
        , const TY_CAMERA_CALIB_INFO* calib_data = nullptr
        , const float depth_scale_unit = 1.f
        );

// -----------------------------------------------------------
struct DepthInpainterParameters {
    int kernel_size;    //Inpainting kernel size, determines the neighborhood range for repair. Valid range: 1 to 30.
    int max_internal_hole; //Maximum internal hole size, holes smaller than this will be filled. Valid range: 3 to 30000.
};

///<default parameter value definition
#define DepthInpainterParameters_Initializer {5, 50}

/// @brief Repair invalid pixels in depth image, filling small holes.
/// @param  [in,out]  depth             Depth image to be processed.
/// @param  [in]      param             Algorithm parameters.
/// @retval TY_STATUS_OK                Success.
/// @retval TY_STATUS_NULL_POINTER      depth, param or depth->buffer is NULL.
/// @retval TY_STATUS_INVALID_PARAMETER param->kernel_size or param->kernel_size out of range
TY_CAPI TYDepthImageInpainter(TY_IMAGE_DATA* depth
        , const DepthInpainterParameters* param
        );

// -----------------------------------------------------------
struct DepthEnhenceParameters{
    float sigma_s;          ///< Spatial standard deviation: Controls the smoothing range of the domain transform filter; larger values produce stronger smoothing
    float sigma_r;          ///< 	Range standard deviation: Controls the edge-preserving sensitivity of the domain transform filter; smaller values preserve edges more aggressively
    int   outlier_win_sz;   ///< Outlier detection window size: Controls the spatial range for speckle/outlier detection
    float outlier_rate;     ///< Minimum ratio threshold for outliers: Minimum proportion of pixels with similar depth within the window; pixels below this threshold are treated as speckle noise and removed
};

///<default parameter value definition
#define DepthEnhenceParameters_Initializer {10, 20, 10, 0.1f}

/// @brief Remove speckles on depth image.
/// @param  [in]  depthImage            Pointer to depth image array.
/// @param  [in]  imageNum              Depth image array size.
/// @param  [in]  guide                 Guide depth image. Depth image used as the edge reference for the domain transform filter; when NULL, the filtered depth output itself is used as the guide
/// @param  [out] output                Output depth image.
/// @param  [in]  param                 Algorithm parameters.
/// @retval TY_STATUS_OK        Succeed.
/// @retval TY_STATUS_NULL_POINTER      Any depthImage, param, output or output->buffer is NULL.
/// @retval TY_STATUS_INVALID_PARAMETER imageNum >= 11 or imageNum <= 0, or any image invalid
/// @retval TY_STATUS_OUT_OF_MEMORY     Output image not suitable.
TY_CAPI TYDepthEnhenceFilter (const TY_IMAGE_DATA* depthImages
        , int imageNum
        , TY_IMAGE_DATA *guide
        , TY_IMAGE_DATA *output
        , const DepthEnhenceParameters* param
        );


/// @brief Apply median filter on depth image.
/// @param  [in,out]  depthImage        Depth image to be processed, pixel format must be TYPixelFormatCoord3D_C16.
/// @param  [in]      kernel_size       Median filter kernel size, must be odd and >= 3
/// @retval TY_STATUS_OK                Succeed.
/// @retval TY_STATUS_NULL_POINTER      depthImage, param or depthImage->buffer is NULL.
/// @retval TY_STATUS_INVALID_PARAMETER kernel_size is even or less than 3, or depthImage pixel format mismatch.
TY_CAPI TYDepthMedianFilter(TY_IMAGE_DATA* depthImage
        , const int kernel_size
        );

/// @brief Check whether the image data contains a watermark.
/// @param  [in]  imageData           Pointer to TY_IMAGE_DATA to check.
/// @param  [out] hasWatermark        Output bool pointer, true if watermark exists, false otherwise.
/// @retval TY_STATUS_OK              Succeed.
/// @retval TY_STATUS_NULL_POINTER    imageData, imageData->buffer, or hasWatermark is NULL.
TY_CAPI TYHasImageWatermark(const TY_IMAGE_DATA* imageData, bool* hasWatermark);

/// @brief Check the image data watermark version if exist.
/// @param  [in]  imageData           Pointer to TY_IMAGE_DATA to check.
/// @param  [out] watermarkVer        Output uint32_t pointer, version info if watermark exists, no op otherwise.
/// @retval TY_STATUS_OK              Succeed, watermarkVer valid.
/// @retval TY_STATUS_NULL_POINTER    imageData, imageData->buffer, or watermarkVer is NULL.
/// @retval TY_STATUS_NO_DATA         No valid watermark found in the image data.

TY_CAPI TYGetImageWatermarkVer(const TY_IMAGE_DATA* imageData, uint32_t* watermarkVer);

/// @brief Extract the watermark structure from image data.
/// @param  [in]  imageData           Pointer to TY_IMAGE_DATA containing watermark.
/// @param  [out] watermark           Output watermark structure to be filled.
/// @retval TY_STATUS_OK              Watermark extracted successfully.
/// @retval TY_STATUS_NULL_POINTER    imageData, imageData->buffer, or watermark is NULL.
/// @retval TY_STATUS_NO_DATA         No valid watermark found in the image data.
/// @retval TY_STATUS_INVALID_PARAMETER len Not valid for Any version of watermark
TY_CAPI TYGetImageWatermark(const TY_IMAGE_DATA* imageData, void* watermark, uint32_t len);

#endif
