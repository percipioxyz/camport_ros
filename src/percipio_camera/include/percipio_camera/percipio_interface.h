/*
 * @Description: 
 * @Author: zxy
 * @Date: 2023-08-04 14:20:07
 * @LastEditors: zxy
 * @LastEditTime: 2024-05-30 15:02:57
 */
#ifndef _PERCIPIO_H_
#define _PERCIPIO_H_

#include <ros/ros.h>

#include <condition_variable>
#include <mutex>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/function.hpp>

#include "TYApi.h"
#include "TYParameter.h"
#include "TYImageProc.h"
#include "stdio.h"

#include <vector>
#include "Utils.hpp"

#include "deviceinfo.h"
#include "videomode.h"

namespace percipio
{
  typedef enum
  {
    SENSOR_NONE     = 0,
    SENSOR_IR_LEFT  = 1,
    SENSOR_IR_RIGHT = 2,
    SENSOR_COLOR    = 3,
    SENSOR_DEPTH    = 4,
    SENSOR_POINT3D  = 5,
  } SensorType;

  typedef enum
  {
    IREnhanceOFF    = 0,
    IREnhanceLinearStretch       = 1,
    IREnhanceLinearStretch_Multi = 2, //2-20;  Def:8
    IREnhanceLinearStretch_STD   = 3, //2-20;  Def:6
    IREnhanceLinearStretch_LOG2  = 4, //5-50;  Def:20
    IREnhanceLinearStretch_Hist  = 5
  }IREnhanceModel;

  typedef enum
  {
    IMAGE_REGISTRATION_OFF				    = 0,
    IMAGE_REGISTRATION_DEPTH_TO_COLOR	= 1,
    IMAGE_REGISTRATION_COLOR_TO_DEPTH	= 2,
  } ImageRegistrationMode;

  typedef enum {
    DISTORTION_CORRECTION,
    EPIPOLAR_RECTIFICATION
  } IRImageRectificationMode;

  typedef enum
  {
    DEVICE_EVENT_OFFLINE = 0,
    DEVICE_EVENT_AUTO_RECONNECTED,
  } DeviceEvent;
  
  typedef enum
  {
    DEVICE_STATE_OK 	= 0,
    DEVICE_STATE_ERROR 	= 1,
    DEVICE_STATE_NOT_READY 	= 2,
    DEVICE_STATE_EOF 	= 3
  } DeviceState;

  typedef enum
  {
    GigeE_2_0,
    GigeE_2_1,
  } GigEVersion;
  
  typedef void (*DeviceInfoCallback)(const DeviceInfo* pInfo, void* pCookie);
  typedef void (*DeviceStateCallback)(const DeviceInfo* pInfo, DeviceState deviceState, void* pCookie);

  typedef boost::function<void(void)> DeviceInitCallbackFunction;
  typedef boost::function<void(const char*)> DeviceEventCallbackFunction;
  typedef boost::function<TY_STATUS(void)> StreamRestoreCallbackFunction;
  typedef struct
  {
    DeviceInfoCallback    deviceConnected;
    DeviceInfoCallback    deviceDisconnected;
    DeviceStateCallback	  deviceStateChanged;
  } DeviceCallbacks;

  typedef struct {
    DeviceCallbacks* cb;
    void* pListener;
  } DeviceCallback_t;

  typedef struct {
    std::string node_desc;
    std::string node_info;
  } DeviceNodeInfo;
  typedef std::map<std::string, std::vector<DeviceNodeInfo>> percipio_feat;

  typedef struct {
      float binning;
      int32_t full_width;
      int32_t full_height;
  } percipio_stream_info;

  class VideoFrameData;
  class VideoStream;
  class Percipio;
  class DeviceDisconnectedListener;
  class DepthTimeDomainMgr;

  class GigEBase
  {
  public:
    GigEBase(const TY_DEV_HANDLE dev) : hDevice(dev){};
    virtual ~GigEBase() {};

    virtual std::vector<VideoMode>    getLeftIRVideoModeList() = 0;
    virtual std::vector<VideoMode>    getRightIRVideoModeList() = 0;
    virtual std::vector<VideoMode>    getColorVideoModeList() = 0;
    virtual std::vector<VideoMode>    getDepthVideoModeList() = 0;

    virtual TY_STATUS getColorCalibData(TY_CAMERA_CALIB_INFO& calib_data) = 0;
    virtual TY_STATUS getDepthCalibData(TY_CAMERA_CALIB_INFO& calib_data) = 0;

    virtual TY_STATUS getColorIntrinsic(TY_CAMERA_INTRINSIC& Intrinsic) = 0;
    virtual TY_STATUS getDepthIntrinsic(TY_CAMERA_INTRINSIC& Intrinsic) = 0;

    virtual TY_STATUS getIRLensType(TYLensOpticalType& type) = 0;
    virtual TY_STATUS getIRRectificationMode(IRImageRectificationMode& mode) = 0;

    virtual TY_STATUS getLeftIRCalibData(TY_CAMERA_CALIB_INFO& calib_data) = 0;
    virtual TY_STATUS getLeftIRRotation(TY_CAMERA_ROTATION& rotation) = 0;
    virtual TY_STATUS getLeftIRRectifiedIntr(TY_CAMERA_INTRINSIC& rectified_intr) = 0;

    virtual TY_STATUS getRightIRCalibData(TY_CAMERA_CALIB_INFO& calib_data) = 0;
    virtual TY_STATUS getRightIRRotation(TY_CAMERA_ROTATION& rotation) = 0;
    virtual TY_STATUS getRightIRRectifiedIntr(TY_CAMERA_INTRINSIC& rectified_intr) = 0;

    virtual TY_STATUS getDepthScaleUnit(float& f_depth_scale) = 0;

    virtual TY_STATUS EnableHwIRUndistortion() = 0;

    virtual TY_STATUS AcquisitionInit() = 0;

    virtual TY_STATUS SetImageMode(const SensorType sensorType, const int width, const int height, const std::string& fmt) = 0;

    virtual TY_STATUS PreSetting() = 0;

    virtual TY_STATUS LoadParametersFromXML(const percipio_feat& cfg) = 0;

    virtual bool is_support_frame_rate_ctrl() = 0;
    virtual TY_STATUS  frame_rate_init(const float fps) = 0;
    virtual TY_STATUS  enable_trigger_mode(const bool en) = 0;

    virtual TY_STATUS  send_soft_trigger_signal() = 0;

    virtual TY_STATUS EnableColorStream(const bool en) = 0;
    virtual TY_STATUS EnableDepthStream(const bool en) = 0;
    virtual TY_STATUS EnableLeftIRStream(const bool en) = 0;
    virtual TY_STATUS EnableRightIRStream(const bool en) = 0;

    virtual TY_STATUS stream_base_info_init() = 0;

    virtual void Reset() = 0;
    
    TY_DEV_HANDLE hDevice;

    bool need_depth_undistortion = false;

    std::map<SensorType, std::vector<VideoMode>> videos;
    std::map<SensorType, VideoMode> current_video_mode;

    float get_stream_binning(TY_COMPONENT_ID comp) {
        auto it = m_stream_base_info.find(comp);
        return (it != m_stream_base_info.end()) ? it->second.binning : 1.f;
    }

  protected:
    std::map<TY_COMPONENT_ID, percipio_stream_info> m_stream_base_info;
  private:
    
  };

  class EnumeratedDevice
  {
    public:
      EnumeratedDevice(const TY_INTERFACE_HANDLE iface, const DeviceInfo& info)
        : hIface(iface), device_info_(info) {}

      ~EnumeratedDevice(){}

      const TY_INTERFACE_HANDLE get_handle() const { return hIface; }
      const DeviceInfo& get_device_info() const { return device_info_; }
      
    private:
      TY_INTERFACE_HANDLE hIface;
      DeviceInfo device_info_;
  };
  class PercipioDepthCam
  {
    public:
      PercipioDepthCam();
      ~PercipioDepthCam()
      {
        TYDeinitLib();
      }

      friend class VideoStream;

      TY_STATUS initialize();
      void GetDeviceList(std::vector<EnumeratedDevice>& devices);

      TY_STATUS openWithSN(const char* sn);
      TY_STATUS openWithIP(const char* ip);

      int send_soft_trigger();

      void reset();

      void dynamic_configure(const std::string& str);

      int parse_xml_parameters(const std::string& xml);

      boost::shared_ptr<DeviceInitCallbackFunction> ptrFuncDeviceInit;
      void setDeviceInitCallback(boost::shared_ptr<DeviceInitCallbackFunction>& callback);

      boost::shared_ptr<DeviceEventCallbackFunction> ptrFuncDeviceEvent;
      void setDeviceEventCallback(boost::shared_ptr<DeviceEventCallbackFunction>& callback);

      boost::shared_ptr<StreamRestoreCallbackFunction> ptrFuncStreamRestore;
      void setStreamRestoreCallback(boost::shared_ptr<StreamRestoreCallbackFunction>& callback);

      const DeviceInfo& get_current_device_info();

      TY_STATUS set_image_mode(const SensorType type, const int width, const int height, const std::string& fmt);
      
      static void eventCallback(TY_EVENT_INFO *event_info, void *userdata);

      TY_STATUS RegisterDeviceCallbacks(DeviceCallbacks* callback, void* listener);

      void UnregisterDeviceCallbacks(void* listener);

      bool isValid();

      TY_STATUS get_device_info(TY_DEVICE_BASE_INFO& info);

      void close();

      const uint32_t& get_components() const ;

      bool DeviceSetColorUndistortionEnable(bool enable);

      bool DeviceSetIRUndistortionEnable(bool enable);

      bool DepthStreamSetSpeckFilterEn(bool enabled);
      bool DepthStreamGetSpeckFilterEn();

      bool DepthStreamSetSpeckFilterSpecSize(int spec_size);
      int DepthStreamGetSpeckFilterSpecSize();
  
      bool DepthStreamSetSpeckFilterDiff(int spec_diff);
      int DepthStreamGetSpeckFilterDiff();

      bool DepthStreamSetSpeckFilterPhySize(float phy_size);
      float DepthStreamGetSpeckFilterPhySize();

      bool DepthStreamSetTimeDomainFilterEn(bool enabled);
      bool DepthStreamGetTimeDomainFilterEn();
      bool DepthStreamSetTimeDomainFilterFCnt(int frameCnt);
      int  DepthStreamGetTimeDomainFilterFCnt();

      void IRStreamEnhancementConfig(IREnhanceModel model, int coeff);

      bool DeviceIsImageRegistrationModeSupported() const;

      //bool load_default_parameter();

      TY_STATUS DeviceSetImageRegistrationMode(ImageRegistrationMode mode);
      ImageRegistrationMode DeviceGetImageRegistrationMode();
      TY_STATUS MapDepthFrameToColorCoordinate(VideoFrameData* src, VideoFrameData* dst);
      TY_STATUS MapXYZ48FrameToColorCoordinate(VideoFrameData* src, VideoFrameData* dst);
      TY_STATUS FrameDecoder(VideoFrameData& src, VideoFrameData& dst);
      TY_STATUS DoIRUndistortion(VideoFrameData& IR);
      TY_STATUS IREnhancement(VideoFrameData& IR);

      TY_CAMERA_CALIB_INFO adjustCalibByBinningCrop(const TY_CAMERA_CALIB_INFO& src_calib, TY_COMPONENT_ID comp, const TY_IMAGE_DATA& image_data);
      TY_CAMERA_INTRINSIC adjustIntrinsicByBinningCrop(const TY_CAMERA_INTRINSIC& src_intr, TY_COMPONENT_ID comp, const TY_IMAGE_DATA& image_data);

      TY_STATUS parseIrStream(VideoFrameData* src, VideoFrameData* dst);
      TY_STATUS parseColorStream(VideoFrameData* src, VideoFrameData* dst);
      TY_STATUS parseDepthStream(VideoFrameData* src, VideoFrameData* dst);
      
      TY_STATUS DeviceGetProperty(TY_COMPONENT_ID comp, TY_FEATURE_ID feat, void* ptr, size_t size);
      TY_STATUS DeviceSetProperty(TY_COMPONENT_ID comp, TY_FEATURE_ID feat, void* ptr, size_t size);
    
      const std::vector<VideoMode>& getVideoModeList(SensorType type) const;

      TY_STATUS CreateStream(SensorType sensorType, boost::shared_ptr<VideoStream>& stream);

      VideoMode get_current_video_mode(boost::shared_ptr<VideoStream> stream);

      TY_STATUS StreamRegisterNewFrameCallback(boost::shared_ptr<VideoStream> stream, void* listener);

      void StreamUnregisterNewFrameCallback(boost::shared_ptr<VideoStream> stream);

      static void* soft_frame_rate_ctrl_thread(void* ptr);
      static void* device_frame_fetch_thread(void* ptr);

      static void* device_offline_reconnect(void* ptr);

      void send_device_event(DeviceEvent eventID);

      void EnableIRTopic(bool en) { enable_ir_topic = en; }
      void EnableColorTopic(bool en) { enable_color_topic = en; }
      void EnableDepthTopic(bool en) { enable_depth_topic = en; }
      void EnableP3DTopic(bool en) { enable_cloud_topic = en; }

      bool ir_topic_swicth() { return enable_ir_topic; }
      bool color_topic_swicth() { return enable_color_topic; }
      bool depth_topic_swicth() { return enable_depth_topic; }
      bool pointcloud_topic_swicth() { return enable_cloud_topic; }

      bool HasStream();

      void StreamEnable(boost::shared_ptr<VideoStream> stream);
      void StreamDisable(boost::shared_ptr<VideoStream> stream);

      TY_STATUS StartCapture();
      void StopCapture(boost::shared_ptr<VideoStream> stream);

      void set_work_mode(bool enable_rate, float frame_rate, bool trigger_mode_en);

      float getDepthScaleUnit() { return f_depth_scale_unit; }
      const TY_CAMERA_CALIB_INFO& getDepthCalib() { return adj_depth_calib; }
      const TY_CAMERA_INTRINSIC&  getDepthIntr() { return adj_depth_calib.intrinsic; }
      const TY_CAMERA_INTRINSIC&  getColorIntr() { return adj_color_calib.intrinsic; }
      const TY_CAMERA_DISTORTION& getColorDist() { return adj_color_calib.distortion; }

      TY_CAMERA_CALIB_INFO adj_depth_calib;
      TY_CAMERA_CALIB_INFO adj_color_calib;
      TY_CAMERA_CALIB_INFO adj_left_ir_calib;
      TY_CAMERA_CALIB_INFO adj_right_ir_calib;

      TY_CAMERA_INTRINSIC adj_left_ir_rectified_intr;
      TY_CAMERA_INTRINSIC adj_right_ir_rectified_intr;
      
      SensorType StreamGetSensorInfo(boost::shared_ptr<VideoStream> stream);

      std::string            m_current_device_sn;
      static bool            isOffline;
      bool                   b_auto_reconnect;
      bool                   isRuning;
      const TY_DEV_HANDLE getCurrentDeviceHandle() const;

      const boost::shared_ptr<GigEBase> getCurrentGigEDevice() const;

      const float get_frame_rate() const;
      
      std::mutex detect_mutex;
      std::condition_variable detect_cond;
      static bool   b_device_opened;
      pthread_t     device_status_listen;
      
    private:
      TY_INTERFACE_HANDLE _M_IFACE;
      TY_DEV_HANDLE       _M_DEVICE;
      TY_COMPONENT_ID     mIDS;

      IREnhanceModel      mEnhanceModel = IREnhanceOFF;
      int32_t             mEnhanceCoeff = 1; //coefficient

      GigEVersion         gige_version = GigeE_2_0;

      boost::shared_ptr<GigEBase> m_gige_dev;

      bool enable_soft_frame_rate_ctrl = false;
      bool b_enable_rate_ctrl;
      float f_frame_rate;
      
      bool b_trigger_mode_en;

      std::vector<VideoMode> leftIRVideoMode;
      std::vector<VideoMode> rightIRVideoMode;
      std::vector<VideoMode> RGBVideoMode;
      std::vector<VideoMode> DepthVideoMode;

      std::vector<char>      frameBuffer[2];
      pthread_t              frame_rate_ctrl_thread;
      pthread_t              frame_fetch_thread;

      boost::shared_ptr<VideoStream> leftIRStream;
      boost::shared_ptr<VideoStream> rightIRStream;
      boost::shared_ptr<VideoStream> ColorStream;
      boost::shared_ptr<VideoStream> DepthStream;
      boost::shared_ptr<VideoStream> Point3DStream;

      bool enable_color_topic = false;
      bool enable_depth_topic = false;
      bool enable_cloud_topic = false;
      bool enable_ir_topic = false;

      boost::shared_ptr<DepthTimeDomainMgr> DepthDomainTimeFilterMgrPtr;

      //std::mutex m_mutex;
      pthread_mutex_t m_mutex = PTHREAD_MUTEX_INITIALIZER;
      
      percipio_feat cfg;
      
      int32_t current_rgb_width;
      int32_t current_rgb_height;
      int32_t actual_rgb_width;
      int32_t actual_rgb_height;

      TY_CAMERA_CALIB_INFO depth_calib;
      TY_CAMERA_CALIB_INFO color_calib;

      bool b_stream_with_color = false;
      TY_CAMERA_INTRINSIC depth_intr;
      TY_CAMERA_INTRINSIC color_intr;

      IRImageRectificationMode ir_rectificatio_mode = DISTORTION_CORRECTION;
      TYLensOpticalType    ir_len_type = TY_LENS_PINHOLE;

      TY_CAMERA_CALIB_INFO left_ir_calib;
      TY_CAMERA_ROTATION   left_ir_rotation;
      TY_CAMERA_INTRINSIC  left_ir_rectified_intr;

      TY_CAMERA_CALIB_INFO right_ir_calib;
      TY_CAMERA_ROTATION   right_ir_rotation;
      TY_CAMERA_INTRINSIC  right_ir_rectified_intr;

      bool color_undistortion = false;
      bool depth_distortion = false;

      bool ir_undistortion = false;
      bool enable_sw_ir_undistortion = false;

      bool depth_stream_spec_enable = false;
      int  depth_stream_spec_size = 150; //default
      int  depth_stream_spec_diff = 64;
      float depth_stream_physical_size = 20;

      bool depth_time_domain_enable = false;
      int  depth_time_domain_frame_cnt = 3;

      float f_depth_scale_unit = 1.0f;

      DeviceInfo                     current_device_info;
      std::vector<EnumeratedDevice>  enumerated_devices;

      ImageRegistrationMode registration_mode = IMAGE_REGISTRATION_OFF;

      TY_STATUS DeviceInit();

      TY_STATUS StreamStart();
      void StreamStop(boost::shared_ptr<VideoStream> stream);
      void StreamStopAll();

      TY_STATUS create_leftIR_stream(boost::shared_ptr<VideoStream>& stream);

      TY_STATUS create_rightIR_stream(boost::shared_ptr<VideoStream>& stream);
      
      TY_STATUS create_color_stream(boost::shared_ptr<VideoStream>& stream);

      TY_STATUS create_depth_stream(boost::shared_ptr<VideoStream>& stream);

      TY_STATUS create_point3d_stream(boost::shared_ptr<VideoStream>& stream);

      const TY_COMPONENT_ID get_stream_component_id(boost::shared_ptr<VideoStream> stream);

      const SensorType get_stream_type(boost::shared_ptr<VideoStream> stream);

  };
  
  class SensorInfo
  {
  public:
	  SensorType getSensorType() const;
    const std::vector<VideoMode>& getSupportedVideoModes() const;

  private:
    SensorInfo(const SensorInfo&);
    SensorInfo& operator=(const SensorInfo&);

    SensorInfo();

    SensorInfo(const SensorType type);

    void _setInternal(const SensorType type);

    SensorType m_type;

    std::vector<VideoMode> m_videoModes;

    friend class VideoStream;
    friend class PercipioDevice;
  };

  class VideoFrameData
  {
    public:
      VideoFrameData();
      ~VideoFrameData();

      uint64_t getTimestamp() const {return timestamp;}
      int32_t  getWidth() const {return width;}
      int32_t  getHeight() const {return height;}
      int32_t  getDataSize() const {return size;}
      uint32_t  getPixelFormat() const {return pixelFormat;}
      void*    getData() const {return buffer;}
      int32_t  getFrameIndex() const {return imageIndex;}
      int32_t  getComponentID() const {return componentID;};


      void  setTimestamp(uint64_t time);
      void  setWidth(int32_t w);
      void  setHeight(int32_t h);
      void  Resize(int32_t sz);
      void  setPixelFormat(uint32_t fmt);
      void  setFrameIndex(int32_t idx);
      void  setComponentID(int32_t compID);

      void  setData(TY_IMAGE_DATA* data);
      void  clone(const VideoFrameData& frame);

    private:
      pthread_mutex_t _mutex = PTHREAD_MUTEX_INITIALIZER;

      bool            m_isOwner;
      uint64_t        timestamp;      ///< Timestamp in microseconds
      int32_t         imageIndex;     ///< image index used in trigger mode
      int32_t         status;         ///< Status of this buffer
      int32_t         componentID;    ///< Where current data come from
      int32_t         size;           ///< Buffer size
      void*           buffer;         ///< Pointer to data buffer
      int32_t         width;          ///< Image width in pixels
      int32_t         height;         ///< Image height in pixels
      uint32_t        pixelFormat;    ///< Pixel format, see TYPixFmtList
  };

  class VideoFrameRef
  {
    public:
      inline bool isValid() const
      {
        return m_pFrame != NULL;
      }

      inline void updateFrameData(VideoFrameData* data)
      {
        m_pFrame = data;
      }

      inline uint64_t getTimestamp() const
	    {
		    return m_pFrame->getTimestamp();
	    }

      inline int getWidth() const
      {
        return m_pFrame->getWidth();
      }

      inline int getHeight() const
      {
        return m_pFrame->getHeight();
      }

      inline int getDataSize() const
      {
        return m_pFrame->getDataSize();
      }

      inline const void* getData() const
      {
        return m_pFrame->getData();
      }
      
      inline const VideoMode getVideoMode() const
      {
        return VideoMode(m_pFrame->getPixelFormat(), m_pFrame->getWidth(), m_pFrame->getHeight());
      }

      inline int getFrameIndex() const
      {
        return m_pFrame->getFrameIndex();
      }

    private:
      VideoFrameData* m_pFrame; 
  };

  class PercipioDevice;
  class VideoStream
  {
    public:
      class NewFrameListener
      {
        public:
          NewFrameListener()/* : m_callbackHandle(NULL)*/
          {
          }

          virtual ~NewFrameListener()
          {
          }

          virtual void onNewFrame(VideoStream&) = 0;

          void VideoStreamInit(VideoStream* video) { 
            std::unique_lock<std::mutex> lock(_mutex);
            ptrVideoStream = video;
          }
          void VideoStreamDeinit() {
            std::unique_lock<std::mutex> lock(_mutex);
            ptrVideoStream = nullptr;
          }

        private:
          friend class VideoStream;
          std::mutex _mutex;
          VideoStream* ptrVideoStream;
      };

      VideoStream(const std::string& name) : stream_desc(name), m_listener(NULL), m_enabled(false)
      {
      }


      ~VideoStream()
      {
        destroy();
      }

      TY_STATUS addNewFrameListener(NewFrameListener* pListener);

      void removeNewFrameListener(NewFrameListener* pListener);

      TY_STATUS start();
      void stop();

      void dispatchFrame(TY_IMAGE_DATA* frame);

      void parseImageData(TY_IMAGE_DATA* frame);

      VideoFrameData& getFrame();

      TY_STATUS readFrame(VideoFrameRef* pFrame);
      
      VideoMode getVideoMode() const;
  
      float getVerticalFieldOfView() const;

      TY_STATUS create(const PercipioDevice& device, SensorType sensorType);

      void destroy();

      bool isValid() const;

      const SensorInfo& getSensorInfo() const;

      const std::string& desc() { return stream_desc; }

      void setEnabled(bool en) { m_enabled = en; }
      bool isEnabled() const { return m_enabled; }

      NewFrameListener* getListener() const { return m_listener; }

      void setSelf(boost::shared_ptr<VideoStream> self) { m_self = self; m_created = true; }

    private:
      const std::string stream_desc;

      friend class PercipioDepthCam;

      bool m_created;
      bool m_enabled;
      NewFrameListener* m_listener;
      boost::shared_ptr<VideoStream> m_self;
      SensorInfo   m_sensorInfo;

      VideoFrameData frame;
        
    };

    static const char* ANY_DEVICE = NULL;

  class PercipioDevice;

  #define PERCIPIO_USB_PID    (0x1003)
  #define PERCIPIO_USB_VID    (0x04B4)
  
  class DeviceConnectedListener
	{
	public:
		DeviceConnectedListener()
		{
			m_deviceConnectedCallbacks.deviceConnected = deviceConnectedCallback;
			m_deviceConnectedCallbacks.deviceDisconnected = NULL;
			m_deviceConnectedCallbacks.deviceStateChanged = NULL;
		}
		
		virtual ~DeviceConnectedListener()
		{
		}
		
		virtual void onDeviceConnected(const DeviceInfo&) = 0;
	private:
		static void deviceConnectedCallback(const DeviceInfo* pInfo, void* pCookie)
		{
			DeviceConnectedListener* pListener = (DeviceConnectedListener*)pCookie;
			pListener->onDeviceConnected(static_cast<const DeviceInfo&>(*pInfo));
		}

 		friend class Percipio;
		DeviceCallbacks m_deviceConnectedCallbacks;
	};
  
  class DeviceDisconnectedListener
  {
    public:
      DeviceDisconnectedListener()
      {
        m_deviceDisconnectedCallbacks.deviceConnected = NULL;
        m_deviceDisconnectedCallbacks.deviceDisconnected = deviceDisconnectedCallback;
        m_deviceDisconnectedCallbacks.deviceStateChanged = NULL;
      }

      virtual ~DeviceDisconnectedListener()
      {
      }
		
      virtual void onDeviceDisconnected(const DeviceInfo&) = 0;
    private:
      static void deviceDisconnectedCallback(const DeviceInfo* pInfo, void* pCookie)
      {
        DeviceDisconnectedListener* pListener = (DeviceDisconnectedListener*)pCookie;
        pListener->onDeviceDisconnected(static_cast<const DeviceInfo&>(*pInfo));
      }

      friend class Percipio;
      DeviceCallbacks m_deviceDisconnectedCallbacks;
  };

  class Percipio
  {
    public:
      static TY_STATUS addDeviceConnectedListener(DeviceConnectedListener* pListener);
      static void removeDeviceConnectedListener(DeviceConnectedListener* pListener);
  
      static TY_STATUS addDeviceDisconnectedListener(DeviceDisconnectedListener* pListener);

      static void removeDeviceDisconnectedListener(DeviceDisconnectedListener* pListener);

      static TY_STATUS initialize();

      static void enumerateDevices(std::vector<EnumeratedDevice>& deviceInfoList);

      static int tycam_log_server_init(bool enable, const std::string& level, int32_t port);

      static bool forceDeviceIP(const std::string& device_URI, const std::string& ip, const std::string& netmask, const std::string& gateway);

      static const char* getExtendedError(TY_STATUS status);

      static boost::shared_ptr<PercipioDepthCam> getContext();
  };
}

#endif