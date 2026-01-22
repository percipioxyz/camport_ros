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
  
  typedef void* StreamHandle;
  

  typedef void (* NewFrameCallback)(StreamHandle stream, void* pCookie, TY_IMAGE_DATA* frame);
  class NewFrameCallbackManager
  {
    public:
      NewFrameCallbackManager() : frame_listener(NULL), cb(NULL), is_enable(false)
      {

      }

      ~NewFrameCallbackManager()
      {

      }

      void register_callback(void* listener, NewFrameCallback func);
      void unregister_callback();
      bool isValid();
      void enableCallback(bool en);

      void* frame_listener;
      bool is_enable;
      NewFrameCallback cb;
  };
    
  typedef void* CallbackHandle;
  typedef void (*DeviceInfoCallback)(const DeviceInfo* pInfo, void* pCookie);
  typedef void (*DeviceStateCallback)(const DeviceInfo* pInfo, DeviceState deviceState, void* pCookie);

  typedef boost::function<void(void)> DeviceInitCallbackFunction;
  typedef boost::function<void(const char*)> DeviceEventCallbackFunction;
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

  class VideoFrameData;
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

    virtual void Reset() = 0;
    
    TY_DEV_HANDLE hDevice;

    bool need_depth_undistortion = false;

    std::map<SensorType, std::vector<VideoMode>> videos;
    std::map<SensorType, VideoMode> current_video_mode;
  private:
    
  };

  class PercipioDepthCam
  {
    public:
      PercipioDepthCam();
      ~PercipioDepthCam()
      {
        TYDeinitLib();
      }

      TY_STATUS initialize();
      void GetDeviceList(DeviceInfo** device_info_ptr, int* cnt);

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

      TY_STATUS parseIrStream(VideoFrameData* src, VideoFrameData* dst);
      TY_STATUS parseColorStream(VideoFrameData* src, VideoFrameData* dst);
      TY_STATUS parseDepthStream(VideoFrameData* src, VideoFrameData* dst);
      
      TY_STATUS DeviceGetProperty(TY_COMPONENT_ID comp, TY_FEATURE_ID feat, void* ptr, size_t size);
      TY_STATUS DeviceSetProperty(TY_COMPONENT_ID comp, TY_FEATURE_ID feat, void* ptr, size_t size);
    
      const std::vector<VideoMode>& getVideoModeList(SensorType type) const;

      TY_STATUS CreateStream(SensorType sensorType, StreamHandle& streamHandle);

      VideoMode get_current_video_mode(StreamHandle stream);

      TY_STATUS StreamRegisterNewFrameCallback(StreamHandle stream, void* listener, NewFrameCallback cb);

      void StreamUnregisterNewFrameCallback(StreamHandle stream);

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

      void StreamEnable(StreamHandle stream);
      void StreamDisable(StreamHandle stream);

      TY_STATUS StartCapture();
      void StopCapture(StreamHandle stream);

      void set_work_mode(bool enable_rate, float frame_rate, bool trigger_mode_en);

      float getDepthScaleUnit() { return f_depth_scale_unit; }
      const TY_CAMERA_CALIB_INFO& getDepthCalib() { return depth_calib; }
      const TY_CAMERA_INTRINSIC&  getDepthIntr() { return depth_intr; }
      const TY_CAMERA_INTRINSIC&  getColorIntr() { return color_intr; }
      const TY_CAMERA_DISTORTION& getColorDist() { return color_calib.distortion; }
      
      SensorType StreamGetSensorInfo(StreamHandle stream);

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

      boost::shared_ptr<NewFrameCallbackManager> leftIRStream;
      boost::shared_ptr<NewFrameCallbackManager> rightIRStream;
      boost::shared_ptr<NewFrameCallbackManager> ColorStream;
      boost::shared_ptr<NewFrameCallbackManager> DepthStream;
      boost::shared_ptr<NewFrameCallbackManager> Point3DStream;

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

      std::vector<DeviceInfo>  device_list;

      DeviceInfo               current_device_info;

      ImageRegistrationMode registration_mode = IMAGE_REGISTRATION_OFF;

      TY_STATUS DeviceInit();

      TY_STATUS StreamStart();
      void StreamStop(StreamHandle stream);
      void StreamStopAll();

      TY_STATUS create_leftIR_stream(StreamHandle& stream);

      TY_STATUS create_rightIR_stream(StreamHandle& stream);
      
      TY_STATUS create_color_stream(StreamHandle& stream);

      TY_STATUS create_depth_stream(StreamHandle& stream);

      TY_STATUS create_point3d_stream(StreamHandle& stream);

      const TY_COMPONENT_ID get_stream_component_id(StreamHandle stream);

      const SensorType get_stream_type(StreamHandle stream);

  };

  template<class T>
  class Array
  {
  public:
    /**
      Default constructor.  Creates an empty Array and sets the element count to zero.
    */
    Array() : m_data(NULL), m_count(0), m_owner(false) {}

    /**
      Constructor.  Creates new Array from an existing primitive array of known size.

      @tparam [in] T Object type this Array will contain.
      @param [in] data Pointer to a primitive array of objects of type T.
      @param [in] count Number of elements in the primitive array pointed to by data.
    */
    Array(const T* data, int count) : m_owner(false) { _setData(data, count); }

    /**
      Destructor.  Destroys the Array object.
    */
    ~Array()
    {
      clear();
    }

    /**
      Getter function for the Array size.
      @returns Current number of elements in the Array.
    */
    int getSize() const { return m_count; }

    /**
      Implements the array indexing operator for the Array class.
    */
    const T& operator[](int index) const {return m_data[index];}

    /**
      @internal
      Setter function for data.  Causes this array to wrap an existing primitive array
      of specified type.  The optional data ownership flag controls whether the primitive
      array this Array wraps will be destroyed when this Array is deconstructed.
      @param [in] T Type of objects array will contain.
      @param [in] data Pointer to first object in list.
      @param [in] count Number of objects in list.
      @param [in] isOwner Optional flag to indicate data ownership
    */
    void _setData(const T* data, int count, bool isOwner = false)
    {
      clear();
      m_count = count;
      m_owner = isOwner;
      if (!isOwner)
      {
        m_data = data;
      }
      else
      {
        m_data = new T[count];
        memcpy((void*)m_data, data, count*sizeof(T));
      }
    }

    const void* ptr() const
    {
      return (void*)m_data;
    }

  private:
    Array(const Array<T>&);
    Array<T>& operator=(const Array<T>&);

    void clear()
    {
      if (m_owner && m_data != NULL)
        delete []m_data;
      m_owner = false;
      m_data = NULL;
      m_count = 0;
    }

    const T* m_data;
    int m_count;
    bool m_owner;
  };

  class SensorInfo
  {
  public:
	  SensorType getSensorType() const;
    const Array<VideoMode>& getSupportedVideoModes() const;

  private:
    SensorInfo(const SensorInfo&);
    SensorInfo& operator=(const SensorInfo&);

    SensorInfo();

    SensorInfo(const SensorType type);

    void _setInternal(const SensorType type);

    SensorType m_type;

    Array<VideoMode> m_videoModes;

    friend class VideoStream;
    friend class Device;
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

  class Device;
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

          void parseImageData(TY_IMAGE_DATA* frame)
          {
            std::unique_lock<std::mutex> lock(_mutex);
            if(ptrVideoStream) {
              ptrVideoStream->parseImageData(frame);
              onNewFrame(*ptrVideoStream);
            }
          }

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
          static void callback(StreamHandle streamHandle, void* pCookie,  TY_IMAGE_DATA* frame)
          {
            NewFrameListener* pListener = (NewFrameListener*)pCookie;
            pListener->parseImageData(frame);
          }
      };

      VideoStream(const std::string& name) : stream_desc(name), m_stream(NULL), m_sensorInfo()
      {
      }

      const std::string& desc() { return stream_desc;}

      ~VideoStream()
      {
        destroy();
      }

      TY_STATUS addNewFrameListener(NewFrameListener* pListener);

      void removeNewFrameListener(NewFrameListener* pListener);

      TY_STATUS start();
      void stop();

      void parseImageData(TY_IMAGE_DATA* frame);

      VideoFrameData& getFrame();
      //void CopyFrame(TY_IMAGE_DATA* image);

      TY_STATUS readFrame(VideoFrameRef* pFrame);
      
      VideoMode getVideoMode() const;
  
      float getVerticalFieldOfView() const;

      TY_STATUS create(const Device& device, SensorType sensorType);

      void destroy();

      bool isValid() const;

      const SensorInfo& getSensorInfo() const;

      const StreamHandle& _getHandle() const;

    private:
      const std::string stream_desc;

      StreamHandle m_stream;
      SensorInfo   m_sensorInfo;

      //TY_IMAGE_DATA* m_pFrame;
      VideoFrameData frame;
      
      void _setHandle(StreamHandle stream);
  };

  static const char* ANY_DEVICE = NULL;
  class Device
  {
    public:
      Device() : /*m_pPlaybackControl(NULL), */m_isOwner(true)
	    {
	    }

      TY_STATUS open(const char* uri, const bool auto_reconnect = false);

      void close();

      const DeviceInfo& getDeviceInfo() const
	    {
		    return m_deviceInfo;
	    }

      boost::shared_ptr<PercipioDepthCam> DevicePtr();

      bool hasSensor(SensorType sensorType);

      bool isValid() const;

      bool ResolutionSetting(SensorType sensorType, int width, int height, const std::string& fmt);

      TY_STATUS _setHandle(TY_DEV_HANDLE deviceHandle);

      void setStreamResendEnable(bool enabled);

      void setColorUndistortion(bool enabled);

      void setIRUndistortion(bool enabled);

      bool setDepthSpecFilterEn(bool enabled);
      bool getDepthSpecFilterEn();
      
      bool setDepthSpecFilterSpecSize(int spec_size);
      int getDepthSpecFilterSpecSize();
      
      bool setDepthSpeckFilterDiff(int spec_diff);
      int getDepthSpeckFilterDiff();

      bool setDepthSpeckFilterPhySize(float phy_size);
      float getDepthSpeckFilterPhySize();

      bool setDepthTimeDomainFilterEn(bool enabled);
      bool getDepthTimeDomainFilterEn();
  
      bool setDepthTimeDomainFilterNum(int frames);
      int getDepthTimeDomainFilterNum();

      void setIREnhancement(IREnhanceModel model, int coeff);

      bool isImageRegistrationModeSupported(ImageRegistrationMode mode) const;
      TY_STATUS setImageRegistrationMode(ImageRegistrationMode mode);
      ImageRegistrationMode getImageRegistrationMode() const;

      void ir_stream_topic_enable(bool enabled);
      void color_stream_topic_enable(bool enabled);
      void depth_stream_topic_enable(bool enabled);
      void pointcloud_stream_topic_enable(bool enabled);

      TY_STATUS PropertyGet(TY_COMPONENT_ID comp, TY_FEATURE_ID feat, void* ptr, size_t size);
      TY_STATUS PropertySet(TY_COMPONENT_ID comp, TY_FEATURE_ID feat, void* ptr, size_t size);

    private:
      bool m_isOwner;
      DeviceInfo m_deviceInfo;
      TY_STATUS DeviceGetInfo();

      bool IPv4_verify(const char *ip);
  };

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
		
		virtual void onDeviceConnected(const DeviceInfo*) = 0;
	private:
		static void deviceConnectedCallback(const DeviceInfo* pInfo, void* pCookie)
		{
			DeviceConnectedListener* pListener = (DeviceConnectedListener*)pCookie;
			pListener->onDeviceConnected(static_cast<const DeviceInfo*>(pInfo));
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
		
      virtual void onDeviceDisconnected(const DeviceInfo*) = 0;
    private:
      static void deviceDisconnectedCallback(const DeviceInfo* pInfo, void* pCookie)
      {
        DeviceDisconnectedListener* pListener = (DeviceDisconnectedListener*)pCookie;
        pListener->onDeviceDisconnected(pInfo);
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

      static void enumerateDevices(Array<DeviceInfo>* deviceInfoList);

      static int tycam_log_server_init(bool enable, const std::string& level, int32_t port);

      static bool forceDeviceIP(const std::string& device_URI, const std::string& ip, const std::string& netmask, const std::string& gateway);

      static const char* getExtendedError(TY_STATUS status);
  };
}

#endif