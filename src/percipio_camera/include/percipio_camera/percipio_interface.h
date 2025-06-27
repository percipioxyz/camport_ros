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
    IMAGE_REGISTRATION_OFF				    = 0,
    IMAGE_REGISTRATION_DEPTH_TO_COLOR	= 1,
    IMAGE_REGISTRATION_COLOR_TO_DEPTH	= 2,
  } ImageRegistrationMode;
  
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
      bool isInvalid();
      void enableCallback(bool en);

      void* frame_listener;
      bool is_enable;
      NewFrameCallback cb;
  };
    
  typedef void* CallbackHandle;
  typedef void (*DeviceInfoCallback)(const DeviceInfo* pInfo, void* pCookie);
  typedef void (*DeviceStateCallback)(const DeviceInfo* pInfo, DeviceState deviceState, void* pCookie);
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

  static std::vector<DeviceCallback_t> cb_list;

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

    virtual TY_STATUS AcquisitionInit() = 0;

    virtual TY_STATUS SetImageMode(const SensorType sensorType, const int width, const int height, const std::string& fmt) = 0;

    virtual TY_STATUS PreSetting() = 0;

    virtual TY_STATUS EnableColorStream(const bool en) = 0;
    virtual TY_STATUS EnableDepthStream(const bool en) = 0;
    virtual TY_STATUS EnableLeftIRStream(const bool en) = 0;
    virtual TY_STATUS EnableRightIRStream(const bool en) = 0;
    
    const TY_DEV_HANDLE hDevice;

    bool need_depth_undistortion;

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

      TY_STATUS openWithSN(const char* sn, const bool auto_reconnect = false);
      TY_STATUS openWithIP(const char* ip, const bool auto_reconnect = false);

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

      bool DepthStreamSetSpeckFilterEn(bool enabled);
      bool DepthStreamGetSpeckFilterEn();

      bool DepthStreamSetSpeckFilterSpecSize(int spec_size);
      int DepthStreamGetSpeckFilterSpecSize();
  
      bool DepthStreamSetSpeckFilterDiff(int spec_diff);
      int DepthStreamGetSpeckFilterDiff();

      bool DepthStreamSetTimeDomainFilterEn(bool enabled);
      bool DepthStreamGetTimeDomainFilterEn();
      bool DepthStreamSetTimeDomainFilterFCnt(int frameCnt);
      int  DepthStreamGetTimeDomainFilterFCnt();

      bool DeviceIsImageRegistrationModeSupported() const;

      //bool load_default_parameter();

      TY_STATUS DeviceSetImageRegistrationMode(ImageRegistrationMode mode);
      ImageRegistrationMode DeviceGetImageRegistrationMode();
      TY_STATUS MapDepthFrameToColorCoordinate(VideoFrameData* src, VideoFrameData* dst);
      TY_STATUS MapXYZ48FrameToColorCoordinate(VideoFrameData* src, VideoFrameData* dst);
      TY_STATUS FrameDecoder(VideoFrameData& src, VideoFrameData& dst);
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

      static void* fetch_thread(void* ptr);

      static void* device_offline_reconnect(void* ptr);

      bool HasStream();

      void StreamEnable(StreamHandle stream);
      void StreamDisable(StreamHandle stream);

      TY_STATUS StartCapture();
      void StopCapture(StreamHandle stream);

      float getDepthScaleUnit() { return f_depth_scale_unit; }
      TY_CAMERA_INTRINSIC getDepthIntr() { return depth_intr; }
      TY_CAMERA_INTRINSIC getColorIntr() { return color_intr; }
      TY_CAMERA_DISTORTION getColorDist() { return color_calib.distortion; }
      
      SensorType StreamGetSensorInfo(StreamHandle stream);

      std::string            m_current_device_sn;
      static bool            isOffline;
      bool                   isRuning;
      const TY_DEV_HANDLE getCurrentDeviceHandle() const;
      
      std::mutex detect_mutex;
      std::condition_variable detect_cond;
      static bool   b_device_opened;
      pthread_t     device_status_listen;
      
    private:
      TY_INTERFACE_HANDLE _M_IFACE;
      TY_DEV_HANDLE       _M_DEVICE;
      TY_COMPONENT_ID     m_ids;

      GigEVersion         gige_version = GigeE_2_0;

      boost::shared_ptr<GigEBase> m_gige_dev;

      std::vector<VideoMode> leftIRVideoMode;
      std::vector<VideoMode> rightIRVideoMode;
      std::vector<VideoMode> RGBVideoMode;
      std::vector<VideoMode> DepthVideoMode;

      char*                  frameBuffer[2];
      pthread_t              frame_fetch_thread;

      boost::shared_ptr<NewFrameCallbackManager> leftIRStream;
      boost::shared_ptr<NewFrameCallbackManager> rightIRStream;
      boost::shared_ptr<NewFrameCallbackManager> ColorStream;
      boost::shared_ptr<NewFrameCallbackManager> DepthStream;
      boost::shared_ptr<NewFrameCallbackManager> Point3DStream;

      boost::shared_ptr<DepthTimeDomainMgr> DepthDomainTimeFilterMgrPtr;

      //std::mutex m_mutex;
      pthread_mutex_t m_mutex = PTHREAD_MUTEX_INITIALIZER;

      int32_t current_depth_width;
      int32_t current_depth_height;
      
      int32_t current_rgb_width;
      int32_t current_rgb_height;

      TY_CAMERA_CALIB_INFO depth_calib;
      TY_CAMERA_CALIB_INFO color_calib;

      bool b_stream_with_color = false;
      TY_CAMERA_INTRINSIC depth_intr;
      TY_CAMERA_INTRINSIC color_intr;

      bool color_undistortion = false;
      bool depth_distortion = false;

      bool depth_stream_spec_enable = false;
      int  depth_stream_spec_size = 150; //default
      int  depth_stream_spec_diff = 64;

      bool depth_time_domain_enable = false;
      int  depth_time_domain_frame_cnt = 3;

      float f_depth_scale_unit = 1.0f;

      std::vector<DeviceInfo>  device_list;

      DeviceInfo               current_device_info;

      ImageRegistrationMode registration_mode = IMAGE_REGISTRATION_OFF;

      TY_STATUS DeviceInit(const bool auto_reconnect);

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
      uint32_t        pixelFormat;    ///< Pixel format, see TY_PIXEL_FORMAT_LIST
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

  class VideoStream;
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

        private:
          friend class VideoStream;
          static void callback(StreamHandle streamHandle, void* pCookie,  TY_IMAGE_DATA* frame)
          {      
            NewFrameListener* pListener = (NewFrameListener*)pCookie;
            VideoStream stream;
            stream._setHandle(streamHandle);
            stream.parseImageData(frame);
            pListener->onNewFrame(stream);
            stream._setHandle(NULL);
          }
      };

      VideoStream() : m_stream(NULL), m_sensorInfo()
      {
      }

      explicit VideoStream(StreamHandle handle) : m_stream(NULL), m_sensorInfo()
      {
        _setHandle(handle);
      }

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

      bool setDepthSpecFilterEn(bool enabled);
      bool getDepthSpecFilterEn();
      
      bool setDepthSpecFilterSpecSize(int spec_size);
      int getDepthSpecFilterSpecSize();
      
      bool setDepthSpeckFilterDiff(int spec_diff);
      int getDepthSpeckFilterDiff();

      bool setDepthTimeDomainFilterEn(bool enabled);
      bool getDepthTimeDomainFilterEn();
  
      bool setDepthTimeDomainFilterNum(int frames);
      int getDepthTimeDomainFilterNum();

      bool isImageRegistrationModeSupported(ImageRegistrationMode mode) const;
      TY_STATUS setImageRegistrationMode(ImageRegistrationMode mode);
      ImageRegistrationMode getImageRegistrationMode() const;

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

      static const char* getExtendedError(TY_STATUS status);

  };
}

#endif