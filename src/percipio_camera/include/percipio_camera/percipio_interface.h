/*
 * @Description: 
 * @Author: zxy
 * @Date: 2023-08-04 14:20:07
 * @LastEditors: zxy
 * @LastEditTime: 2023-08-16 10:11:31
 */
#ifndef _PERCIPIO_H_
#define _PERCIPIO_H_

#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread/mutex.hpp>

#include "TYApi.h"
#include "stdio.h"

#include <vector>
#include "Utils.hpp"

#include "deviceinfo.h"
#include "videomode.h"

namespace percipio
{
  enum
  {
    TY_DEVICE_PROPERTY_SERIAL_NUMBER		= 0, // string
    TY_DEVICE_PROPERTY_DEPTH_CALIB_INTRISTIC   = 1, // float
    TY_DEVICE_PROPERTY_COLOR_CALIB_INTRISTIC   = 2, // float
    TY_DEVICE_PROPERTY_COLOR_CALIB_DISTORTION  = 3, // float
  };


  typedef enum
  {
    SENSOR_NONE     = 0,
	  SENSOR_IR_LEFT  = 1,
    SENSOR_IR_RIGHT = 2,
	  SENSOR_COLOR    = 3,
	  SENSOR_DEPTH    = 4,
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
  class percipio_depth_cam
  {
    public:
      percipio_depth_cam() : _M_IFACE(0), _M_DEVICE(0), m_ids(0)
      {
        frameBuffer[0] = NULL;
        frameBuffer[1] = NULL;
        isRuning = false;

        current_depth_width = 0;
        current_depth_height = 0;

        current_rgb_width = 0;
        current_rgb_height = 0;

        device_list.clear();
      }
      ~percipio_depth_cam()
      {
      }

      TY_STATUS initialize();
      void GetDeviceList(DeviceInfo** device_info_ptr, int* cnt);

      TY_STATUS open(const char* sn);

      const DeviceInfo& get_current_device_info();

      bool set_image_mode(TY_COMPONENT_ID  comp, int width, int height);
      
      //static DeviceCallbacks* cb;
      //static DeviceDisconnectedListener* pListener;
      static void eventCallback(TY_EVENT_INFO *event_info, void *userdata);

      TY_STATUS RegisterDeviceCallbacks(DeviceCallbacks* callback, void* listener);

      void UnregisterDeviceCallbacks(void* listener);

      bool isValid();

      TY_STATUS get_device_info(TY_DEVICE_BASE_INFO& info);

      void close();

      const uint32_t&  get_components() const ;

      bool DeviceSetColorUndistortionEnable(bool enable);

      bool DeviceIsImageRegistrationModeSupported() const;

      TY_STATUS DeviceSetImageRegistrationMode(ImageRegistrationMode mode);
      ImageRegistrationMode DeviceGetImageRegistrationMode();
      TY_STATUS MapDepthFrameToColorCoordinate(VideoFrameData* src, VideoFrameData* dst);
      TY_STATUS FrameDecoder(VideoFrameData& src, VideoFrameData& dst);
      TY_STATUS parseColorStream(VideoFrameData* src, VideoFrameData* dst);
      TY_STATUS parseDepthStream(VideoFrameData* src, VideoFrameData* dst);

      bool isDepthColorSyncSupport();
      TY_STATUS DeviceEnableDepthColorSync();
      TY_STATUS DeviceDisableDepthColorSync();
      bool DeviceGetDepthColorSyncEnabled();

      template <class T>
      TY_STATUS getProperty(StreamHandle stream, uint32_t propertyId, void* value);
      template <class T>
      TY_STATUS setProperty(StreamHandle stream, uint32_t propertyId, const T& value);

      TY_STATUS DeviceGetProperty(int propertyId, void* data, int* dataSize);

      TY_STATUS DeviceSetProperty(int propertyId, const void* data, int* dataSize);
    
      const std::vector<VideoMode>& getVideoModeList(SensorType type) const;

      TY_STATUS CreateStream(SensorType sensorType, StreamHandle& streamHandle);

      VideoMode get_current_video_mode(StreamHandle stream);

      TY_STATUS set_current_video_mode(StreamHandle stream, const VideoMode& videoMode);

      TY_STATUS StreamRegisterNewFrameCallback(StreamHandle stream, void* listener, NewFrameCallback cb);

      void StreamUnregisterNewFrameCallback(StreamHandle stream);

      static void* fetch_thread(void* ptr);

      bool HasStream();

      void enable_stream(StreamHandle stream);

      void disable_stream(StreamHandle stream);

      TY_STATUS StartCapture();
      void StopCapture(StreamHandle stream);
      

      SensorType StreamGetSensorInfo(StreamHandle stream);

      bool                   isRuning;
      const TY_DEV_HANDLE getCurrentDeviceHandle() const;
      
    private:
      TY_INTERFACE_HANDLE _M_IFACE;
      TY_DEV_HANDLE       _M_DEVICE;
      TY_COMPONENT_ID     m_ids;

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

      float f_depth_scale_unit = 1.0f;

      std::vector<DeviceInfo>  device_list;

      DeviceInfo               current_device_info;

      ImageRegistrationMode registration_mode = IMAGE_REGISTRATION_OFF;

      TY_STATUS StreamStart();
      void StreamStop(StreamHandle stream);
      void StreamStopAll();

      void generate_video_mode(const std::vector<TY_ENUM_ENTRY>& feature_info, std::vector<VideoMode>& videomode);
#if 0    
      void onNewFrame();

      void newFrameCallback();
#endif
      TY_STATUS create_leftIR_stream(StreamHandle& stream);

      TY_STATUS create_rightIR_stream(StreamHandle& stream);
      
      TY_STATUS create_color_stream(StreamHandle& stream);

      TY_STATUS create_depth_stream(StreamHandle& stream);

      const TY_COMPONENT_ID get_stream_component_id(StreamHandle stream);

      const SensorType get_stream_type(StreamHandle stream);

      uint32_t get_stream_image_mode(TY_COMPONENT_ID comp, const VideoMode& videoMode);

      uint32_t get_leftir_stream_image_mode(const VideoMode& videoMode);

      uint32_t get_rightir_stream_image_mode(const VideoMode& videoMode);

      uint32_t get_rgb_stream_image_mode(const VideoMode& videoMode);

      uint32_t get_depth_stream_image_mode(const VideoMode& videoMode);
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

    //const OniSensorInfo* m_pInfo;
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
      int32_t  getPixelFormat() const {return pixelFormat;}
      void*    getData() const {return buffer;}
      int32_t  getFrameIndex() const {return imageIndex;}
      int32_t  getComponentID() const {return componentID;};


      void  setTimestamp(uint64_t time);
      void  setWidth(int32_t w);
      void  setHeight(int32_t h);
      void  Resize(int32_t sz);
      void  setPixelFormat(int32_t fmt);
      void  setFrameIndex(int32_t idx);
      void  setComponentID(int32_t compID);

      void  setData(TY_IMAGE_DATA* data);
      void  clone(const VideoFrameData& frame);

    private:
      bool            m_isOwner;
      uint64_t        timestamp;      ///< Timestamp in microseconds
      int32_t         imageIndex;     ///< image index used in trigger mode
      int32_t         status;         ///< Status of this buffer
      int32_t         componentID;    ///< Where current data come from
      int32_t         size;           ///< Buffer size
      void*           buffer;         ///< Pointer to data buffer
      int32_t         width;          ///< Image width in pixels
      int32_t         height;         ///< Image height in pixels
      int32_t         pixelFormat;    ///< Pixel format, see TY_PIXEL_FORMAT_LIST
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
  class CameraSettings
  {
public:
    TY_STATUS setLaserPower(int power);
    TY_STATUS setAutoExposureEnabled(bool enabled);
    TY_STATUS setAutoWhiteBalanceEnabled(bool enabled);
    TY_STATUS setPixelsAnalogGain(int gain);
    TY_STATUS setPixelsRedGain(int gain);
    TY_STATUS setPixelsGreenGain(int gain);
    TY_STATUS setPixelsBlueGain(int gain);
    TY_STATUS setGain(int gain);
    TY_STATUS setExposure(int exposure);
    TY_STATUS setTOFCamDepthChannel(int channel);
    TY_STATUS setTOFCamDepthQuality(int quality);

    TY_STATUS setDepthSgbmImageChanNumber(int value);
    TY_STATUS setDepthSgbmDispNumber(int value);
    TY_STATUS setDepthSgbmDispOffset(int value);
    TY_STATUS setDepthSgbmMatchWinHeight(int value);
    TY_STATUS setDepthSgbmSemiP1(int value);
    TY_STATUS setDepthSgbmSemiP2(int value);
    TY_STATUS setDepthSgbmUniqueFactor(int value);
    TY_STATUS setDepthSgbmUniqueAbsDiff(int value);
    TY_STATUS setDepthSgbmCostParam(int value);
    TY_STATUS setDepthSgbmHalfWinSizeEn(int value);
    TY_STATUS setDepthSgbmMatchWinWidth(int value);
    TY_STATUS setDepthSgbmMedianFilterEn(int value);
    TY_STATUS setDepthSgbmLRCCheckEn(int value);
    TY_STATUS setDepthSgbmLRCMaxDiff(int value);
    TY_STATUS setDepthSgbmMedianFilterThresh(int value);
    TY_STATUS setDepthSgbmSemiP1Scale(int value);
    //
    bool getLaserPower(int* power) const;
    bool getAutoExposureEnabled(bool* enable) const;
    bool getAutoWhiteBalanceEnabled(bool* enable) const;
    bool getAnalogGain(int* gain) const;
    bool getRedGain(int* gain) const;
    bool getGreenGain(int* gain) const;
    bool getBlueGain(int* gain) const;
    int getExposure(int* exposure);
    int getGain(int* gain);
    int getTOFCamDepthChannel(int* channel);
    int getTOFCamDepthQuality(int* quality);
    bool isValid() const;
  private:
    template <class T>
    TY_STATUS getProperty(uint32_t propertyId, T* value) const;
    template <class T>
    TY_STATUS setProperty(uint32_t propertyId, const T& value);

    friend class VideoStream;
    CameraSettings(VideoStream* pStream);

    VideoStream* m_pStream;
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

      VideoStream() : m_stream(NULL), m_sensorInfo(), m_pCameraSettings(NULL)
      {
      }

      explicit VideoStream(StreamHandle handle) : m_stream(NULL), m_sensorInfo() , m_pCameraSettings(NULL)
      {
        _setHandle(handle);
      }

      ~VideoStream()
      {
        destroy();
      }

      CameraSettings* getCameraSettings();

      TY_STATUS addNewFrameListener(NewFrameListener* pListener);

      void removeNewFrameListener(NewFrameListener* pListener);

      TY_STATUS start();
      void stop();

      void parseImageData(TY_IMAGE_DATA* frame);

      VideoFrameData& getFrame();
      //void CopyFrame(TY_IMAGE_DATA* image);

      TY_STATUS readFrame(VideoFrameRef* pFrame);
      
      TY_STATUS getProperty(int propertyId, void* data, int* dataSize) const;

      TY_STATUS setProperty(int propertyId, const void* data, int dataSize);

      VideoMode getVideoMode() const;

      TY_STATUS setVideoMode(const VideoMode& videoMode);
  
      float getVerticalFieldOfView() const;

      TY_STATUS create(const Device& device, SensorType sensorType);

      void destroy();

      bool isValid() const;

      const SensorInfo& getSensorInfo() const;

      const StreamHandle& _getHandle() const;

    private:
      StreamHandle m_stream;
      SensorInfo   m_sensorInfo;
      CameraSettings* m_pCameraSettings;

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

      TY_STATUS open(const char* uri);

      void close();

      const DeviceInfo& getDeviceInfo() const
	    {
		    return m_deviceInfo;
	    }

      bool hasSensor(SensorType sensorType);

      bool isValid() const;

      bool ReslotionSetting(SensorType sensorType, int width, int height);

      TY_STATUS _setHandle(TY_DEV_HANDLE deviceHandle);

      void setColorUndistortion(bool enabled);

      bool isImageRegistrationModeSupported(ImageRegistrationMode mode) const;
      TY_STATUS setImageRegistrationMode(ImageRegistrationMode mode);
      ImageRegistrationMode getImageRegistrationMode() const;

      bool isDepthColorSyncSupport();
      TY_STATUS setDepthColorSyncEnabled(bool isEnabled);
      bool getDepthColorSyncEnabled();

      TY_STATUS getProperty(int propertyId, void* data, int* dataSize) const;

      TY_STATUS setProperty(int propertyId, const void* data, int* dataSize);
    private:
      bool m_isOwner;
      DeviceInfo m_deviceInfo;
      TY_STATUS DeviceGetInfo();
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