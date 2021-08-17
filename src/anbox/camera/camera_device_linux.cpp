/*
 * Copyright (C) 2011 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * Adapted for Anbox by Alec Barber 2021.
 */

/*
 * Contains code that is used to capture video frames from a camera device
 * on Linux. This code uses V4L2 API to work with camera devices, and requires
 * Linux kernel version at least 2.5
 */
#include <hybris/camera/camera_compatibility_layer.h>
#include <hybris/camera/camera_compatibility_layer_capabilities.h>

#include <hybris/input/input_stack_compatibility_layer.h>
#include <hybris/input/input_stack_compatibility_layer_codes_key.h>
#include <hybris/input/input_stack_compatibility_layer_flags_key.h>
#include <hybris/input/input_stack_compatibility_layer_flags_motion.h>

#include <EGL/egl.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>

#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#include "anbox/logger.h"
#include "anbox/camera/camera_device.h"
#include "anbox/camera/camera_format_converters.h"

#define  E(...)    ERROR(__VA_ARGS__)
#define  W(...)    WARNING(__VA_ARGS__)
#define  D(...)    DEBUG(__VA_ARGS__)
#define  D_ACTIVE  1

#define CLEAR(x) memset (&(x), 0, sizeof(x))

namespace {

/* Pixel format descriptor.
 * Instances of this descriptor are created during camera device enumeration,
 * and an instance of this structure describing pixel format chosen for the
 * camera emulation is saved by the camera factory service to represent an
 * emulating camera properties.
 */
struct QemuPixelFormat {
  /* Pixel format in V4L2_PIX_FMT_XXX form. */
  uint32_t                                   format;
  /* Frame dimensions supported by this format. */
  std::shared_ptr<std::vector<anbox::camera::CameraFrameDim>> dims;
};

/* Describes a framebuffer. */
typedef struct CameraFrameBuffer {
  /* Framebuffer data. */
  uint8_t*    data;
  /* Framebuffer data size. */
  size_t      size;
} CameraFrameBuffer;

/* Defines type of the I/O used to obtain frames from the device. */
enum CameraIoType {
  /* Framebuffers are shared via memory mapping. */
  CAMERA_IO_MEMMAP,
  /* Framebuffers are available via user pointers. */
  CAMERA_IO_USERPTR,
  /* Framebuffers are to be read from the device. */
  CAMERA_IO_DIRECT
};

/* Preferred pixel formats arranged from the most to the least desired.
 *
 * More than anything else this array is defined by an existance of format
 * conversion between the camera supported formats, and formats that are
 * supported by camera framework in the guest system. Currently, guest supports
 * only YV12 pixel format for data, and RGB32 for preview. So, this array should
 * contain only those formats, for which converters are implemented. Generally
 * speaking, the order in which entries should be arranged in this array matters
 * only as far as conversion speed is concerned. So, formats with the fastest
 * converters should be put closer to the top of the array, while slower ones
 * should be put closer to the bottom. But as far as functionality is concerned,
 * the orser doesn't matter, and any format can be placed anywhere in this array,
 * as long as conversion for it exists.
 */
static const std::vector<uint32_t> _preferred_formats =
{
  /* Native format for the emulated camera: no conversion at all. */
  V4L2_PIX_FMT_YUV420,
  V4L2_PIX_FMT_YVU420,
  /* Continue with YCbCr: less math than with RGB */
  V4L2_PIX_FMT_NV12,
  V4L2_PIX_FMT_NV21,
  V4L2_PIX_FMT_YUYV,
  /* End with RGB. */
  V4L2_PIX_FMT_RGB32,
  V4L2_PIX_FMT_RGB24,
  V4L2_PIX_FMT_RGB565,
};

/*******************************************************************************
 *                     Helper routines
 ******************************************************************************/

/*******************************************************************************
 *                     CameraFrameBuffer routines
 ******************************************************************************/

/* Frees array of framebuffers, depending on the I/O method the array has been
 * initialized for.
 * Note that this routine doesn't frees the array itself.
 * Param:
 *  fbs - Array data
 *  io_type - Type of the I/O the array has been initialized for.
 */
static void _free_framebuffers(std::vector<CameraFrameBuffer> fbs,
                               CameraIoType io_type) {
    /* Free framebuffers. */
    for (auto fb : fbs) {
        if (fb.data != NULL) {
            free(fb.data);
            fb.data = NULL;
            fb.size = 0;
        }
    }
}

}

namespace anbox {
namespace camera {

/*******************************************************************************
 *                     CameraDevice routines
 ******************************************************************************/

/*
 * Describes a connection to an actual camera device.
 */
class LinuxCameraDevice : public CameraDevice {
 public:
  /* Camera device name. (default is /dev/video0) */
  char*                       device_name;
  /* Input channel. (default is 0) */
  int                         input_channel;

  /*
   * Set by the framework after initializing camera connection.
   */

  /* Handle to the opened camera device. */
  struct CameraControl* handle;
  /* Device capabilities. */
  struct v4l2_capability      caps;
  /* Actual pixel format reported by the device when capturing is started. */
  struct v4l2_pix_format      actual_pixel_format;
  /* Defines type of the I/O to use to retrieve frames from the device. */
  CameraIoType                io_type;
  /* Allocated framebuffers. */
  std::vector<CameraFrameBuffer> framebuffers;

 public:

  /* Initialises an instance of LinuxCameraDevice structure. Note that this
   *  routine also sets 'opaque' field in the 'header' structure to point back
   *  to the containing LinuxCameraDevice instance.
   */
  LinuxCameraDevice();

  /* Uninitializes and frees CameraDevice structure.
   */
  ~LinuxCameraDevice();

  /* Resets camera device after capturing.
   * Since new capture request may require different frame dimensions we must
   * reset camera device by reopening its handle. Otherwise attempts to set up
   * new frame properties (different from the previous one) may fail.
   */
  void reset();


  /* Opens camera device.
   * Return:
   *  0 on success, != 0 on failure.
   */
  int openDevice();

  /* Collects information about an opened camera device.
   * The information collected in this routine contains list of pixel formats,
   * supported by the device, and list of frame dimensions supported by the
   * camera for each pixel format.
   * Param:
   *  cis - Upon success contains information collected from the camera device.
   * Return:
   *  0 on success, != 0 on failure.
   */
  int getInfo(CameraInfo* cis);

  // 预览数据回调
  static void preview_frame_cb(void* data, uint32_t data_size, void* context);

 public:
  // Inherited interface
  virtual int startCapturing(uint32_t pixel_format,
                             unsigned int frame_width,
                             unsigned int frame_height);
      
  virtual int stopCapturing();

  virtual int readFrame(ClientFrameBuffer* framebuffers,
                        int fbs_num,
                        float r_scale,
                        float g_scale,
                        float b_scale,
                        float exp_comp);
      
  virtual void closeCameraDevice();

};

CameraDevice::~CameraDevice() {}

LinuxCameraDevice::LinuxCameraDevice() : 
    device_name(NULL), input_channel(0), handle(NULL), caps(),
    actual_pixel_format(), io_type(), framebuffers() {}

LinuxCameraDevice::~LinuxCameraDevice() {
  /* Closing handle will also disconnect from the driver. */
  if (handle != NULL) {
    android_camera_disconnect(handle);
  }
  if (device_name != NULL) {
    free(device_name);
  }
  _free_framebuffers(framebuffers, io_type);
}

void LinuxCameraDevice::preview_frame_cb(void* data, uint32_t data_size, void* context){
    LinuxCameraDevice* lcd = (LinuxCameraDevice*)context;

	printf("%s: %d \n", __PRETTY_FUNCTION__, data_size);
	lcd->framebuffers[0].size = data_size;
    //lcd->framebuffers[0].data = static_cast<uint8_t*>(malloc(framebuffers[0].size));
    lcd->framebuffers[0].data = static_cast<uint8_t*>(data);

    lcd->actual_pixel_format.pixelformat = V4L2_PIX_FMT_YUV420;
    lcd->actual_pixel_format.sizeimage = data_size;
}

void LinuxCameraDevice::reset() {
  closeCameraDevice();
  openDevice();
}

int LinuxCameraDevice::openDevice() {
    struct CameraControlListener listener;
	memset(&listener, 0, sizeof(listener));
	listener.on_preview_frame_cb = preview_frame_cb;

    handle = android_camera_connect_to(FRONT_FACING_CAMERA_TYPE,
			&listener);
	if (handle == NULL) {
		printf("Problem connecting to camera");
		return 1;
	}

	listener.context = this;

  return 0;
}

int LinuxCameraDevice::getInfo(CameraInfo* cis) {
  const auto result = std::make_shared<std::vector<CameraFrameDim>>();
  result->resize(1);
  result->at(0).width = 1920;
  result->at(0).height = 1080;

  cis->device_name = strdup(device_name);
  cis->inp_channel = input_channel;
  cis->pixel_format = V4L2_PIX_FMT_YUV420;
  cis->frame_sizes = result;
  cis->in_use = false;

  return 0;
}

int LinuxCameraDevice::startCapturing(uint32_t pixel_format,
                                      unsigned int frame_width,
                                      unsigned int frame_height) {


    actual_pixel_format.width = frame_width;
    actual_pixel_format.height = frame_height;
    android_camera_set_preview_size(handle, frame_width, frame_height);
    // CAMERA_PIXEL_FORMAT_YUV420SP  CAMERA_PIXEL_FORMAT_YUV420P
    android_camera_set_preview_format(handle, CAMERA_PIXEL_FORMAT_YUV420SP);
    //必须要有这个，会在里面建缓冲区，无这个不能生成预览数据和拍照
	GLuint preview_texture_id;
	glGenTextures(1, &preview_texture_id);
	android_camera_set_preview_texture(handle, preview_texture_id);

	android_camera_set_effect_mode(handle, EFFECT_MODE_SEPIA);
	android_camera_set_flash_mode(handle, FLASH_MODE_AUTO);
	android_camera_set_auto_focus_mode(handle, AUTO_FOCUS_MODE_CONTINUOUS_PICTURE);
    //使能回调preview_frame_cb
	android_camera_set_preview_callback_mode(handle,PREVIEW_CALLBACK_ENABLED);
	android_camera_start_preview(handle);

  return 0;
}

int LinuxCameraDevice::stopCapturing() {

  android_camera_stop_preview(handle);

  reset();

  return 0;
}

int LinuxCameraDevice::readFrame(ClientFrameBuffer* framebuffers,
                                 int fbs_num,
                                 float r_scale,
                                 float g_scale,
                                 float b_scale,
                                 float exp_comp) {
  /* Sanity checks. */
  if (handle == NULL) {
    E("Camera device is not opened");
    return -1;
  }

    /* Convert frame to the receiving buffers. */
  return convert_frame(this->framebuffers[0].data,
                        actual_pixel_format.pixelformat,
                        actual_pixel_format.sizeimage,
                        actual_pixel_format.width,
                        actual_pixel_format.height,
                        framebuffers, fbs_num,
                        r_scale, g_scale, b_scale, exp_comp);
}

void LinuxCameraDevice::closeCameraDevice() {
  /* Free capturing framebuffers first. */
    if (!framebuffers.empty()) {
      _free_framebuffers(framebuffers, io_type);
      framebuffers.clear();
    }

    android_camera_disconnect(handle);
}

/*******************************************************************************
 *                     CameraDevice API
 ******************************************************************************/

std::shared_ptr<CameraDevice> CameraDevice::openCameraDevice(const char* name,
                                                             int inp_channel) {
  std::shared_ptr<LinuxCameraDevice> cd = std::make_shared<LinuxCameraDevice>();

  /* Allocate and initialize the descriptor. */
  cd->device_name = name != NULL ? strdup(name) : strdup("camera0");
  cd->input_channel = inp_channel;

  /* Open the device. */
  if (cd->openDevice()) {
    return NULL;
  }

  return cd;
}

std::shared_ptr<std::vector<CameraInfo>>
CameraDevice::enumerateCameraDevices(int max) {
  char dev_name[24];
  const auto result = std::make_shared<std::vector<CameraInfo>>();

  for (int n = 0; n < max; n++) {
    sprintf(dev_name, "camera%d", n);
    const auto cd = openCameraDevice(dev_name, 0);
    if (cd) {
      LinuxCameraDevice* lcd = dynamic_cast<LinuxCameraDevice*>(cd.get());
      CameraInfo ci;
      if (!lcd->getInfo(&ci)) {
        char user_name[24];
        sprintf(user_name, "camera%d", n);
        ci.display_name = strdup(user_name);
        ci.in_use = 0;
        result->push_back(ci);
      }
      cd->closeCameraDevice();
    }
  }

  return result;
}

}
}