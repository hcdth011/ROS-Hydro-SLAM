/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2012 Ken Tossell
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include "libuvc_camera/camera_driver.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <image_transport/camera_publisher.h>
#include <dynamic_reconfigure/server.h>
#include <libuvc/libuvc.h>

namespace libuvc_camera {

CameraDriver::CameraDriver(ros::NodeHandle nh, ros::NodeHandle priv_nh)
  : nh_(nh), priv_nh_(priv_nh),
    state_(kInitial),
    ctx_(NULL), dev_(NULL), devh_(NULL), rgb_frame_(NULL),
    it_(nh_),
    config_server_(mutex_, priv_nh_),
    config_changed_(false),
    cinfo_manager_(nh) {
  cam_pub_ = it_.advertiseCamera("image_raw", 1, false);
}

CameraDriver::~CameraDriver() {
  if (rgb_frame_)
    uvc_free_frame(rgb_frame_);

  if (ctx_)
    uvc_exit(ctx_);  // Destroys dev_, devh_, etc.
}

bool CameraDriver::Start() {
  assert(state_ == kInitial);

  uvc_error_t err;

  err = uvc_init(&ctx_, NULL);

  if (err != UVC_SUCCESS) {
    uvc_perror(err, "ERROR: uvc_init");
    return false;
  }

  state_ = kStopped;

  config_server_.setCallback(boost::bind(&CameraDriver::ReconfigureCallback, this, _1, _2));

  return state_ == kRunning;
}

void CameraDriver::Stop() {
  boost::recursive_mutex::scoped_lock(mutex_);

  assert(state_ != kInitial);

  if (state_ == kRunning)
    CloseCamera();

  assert(state_ == kStopped);

  uvc_exit(ctx_);
  ctx_ = NULL;

  state_ = kInitial;
}

void CameraDriver::ReconfigureCallback(UVCCameraConfig &new_config, uint32_t level) {
  boost::recursive_mutex::scoped_lock(mutex_);

  if ((level & kReconfigureClose) == kReconfigureClose) {
    if (state_ == kRunning)
      CloseCamera();
  }

  if (state_ == kStopped) {
    OpenCamera(new_config);
  }

  if (new_config.camera_info_url != config_.camera_info_url)
    cinfo_manager_.loadCameraInfo(new_config.camera_info_url);

  if (state_ == kRunning) {
#define PARAM_INT(name, fn, value) if (new_config.name != config_.name) { \
      int val = (value);                                                \
      if (uvc_set_##fn(devh_, val)) {                                   \
        ROS_WARN("Unable to set " #name " to %d", val);                 \
        new_config.name = config_.name;                                 \
      }                                                                 \
    }

    PARAM_INT(scanning_mode, scanning_mode, new_config.scanning_mode);
    PARAM_INT(auto_exposure, ae_mode, 1 << new_config.auto_exposure);
    PARAM_INT(auto_exposure_priority, ae_priority, new_config.auto_exposure_priority);
    PARAM_INT(exposure_absolute, exposure_abs, new_config.exposure_absolute * 10000);
    // TODO: iris_absolute
    PARAM_INT(auto_focus, focus_auto, new_config.auto_focus ? 1 : 0);
    PARAM_INT(focus_absolute, focus_abs, new_config.focus_absolute);

    if (new_config.pan_absolute != config_.pan_absolute || new_config.tilt_absolute != config_.tilt_absolute) {
      if (uvc_set_pantilt_abs(devh_, new_config.pan_absolute, new_config.tilt_absolute)) {
        ROS_WARN("Unable to set pantilt to %d, %d", new_config.pan_absolute, new_config.tilt_absolute);
        new_config.pan_absolute = config_.pan_absolute;
        new_config.tilt_absolute = config_.tilt_absolute;
      }
    }
    // TODO: roll_absolute
    // TODO: privacy
    // TODO: backlight_compensation
    // TODO: brightness
    // TODO: contrast
    // TODO: gain
    // TODO: power_line_frequency
    // TODO: auto_hue
    // TODO: saturation
    // TODO: sharpness
    // TODO: gamma
    // TODO: auto_white_balance
    // TODO: white_balance_temperature
    // TODO: white_balance_BU
    // TODO: white_balance_RV
  }

  config_ = new_config;
}

void CameraDriver::ImageCallback(uvc_frame_t *frame) {
  // TODO: Switch to {frame}'s timestamp once that becomes reliable.
  ros::Time timestamp = ros::Time::now();

  boost::recursive_mutex::scoped_lock(mutex_);

  assert(state_ == kRunning);
  assert(rgb_frame_);

  uvc_error_t conv_ret = uvc_any2rgb(frame, rgb_frame_);

  if (conv_ret != UVC_SUCCESS) {
    uvc_perror(conv_ret, "Couldn't convert frame to RGB");
    return;
  }

  sensor_msgs::Image::Ptr image(new sensor_msgs::Image());
  image->width = config_.width;
  image->height = config_.height;
  image->encoding = "rgb8";
  image->step = image->width * 3;
  image->data.resize(image->step * image->height);
  memcpy(&(image->data[0]), rgb_frame_->data, rgb_frame_->data_bytes);

  sensor_msgs::CameraInfo::Ptr cinfo(
    new sensor_msgs::CameraInfo(cinfo_manager_.getCameraInfo()));

  image->header.frame_id = config_.frame_id;
  image->header.stamp = timestamp;
  cinfo->header.frame_id = config_.frame_id;
  cinfo->header.stamp = timestamp;

  cam_pub_.publish(image, cinfo);

  if (config_changed_) {
    config_server_.updateConfig(config_);
    config_changed_ = false;
  }
}

/* static */ void CameraDriver::ImageCallbackAdapter(uvc_frame_t *frame, void *ptr) {
  CameraDriver *driver = static_cast<CameraDriver*>(ptr);

  driver->ImageCallback(frame);
}

void CameraDriver::AutoControlsCallback(
  enum uvc_status_class status_class,
  int event,
  int selector,
  enum uvc_status_attribute status_attribute,
  void *data, size_t data_len) {
  boost::recursive_mutex::scoped_lock(mutex_);

  printf("Controls callback. class: %d, event: %d, selector: %d, attr: %d, data_len: %u\n",
         status_class, event, selector, status_attribute, data_len);

  if (status_attribute == UVC_STATUS_ATTRIBUTE_VALUE_CHANGE) {
    switch (status_class) {
    case UVC_STATUS_CLASS_CONTROL_CAMERA: {
      switch (selector) {
      case UVC_CT_EXPOSURE_TIME_ABSOLUTE_CONTROL:
        uint8_t *data_char = (uint8_t*) data;
        uint32_t exposure_int = ((data_char[0]) | (data_char[1] << 8) |
                                 (data_char[2] << 16) | (data_char[3] << 24));
        config_.exposure_absolute = exposure_int * 0.0001;
        config_changed_ = true;
        break;
      }
      break;
    }
    case UVC_STATUS_CLASS_CONTROL_PROCESSING: {
      switch (selector) {
      case UVC_PU_WHITE_BALANCE_TEMPERATURE_CONTROL:
        uint8_t *data_char = (uint8_t*) data;
        config_.white_balance_temperature = 
          data_char[0] | (data_char[1] << 8);
        config_changed_ = true;
        break;
      }
      break;
    }
    }

    // config_server_.updateConfig(config_);
  }
}

/* static */ void CameraDriver::AutoControlsCallbackAdapter(
  enum uvc_status_class status_class,
  int event,
  int selector,
  enum uvc_status_attribute status_attribute,
  void *data, size_t data_len,
  void *ptr) {
  CameraDriver *driver = static_cast<CameraDriver*>(ptr);

  driver->AutoControlsCallback(status_class, event, selector,
                               status_attribute, data, data_len);
}

void CameraDriver::OpenCamera(UVCCameraConfig &new_config) {
  assert(state_ == kStopped);

  int vendor_id = strtol(new_config.vendor.c_str(), NULL, 0);
  int product_id = strtol(new_config.product.c_str(), NULL, 0);

  ROS_INFO("Opening camera with vendor=0x%x, product=0x%x, serial=\"%s\", index=%d",
           vendor_id, product_id, new_config.serial.c_str(), new_config.index);

  uvc_error_t find_err = uvc_find_device(
    ctx_, &dev_,
    vendor_id,
    product_id,
    new_config.serial.empty() ? NULL : new_config.serial.c_str());

  // TODO: index

  if (find_err != UVC_SUCCESS) {
    uvc_perror(find_err, "uvc_find_device");
    return;
  }

  uvc_error_t open_err = uvc_open(dev_, &devh_);
  if (open_err != UVC_SUCCESS) {
    switch (open_err) {
    case UVC_ERROR_ACCESS:
#ifdef __linux__
      ROS_ERROR("Permission denied opening /dev/bus/usb/%03d/%03d",
                uvc_get_bus_number(dev_), uvc_get_device_address(dev_));
#else
      ROS_ERROR("Permission denied opening device %d on bus %d",
                uvc_get_device_address(dev_), uvc_get_bus_number(dev_));
#endif
      break;
    default:
#ifdef __linux__
      ROS_ERROR("Can't open /dev/bus/usb/%03d/%03d: %s (%d)",
                uvc_get_bus_number(dev_), uvc_get_device_address(dev_),
                uvc_strerror(open_err), open_err);
#else
      ROS_ERROR("Can't open device %d on bus %d: %s (%d)",
                uvc_get_device_address(dev_), uvc_get_bus_number(dev_),
                uvc_strerror(open_err), open_err);
#endif
      break;
    }

    uvc_unref_device(dev_);
    return;
  }

  uvc_set_status_callback(devh_, &CameraDriver::AutoControlsCallbackAdapter, this);

  uvc_stream_ctrl_t ctrl;
  uvc_error_t mode_err = uvc_get_stream_ctrl_format_size(
    devh_, &ctrl,
    UVC_COLOR_FORMAT_UNCOMPRESSED,
    new_config.width, new_config.height,
    new_config.frame_rate);

  if (mode_err != UVC_SUCCESS) {
    uvc_perror(mode_err, "uvc_get_stream_ctrl_format_size");
    uvc_close(devh_);
    uvc_unref_device(dev_);
    return;
  }

  uvc_error_t stream_err = uvc_start_iso_streaming(devh_, &ctrl, &CameraDriver::ImageCallbackAdapter, this);

  if (stream_err != UVC_SUCCESS) {
    uvc_perror(stream_err, "uvc_start_iso_streaming");
    uvc_close(devh_);
    uvc_unref_device(dev_);
    return;
  }

  if (rgb_frame_)
    uvc_free_frame(rgb_frame_);

  rgb_frame_ = uvc_allocate_frame(new_config.width * new_config.height * 3);
  assert(rgb_frame_);

  state_ = kRunning;
}

void CameraDriver::CloseCamera() {
  assert(state_ == kRunning);

  uvc_close(devh_);
  devh_ = NULL;

  uvc_unref_device(dev_);
  dev_ = NULL;

  state_ = kStopped;
}

};
