/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
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

#include "compressed_image_transport/compressed_publisher.h"

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/imgcodecs.hpp>

#include "compressed_image_transport/compression_common.h"
#include "compressed_image_transport/encoder.h"
#include "compressed_image_transport/encoders/jpeg_encoder.h"
#include "compressed_image_transport/encoders/png_encoder.h"
#include "compressed_image_transport/encoders/tiff_encoder.h"
#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/parameter_events_filter.hpp>

#include <sstream>
#include <vector>

using namespace cv;
using namespace std;

namespace enc = sensor_msgs::image_encodings;

namespace compressed_image_transport
{

enum compressedParameters
{
  FORMAT = 0,
  PNG_LEVEL,
  JPEG_QUALITY,
  TIFF_RESOLUTION_UNIT,
  TIFF_XDPI,
  TIFF_YDPI,
  USE_CACHE
};

const struct ParameterDefinition kParameters[] =
{
  { //FORMAT - Compression format to use "jpeg", "png" or "tiff".
    ParameterValue("jpeg"),
    ParameterDescriptor()
      .set__name("format")
      .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
      .set__description("Compression method")
      .set__read_only(false)
      .set__additional_constraints("Supported values: [jpeg, png, tiff]")
  },
  { //PNG_LEVEL - PNG Compression Level from 0 to 9.  A higher value means a smaller size.
    ParameterValue((int)3), //Default to OpenCV default of 3
    ParameterDescriptor()
      .set__name("png_level")
      .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
      .set__description("Compression level for PNG format")
      .set__read_only(false)
      .set__integer_range(
        {rcl_interfaces::msg::IntegerRange()
          .set__from_value(0)
          .set__to_value(9)
          .set__step(1)})
  },
  { //JPEG_QUALITY - JPEG Quality from 0 to 100 (higher is better quality).
    ParameterValue((int)95), //Default to OpenCV default of 95.
    ParameterDescriptor()
      .set__name("jpeg_quality")
      .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
      .set__description("Image quality for JPEG format")
      .set__read_only(false)
      .set__integer_range(
        {rcl_interfaces::msg::IntegerRange()
          .set__from_value(1)
          .set__to_value(100)
          .set__step(1)})
  },
  { //TIFF_RESOLUTION_UNIT - TIFF resolution unit, can be one of "none", "inch", "centimeter".
    ParameterValue("inch"),
    ParameterDescriptor()
      .set__name("tiff.res_unit")
      .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
      .set__description("tiff resolution unit")
      .set__read_only(false)
      .set__additional_constraints("Supported values: [none, inch, centimeter]")
  },
  { //TIFF_XDPI
    ParameterValue((int)-1),
    ParameterDescriptor()
      .set__name("tiff.xdpi")
      .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
      .set__description("tiff xdpi")
      .set__read_only(false)
  },
  { //TIFF_YDPI
    ParameterValue((int)-1),
    ParameterDescriptor()
      .set__name("tiff.ydpi")
      .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
      .set__description("tiff ydpi")
      .set__read_only(false)
  },
  { //TIFF_YDPI
    ParameterValue((int)-1),
    ParameterDescriptor()
      .set__name("use_cache")
      .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
      .set__description("Cache compressed image message for reuse")
      .set__read_only(false)
  }
};

void CompressedPublisher::advertiseImpl(
  rclcpp::Node* node,
  const std::string& base_topic,
  rmw_qos_profile_t custom_qos,
  rclcpp::PublisherOptions options)
{
  node_ = node;
  typedef image_transport::SimplePublisherPlugin<sensor_msgs::msg::CompressedImage> Base;
  Base::advertiseImpl(node, base_topic, custom_qos, options);

  // Declare Parameters
  uint ns_len = node->get_effective_namespace().length();
  std::string param_base_name = base_topic.substr(ns_len);
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');

  using callbackT = std::function<void(ParameterEvent::SharedPtr event)>;
  auto callback = std::bind(&CompressedPublisher::onParameterEvent, this, std::placeholders::_1,
                            node->get_fully_qualified_name(), param_base_name);

  parameter_subscription_ = rclcpp::SyncParametersClient::on_parameter_event<callbackT>(node, callback);

  for(const ParameterDefinition &pd : kParameters)
    declareParameter(param_base_name, pd);
}

void CompressedPublisher::publish(
  const sensor_msgs::msg::Image& message,
  const PublishFn& publish_fn) const
{
  auto encoder = buildEncoderFor(message.encoding);

  bool cfg_use_cache = node_->get_parameter(parameters_[USE_CACHE]).get_value<bool>();

  if (cfg_use_cache) {
    if (encoder->encode(message, compressed_image_cache_)) {
      publish_fn(compressed_image_cache_);
    }
  } else {
    if (auto compressed = encoder->encode(message)) {
      publish_fn(*compressed);
    }
  }
}

void CompressedPublisher::declareParameter(const std::string &base_name,
                                           const ParameterDefinition &definition)
{
  //transport scoped parameter (e.g. image_raw.compressed.format)
  const std::string transport_name = getTransportName();
  const std::string param_name = base_name + "." + transport_name + "." + definition.descriptor.name;
  parameters_.push_back(param_name);

  //deprecated non-scoped parameter name (e.g. image_raw.format)
  const std::string deprecated_name = base_name + "." + definition.descriptor.name;
  deprecatedParameters_.push_back(deprecated_name);

  rclcpp::ParameterValue param_value;

  try {
    param_value = node_->declare_parameter(param_name, definition.defaultValue, definition.descriptor);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(logger_, "%s was previously declared", definition.descriptor.name.c_str());
    param_value = node_->get_parameter(param_name).get_parameter_value();
  }

  // transport scoped parameter as default, otherwise we would overwrite
  try {
    node_->declare_parameter(deprecated_name, param_value, definition.descriptor);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(logger_, "%s was previously declared", definition.descriptor.name.c_str());
  }
}

void CompressedPublisher::onParameterEvent(ParameterEvent::SharedPtr event, std::string full_name, std::string base_name)
{
  // filter out events from other nodes
  if (event->node != full_name)
    return;

  // filter out new/changed deprecated parameters
  using EventType = rclcpp::ParameterEventsFilter::EventType;

  rclcpp::ParameterEventsFilter filter(event, deprecatedParameters_, {EventType::NEW, EventType::CHANGED});

  const std::string transport = getTransportName();

  // emit warnings for deprecated parameters & sync deprecated parameter value to correct
  for (auto & it : filter.get_events())
  {
    const std::string name = it.second->name;

    size_t baseNameIndex = name.find(base_name); //name was generated from base_name, has to succeed
    size_t paramNameIndex = baseNameIndex + base_name.size();
    //e.g. `color.image_raw.` + `compressed` + `format`
    std::string recommendedName = name.substr(0, paramNameIndex + 1) + transport + name.substr(paramNameIndex);

    rclcpp::Parameter recommendedValue = node_->get_parameter(recommendedName);

    // do not emit warnings if deprecated value matches
    if(it.second->value == recommendedValue.get_value_message())
      continue;

    RCLCPP_WARN_STREAM(logger_, "parameter `" << name << "` is deprecated and ambiguous" <<
                                "; use transport qualified name `" << recommendedName << "`");

    node_->set_parameter(rclcpp::Parameter(recommendedName, it.second->value));
  }
}

std::unique_ptr<Encoder> CompressedPublisher::buildEncoderFor(const std::string& image_encoding) const
{
  // Fresh Configuration
  std::string cfg_format = node_->get_parameter(parameters_[FORMAT]).get_value<std::string>();
  
  compressionFormat encodingFormat = UNDEFINED;
  if (cfg_format == "jpeg") {
    encodingFormat = JPEG;
  } else if (cfg_format == "png") {
    encodingFormat = PNG;
  } else if (cfg_format == "tiff") {
    encodingFormat = TIFF;
  }

  switch (encodingFormat)
  {
    // JPEG Compression
    case JPEG:
    {
      int cfg_jpeg_quality = node_->get_parameter(parameters_[JPEG_QUALITY]).get_value<int64_t>();
      return std::make_unique<JpegEncoder>(image_encoding, cfg_jpeg_quality);
    }
    // PNG Compression
    case PNG:
    {
      int cfg_png_level = node_->get_parameter(parameters_[PNG_LEVEL]).get_value<int64_t>();
      return std::make_unique<PngEncoder>(image_encoding, cfg_png_level);
    }
    // TIFF Compression
    case TIFF:
    {
      std::string cfg_tiff_res_unit = node_->get_parameter(parameters_[TIFF_RESOLUTION_UNIT]).get_value<std::string>();
      int cfg_tiff_xdpi = node_->get_parameter(parameters_[TIFF_XDPI]).get_value<int64_t>();
      int cfg_tiff_ydpi = node_->get_parameter(parameters_[TIFF_YDPI]).get_value<int64_t>();
      
      int res_unit = -1;
      // See https://gitlab.com/libtiff/libtiff/-/blob/v4.3.0/libtiff/tiff.h#L282-284
      if (cfg_tiff_res_unit == "inch") {
        res_unit = 2;
      } else if (cfg_tiff_res_unit == "centimeter") {
        res_unit = 3;
      } else if (cfg_tiff_res_unit == "none") {
        res_unit = 1;
      } else {
        RCLCPP_WARN(
          logger_,
          "tiff.res_unit parameter should be either 'inch', 'centimeter' or 'none'; "
          "defaulting to 'inch'. Found '%s'", cfg_tiff_res_unit.c_str());
      }

      return std::make_unique<TiffEncoder>(image_encoding, cfg_tiff_xdpi, cfg_tiff_ydpi, res_unit);
    }

    default:
      RCUTILS_LOG_ERROR("Unknown compression type '%s', valid options are 'jpeg', 'png' and 'tiff'", cfg_format.c_str());
      break;
  }
}

} //namespace compressed_image_transport
