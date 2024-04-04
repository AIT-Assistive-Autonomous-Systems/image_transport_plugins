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

#include "compressed_image_transport/encoders/jpeg_encoder.h"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "compressed_image_transport/compression_common.h"

namespace enc = sensor_msgs::image_encodings;

using namespace cv;
using namespace std;

namespace compressed_image_transport {
class CompressedPublisher;

JpegEncoder::JpegEncoder(const std::string& image_encoding, int jpeg_quality)
  : Encoder(rclcpp::get_logger("JpegEncoder"))
{
  format_description_ = image_encoding;
  bit_depth_ = enc::bitDepth(image_encoding);

  // Jpeg specific
  params_.emplace_back(cv::IMWRITE_JPEG_QUALITY);
  params_.emplace_back(jpeg_quality);

  format_description_ += "; jpeg compressed ";

  if ((bit_depth_ == 8) || (bit_depth_ == 16)) {
    // Target image format
    if (enc::isColor(image_encoding)) {
      // convert color images to BGR8 format
      format_ = "bgr8";
    } else {
      // convert gray images to mono8 format
      format_ = "mono8";
    }

    format_description_ += format_;
  } else {
    RCLCPP_ERROR(logger_,
                 "Compressed Image Transport - JPEG compression requires 8/16-bit color format (input format is: %s)",
                 image_encoding.c_str());
  }
}

bool JpegEncoder::encode(const sensor_msgs::msg::Image& message, sensor_msgs::msg::CompressedImage& compressed) const {
  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(message, nullptr, format_);
  return encode(message.header, cv_ptr->image, compressed);
}

bool JpegEncoder::encode(const std_msgs::msg::Header& header, const cv::Mat& mat,
                         sensor_msgs::msg::CompressedImage& compressed) const {
  compressed.header = header;
  compressed.format = format_description_;

  // OpenCV-ros bridge
  try {
    // Compress image
    if (cv::imencode(".jpg", mat, compressed.data, params_)) {
      float cRatio = (float)(mat.rows * mat.cols * mat.elemSize()) /
                      (float)compressed.data.size();
      RCLCPP_DEBUG(logger_, "Compressed Image Transport - Codec: jpg, Compression Ratio: 1:%.2f (%lu bytes)",
                    cRatio, compressed.data.size());

      return true;
    } else {
      RCLCPP_ERROR(logger_, "cv::imencode (jpeg) failed on input image");
    }

  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(logger_, "%s", e.what());
  } catch (cv::Exception& e) {
    RCLCPP_ERROR(logger_, "%s", e.what());
  }

  return false;
}

}  // namespace compressed_image_transport