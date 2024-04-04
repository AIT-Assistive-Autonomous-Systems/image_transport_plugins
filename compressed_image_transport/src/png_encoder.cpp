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

#include "compressed_image_transport/encoders/png_encoder.h"

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

PngEncoder::PngEncoder(const std::string& image_encoding, int png_level)
  : Encoder(rclcpp::get_logger("PngEncoder"))
{
  format_description_ = image_encoding;
  bit_depth_ = enc::bitDepth(image_encoding);

  // Png specific
  params_.emplace_back(cv::IMWRITE_PNG_COMPRESSION);
  params_.emplace_back(png_level);

  format_description_ += "; png compressed ";

  if ((bit_depth_ == 8) || (bit_depth_ == 16)) {
    // Target image format
    if (enc::isColor(image_encoding)) {
      // convert color images to RGB domain
      stringstream targetFormat;
      targetFormat << "bgr" << bit_depth_;
      format_ = targetFormat.str();
      format_description_ += format_;
    }
  } else {
    RCUTILS_LOG_ERROR(
        "Compressed Image Transport - PNG compression requires 8/16-bit encoded color format (input format is: %s)",
        image_encoding.c_str());
  }
}

bool PngEncoder::encode(const sensor_msgs::msg::Image& message, sensor_msgs::msg::CompressedImage& compressed) const {
  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(message, nullptr, format_);
  return encode(message.header, cv_ptr->image, compressed);
}

bool PngEncoder::encode(const std_msgs::msg::Header& header, const cv::Mat& mat,
                         sensor_msgs::msg::CompressedImage& compressed) const {
  compressed.header = header;
  compressed.format = format_description_;

  // OpenCV-ros bridge
  try {
    // Compress image
    if (cv::imencode(".png", mat, compressed.data, params_)) {
      float cRatio = (float)(mat.rows * mat.cols * mat.elemSize()) /
                      (float)compressed.data.size();
      RCUTILS_LOG_DEBUG("Compressed Image Transport - Codec: png, Compression Ratio: 1:%.2f (%lu bytes)", cRatio,
                        compressed.data.size());

      return true;
    } else {
      RCUTILS_LOG_ERROR("cv::imencode (png) failed on input image");
    }
  } catch (cv_bridge::Exception& e) {
    RCUTILS_LOG_ERROR("%s", e.what());
  } catch (cv::Exception& e) {
    RCUTILS_LOG_ERROR("%s", e.what());
  }

  return false;
}

}  // namespace compressed_image_transport