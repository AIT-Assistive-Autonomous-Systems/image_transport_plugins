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

#include "compressed_image_transport/encoders/tiff_encoder.h"

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

TiffEncoder::TiffEncoder(const std::string& image_encoding, int xdpi, int ydpi, int res_unit)
  : Encoder(rclcpp::get_logger("TiffEncoder"))
{
  format_description_ = image_encoding;
  bit_depth_ = enc::bitDepth(image_encoding);

  // Tiff specific
  params_.emplace_back(cv::IMWRITE_TIFF_XDPI);
  params_.emplace_back(xdpi);
  params_.emplace_back(cv::IMWRITE_TIFF_YDPI);
  params_.emplace_back(ydpi);
  params_.emplace_back(cv::IMWRITE_TIFF_RESUNIT);
  params_.emplace_back(res_unit);

  bool accepted_bit_depth = (bit_depth_ == 8) || (bit_depth_ == 16) || (bit_depth_ == 32);

  if (!accepted_bit_depth) {
    RCUTILS_LOG_ERROR(
        "Compressed Image Transport - TIFF compression requires 8/16/32-bit encoded color format (input format is: "
        "%s)",
        image_encoding.c_str());
  }
}

bool TiffEncoder::encode(const sensor_msgs::msg::Image& message, sensor_msgs::msg::CompressedImage& compressed) const {
  compressed.header = message.header;
  compressed.format = format_description_;

  try {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(message, nullptr, "");

    // Compress image
    if (cv::imencode(".tiff", cv_ptr->image, compressed.data, params_)) {
      float cRatio = static_cast<float>((cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.elemSize())) /
                      static_cast<float>((float)compressed.data.size());
      RCUTILS_LOG_DEBUG("Compressed Image Transport - Codec: tiff, Compression Ratio: 1:%.2f (%lu bytes)", cRatio,
                        compressed.data.size());

      return true;
    } else {
      RCUTILS_LOG_ERROR("cv::imencode (tiff) failed on input image");
    }

  } catch (cv_bridge::Exception& e) {
    RCUTILS_LOG_ERROR("%s", e.what());
  } catch (cv::Exception& e) {
    RCUTILS_LOG_ERROR("%s", e.what());
  }

  return false;
}

}  // namespace compressed_image_transport