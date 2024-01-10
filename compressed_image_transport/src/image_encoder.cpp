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

#include "compressed_image_transport/image_encoder.h"

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

bool ImageEncoder::encode(const sensor_msgs::msg::Image& message, sensor_msgs::msg::CompressedImage& compressed) const {
  compressed.header = message.header;
  compressed.format = format_description_;

  switch (encoding_format_) {
    case JPEG: {
      // OpenCV-ros bridge
      try {
        std::shared_ptr<CompressedPublisher> tracked_object;
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(message, tracked_object, format_);

        // Compress image
        if (cv::imencode(".jpg", cv_ptr->image, compressed.data, params_)) {
          float cRatio = (float)(cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.elemSize()) /
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

      break;
    }
    case PNG: {
      // OpenCV-ros bridge
      try {
        std::shared_ptr<CompressedPublisher> tracked_object;
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(message, tracked_object, format_);

        // Compress image
        if (cv::imencode(".png", cv_ptr->image, compressed.data, params_)) {
          float cRatio = (float)(cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.elemSize()) /
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
      break;
    }
    case TIFF: {
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
      break;
    }
    default:
      RCUTILS_LOG_ERROR("Compression not set, valid options are 'jpeg', 'png' and 'tiff'");
      break;
  }

  return false;
}

void ImageEncoder::setToJpegCompression(const std::string& encoding, int jpeg_quality) {
  format_description_ = encoding;
  bit_depth_ = enc::bitDepth(encoding);

  // Jpeg specific
  encoding_format_ = JPEG;

  params_.clear();
  params_.emplace_back(cv::IMWRITE_PNG_COMPRESSION);
  params_.emplace_back(jpeg_quality);

  format_description_ += "; jpeg compressed ";

  if ((bit_depth_ == 8) || (bit_depth_ == 16)) {
    // Target image format
    if (enc::isColor(encoding)) {
      // convert color images to BGR8 format
      format_ = "bgr8";
    } else {
      // convert gray images to mono8 format
      format_ = "mono8";
    }

    format_description_ += format_;
  } else {
    encoding_format_ = UNDEFINED;
    RCLCPP_ERROR(logger_,
                 "Compressed Image Transport - JPEG compression requires 8/16-bit color format (input format is: %s)",
                 encoding.c_str());
  }
}

void ImageEncoder::setToPngCompression(const std::string& encoding, int png_level) {
  format_description_ = encoding;
  bit_depth_ = enc::bitDepth(encoding);

  // Png specific
  encoding_format_ = PNG;

  params_.clear();
  params_.emplace_back(cv::IMWRITE_PNG_COMPRESSION);
  params_.emplace_back(png_level);

  format_description_ += "; png compressed ";

  if ((bit_depth_ == 8) || (bit_depth_ == 16)) {
    // Target image format
    if (enc::isColor(encoding)) {
      // convert color images to RGB domain
      stringstream targetFormat;
      targetFormat << "bgr" << bit_depth_;
      format_ = targetFormat.str();
      format_description_ += format_;
    }
  } else {
    encoding_format_ = UNDEFINED;
    RCUTILS_LOG_ERROR(
        "Compressed Image Transport - PNG compression requires 8/16-bit encoded color format (input format is: %s)",
        encoding.c_str());
  }
}

void ImageEncoder::setToTiffCompression(const std::string& encoding, int xdpi, int ydpi, int res_unit) {
  format_description_ = encoding;
  bit_depth_ = enc::bitDepth(encoding);

  // Tiff specific
  encoding_format_ = TIFF;

  params_.clear();
  params_.emplace_back(cv::IMWRITE_TIFF_XDPI);
  params_.emplace_back(xdpi);
  params_.emplace_back(cv::IMWRITE_TIFF_YDPI);
  params_.emplace_back(ydpi);
  params_.emplace_back(cv::IMWRITE_TIFF_RESUNIT);
  params_.emplace_back(res_unit);

  bool accepted_bit_depth = (bit_depth_ == 8) || (bit_depth_ == 16) || (bit_depth_ == 32);

  if (!accepted_bit_depth) {
    encoding_format_ = UNDEFINED;

    RCUTILS_LOG_ERROR(
        "Compressed Image Transport - TIFF compression requires 8/16/32-bit encoded color format (input format is: "
        "%s)",
        encoding.c_str());
  }
}
}  // namespace compressed_image_transport