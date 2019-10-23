// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/imageMasking/imageMasking.hpp"
#include "aliceVision/imageMasking/eigen2cvHelpers.hpp"

#include "aliceVision/image/Image.hpp"
#include "aliceVision/image/io.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

namespace aliceVision{
namespace imageMasking{

namespace{
  float clamp(float value)
  {
    return std::min(1.f, std::max(0.f, value));
  }

  uint8_t remap_float2uint8(float value)
  {
    return uint8_t(clamp(value) * 255.f + 0.49999f);
  }

  cv::Mat wrapCvMask(OutImage& result)
  {
    return cv::Mat(result.rows(), result.cols(), CV_8UC1, result.data(), result.rowStride());
  }

  void rotateHue(cv::Mat & hsv, uint8_t delta)
  {
    // cannot use cv::add because it saturates the output.
    // do not use forEach because this call is already parallelized.
    for (int r = 0; r < hsv.rows; r++)
    {
      for ( int c = 0; c < hsv.cols; c++)
      {
        auto & pixel = hsv.at<cv::Point3_<uint8_t>>(r, c);
        pixel.x += delta;
      }
    }
  }
}

void hsv(OutImage& result, const InImagePath& inputPath, float hue, float hueRange, float minSaturation, float minValue)
{
  // HSV's hue channel is an rotation angle: it wraps at 0deg/360deg.
  // Hue for blue is 240deg. With hueRange=0.1, pixels are selected if (hue > 204deg && hue < 276deg).
  // Hue for red is 0deg. With hueRange=0.1, pixels are selected if (hue < 36deg && hue > 324deg).
  // For consistency, we rotate the hue channel so the selected hue is always at 180deg.
  // Then, with hueRange=0.1, pixels are selected if (hue > 180-36deg && hue < 180+36deg).
  image::Image<image::RGBColor> input;
  image::readImage(inputPath, input, image::EImageColorSpace::SRGB);

  cv::Mat input_hsv;
  cv::eigen2cv(input.GetMat(), input_hsv);  // copy the buffer, but a copy is needed to convert to HSV colorspace anyway.
  cv::cvtColor(input_hsv, input_hsv, cv::COLOR_RGB2HSV_FULL);  // "_FULL" to encode hue in the [0, 255] range.
  rotateHue(input_hsv, uint8_t((0.5f - hue)*256.f));  // hue == 0 <=> hue == 1

  result.resize(input.Width(), input.Height(), false);  // allocate un-initialized
  const cv::Mat result_cv = wrapCvMask(result);

  const uint8_t lowH = remap_float2uint8(0.5f - hueRange);
  const uint8_t highH = remap_float2uint8(0.5f + hueRange);
  const uint8_t lowS = remap_float2uint8(minSaturation);
  const uint8_t highS = 255;
  const uint8_t lowV = remap_float2uint8(minValue);
  const uint8_t highV = 255;
  cv::inRange(input_hsv, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), result_cv);
};

void postprocess_close(OutImage& result, int iterations)
{
  const cv::Mat result_cv = wrapCvMask(result);
  const auto kernel = cv::getStructuringElement(cv::MORPH_RECT, {3, 3});
  const auto anchor = cv::Point(-1, -1);
  cv::morphologyEx(result_cv, result_cv, cv::MORPH_CLOSE, kernel, anchor, iterations);
}

}//namespace imageMasking
}//namespace aliceVision
