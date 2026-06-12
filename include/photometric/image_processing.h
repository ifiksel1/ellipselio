// Copyright (c) 2024, Patrick Pfreundschuh
// https://opensource.org/license/bsd-3-clause

#ifndef COIN_LIO_IMAGE_PROCESSOR_H_
#define COIN_LIO_IMAGE_PROCESSOR_H_

#include <vector>
#include <opencv2/opencv.hpp>
#include "photometric/photometric_common.h"

namespace photometric {

class ImageProcessor {
 public:
  // All tunables are passed explicitly; no ROS NodeHandle.
  // Required (no defaults):
  //   rows, cols       — LiDAR image dimensions (from Projector::rows()/cols())
  //   min_range        — minimum valid range [m] (from FeatureManager::minRange())
  //   max_range        — maximum valid range [m] (from FeatureManager::maxRange())
  //   patch_size       — feature patch half-size in pixels (from FeatureManager::patchSize())
  //   window           — adaptive-blur window {width, height}, must have exactly 2 elements
  //   highpass         — vertical FIR coefficients for line-removal high-pass filter
  //   lowpass          — horizontal FIR coefficients for line-removal low-pass filter
  // Optional (COIN defaults preserved):
  //   intensity_scale  — scalar applied to raw intensity when reflectivity=false (default 0.25)
  //   reflectivity     — if true, skip intensity_scale multiplication (default false)
  //   remove_lines     — apply FIR line-removal (default true)
  //   brightness_filter— apply adaptive brightness normalisation (default true)
  //   blur             — apply 3x3 Gaussian blur (default true)
  //   erosion_margin   — extra margin added to patch_size for erosion kernel (default 2)
  //   masks            — rectangular regions to zero out in the mask (default empty)
  ImageProcessor(
      int rows,
      int cols,
      double min_range,
      double max_range,
      int patch_size,
      std::vector<int> window,
      std::vector<double> highpass,
      std::vector<double> lowpass,
      double intensity_scale  = 0.25,
      bool   reflectivity     = false,
      bool   remove_lines     = true,
      bool   brightness_filter = true,
      bool   blur             = true,
      int    erosion_margin   = 2,
      std::vector<cv::Rect> masks = {});

  // Run the intensity-image processing pipeline on a LidarFrame that has
  // already been populated by Projector::createImages (img_intensity, img_range,
  // img_mask set). Writes back to frame.img_intensity, img_photo_u8, img_dx,
  // img_dy, img_mask, and points_corrected[*].normal_z.
  void createImages(LidarFrame& frame);

 private:
  void removeLines(cv::Mat& img);
  void filterBrightness(cv::Mat& img);
  void createMask(const cv::Mat& img, cv::Mat& mask);

  cv::Mat low_pass_fir_;
  cv::Mat high_pass_fir_;
  cv::Mat kernel_dx_;
  cv::Mat kernel_dy_;
  cv::Mat kernel_erosion_;
  cv::Size window_size_;
  std::vector<cv::Rect> masks_;
  bool reflectivity_;
  bool remove_lines_;
  bool brightness_filter_;
  bool blur_;
  double intensity_scale_;
  double min_range_;
  double max_range_;
  int rows_;
  int cols_;
  int erosion_margin_;
};

}  // namespace photometric

#endif  // COIN_LIO_IMAGE_PROCESSOR_H_
