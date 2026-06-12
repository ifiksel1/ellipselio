// Copyright (c) 2024, Patrick Pfreundschuh
// https://opensource.org/license/bsd-3-clause

#include "photometric/image_processing.h"

#include <stdexcept>
#include <utility>  // std::move

namespace photometric {

ImageProcessor::ImageProcessor(
    int rows,
    int cols,
    double min_range,
    double max_range,
    int patch_size,
    std::vector<int> window,
    std::vector<double> highpass,
    std::vector<double> lowpass,
    double intensity_scale,
    bool   reflectivity,
    bool   remove_lines,
    bool   brightness_filter,
    bool   blur,
    int    erosion_margin,
    std::vector<cv::Rect> masks)
    : reflectivity_(reflectivity),
      remove_lines_(remove_lines),
      brightness_filter_(brightness_filter),
      blur_(blur),
      intensity_scale_(intensity_scale),
      min_range_(min_range),
      max_range_(max_range),
      rows_(rows),
      cols_(cols),
      erosion_margin_(erosion_margin),
      masks_(std::move(masks))
{
    if (window.size() != 2) {
        throw std::runtime_error("ImageProcessor: invalid window size, must have exactly 2 elements");
    }
    window_size_ = cv::Size(window[0], window[1]);

    high_pass_fir_ = cv::Mat(highpass).clone();
    low_pass_fir_  = cv::Mat(lowpass).clone();

    kernel_dx_ = cv::Mat::zeros(1, 3, CV_32F);
    kernel_dy_ = cv::Mat::zeros(3, 1, CV_32F);
    kernel_dx_.at<float>(0, 0) = -0.5f;
    kernel_dx_.at<float>(0, 2) =  0.5f;
    kernel_dy_.at<float>(0, 0) = -0.5f;
    kernel_dy_.at<float>(2, 0) =  0.5f;

    int k_size = patch_size + erosion_margin_;
    kernel_erosion_ = cv::Mat::ones(k_size, k_size, CV_32FC1);
}

void ImageProcessor::createImages(LidarFrame& frame) {
    // Projector::createImages must have been called by the caller before this.
    // frame.img_intensity / img_range / img_mask are assumed populated.

    if (!reflectivity_) {
        frame.img_intensity *= intensity_scale_;
    }

    if (remove_lines_) {
        removeLines(frame.img_intensity);
    }

    if (brightness_filter_) {
        filterBrightness(frame.img_intensity);
    }

    if (blur_) {
        cv::Mat img_blur;
        cv::GaussianBlur(frame.img_intensity, img_blur, cv::Size(3, 3), 0);
        frame.img_intensity = img_blur;
    }

    cv::threshold(frame.img_intensity, frame.img_intensity, 255., 255., cv::THRESH_TRUNC);

    // Convert to 8-bit for visualisation
    frame.img_intensity.convertTo(frame.img_photo_u8, CV_8UC1, 1);

    // Calculate gradient images
    cv::filter2D(frame.img_intensity, frame.img_dx, CV_32F, kernel_dx_);
    cv::filter2D(frame.img_intensity, frame.img_dy, CV_32F, kernel_dy_);

    // Create mask
    createMask(frame.img_range, frame.img_mask);

    // Assign filtered intensity values to points
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int v = 0; v < rows_; v++) {
        for (int u = 0; u < cols_; u++) {
            const int idx = frame.img_idx.ptr<int>(v)[u];
            if (idx == -1) continue;
            frame.points_corrected->points[idx].normal_z = frame.img_intensity.ptr<float>(v)[u];
        }
    }
}

void ImageProcessor::removeLines(cv::Mat& img) {
    // Perform highpass vertically
    cv::Mat im_hpf;
    cv::filter2D(img, im_hpf, CV_32F, high_pass_fir_);
    // Perform lowpass horizontally
    cv::Mat im_lpf;
    cv::filter2D(im_hpf, im_lpf, CV_32F, low_pass_fir_.t());
    // Remove filtered signal from original image
    img -= im_lpf;
    img.setTo(0, img < 0);
}

void ImageProcessor::filterBrightness(cv::Mat& img) {
    // Create brightness map
    cv::Mat brightness;
    cv::blur(img, brightness, window_size_);
    brightness += 1;
    // Normalize and scale image
    cv::Mat normalized_img = (140. * img / brightness);
    img = normalized_img;
}

void ImageProcessor::createMask(const cv::Mat& range_img, cv::Mat& mask) {
    // Mask out user-defined rectangular regions (e.g. sensor connector)
    mask = cv::Mat::ones(range_img.rows, range_img.cols, CV_8UC1);
    for (auto& mask_rect : masks_) {
        mask(mask_rect) = 0;
    }

    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    // Mask out points outside of range bounds
    for (int v = 0; v < rows_; v++) {
        for (int u = 0; u < cols_; u++) {
            const float r = range_img.ptr<float>(v)[u];
            if (r < min_range_ || r > max_range_) {
                mask.ptr<uchar>(v)[u] = 0u;
            }
        }
    }
    // Get a margin around the invalid pixels. SKIP when erosion_margin_ <= 0: with a
    // sparse splat image (EllipseLIO's downsampled cloud), eroding requires solid
    // neighborhoods and wipes the mask entirely. v1 keeps the per-point mask.
    if (erosion_margin_ > 0) {
        cv::Mat img_eroded;
        cv::erode(mask, img_eroded, kernel_erosion_);
        mask = img_eroded;
    }
}

}  // namespace photometric
