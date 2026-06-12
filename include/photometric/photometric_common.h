// Photometric-fusion foundation header — shared types for the COIN-LIO LiDAR-intensity
// direct-photometric residual ported into EllipseLIO's IKFoM measurement model.
//
// This is the ROS2 replacement for the parts of COIN-LIO's common_lib.h that the
// photometric front-end (Projector / ImageProcessor / FeatureManager) and the residual
// depend on: the LidarFrame container, the Feature point representation, the bilinear
// sampler, and a few Eigen/PCL aliases. Kept self-contained so the ported modules need
// no other COIN headers.
//
// Port plan: slam-agent/ellipselio_integration/docs/coinlio_photometric_port_plan.md
#ifndef ELLIPSELIO_PHOTOMETRIC_COMMON_H_
#define ELLIPSELIO_PHOTOMETRIC_COMMON_H_

#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <std_msgs/msg/header.hpp>

namespace photometric {

// --- Eigen aliases. V3D/M3D also exist in EllipseLIO common_lib.h; identical `using`
//     redeclarations are legal, so a TU including both is fine. V2D/M4D are new. ---
using V3D = Eigen::Vector3d;
using V2D = Eigen::Vector2d;
using M3D = Eigen::Matrix3d;
using M4D = Eigen::Matrix4d;

// --- The photometric front-end keeps its OWN point cloud (raw scan intensity), distinct
//     from EllipseLIO's EllipseLioPoint. PointXYZINormal matches COIN-LIO's convention:
//       intensity = raw LiDAR intensity (pre image-pipeline)
//       normal_y  = range (m)           [filled by the image builder]
//       normal_z  = processed intensity [written back after ImageProcessor pipeline]
//     (normal_x, COIN's stagger index, is NOT used — v1 splats via Projector::projectPoint.)
using PhotoPoint = pcl::PointXYZINormal;
using PhotoCloud = pcl::PointCloud<PhotoPoint>;

// --- Per-scan container the photometric pipeline fills and the residual reads.
//     ROS2 port of COIN-LIO common_lib.h:161 (std_msgs::Header -> ::msg::Header). ---
struct LidarFrame {
  std_msgs::msg::Header header;
  cv::Mat img_intensity;            // CV_32FC1, processed float intensity (residual LHS)
  cv::Mat img_photo_u8;            // 8-bit copy for viz/feature detection
  cv::Mat img_range;               // CV_32FC1, range (m)
  cv::Mat img_idx;                 // CV_32SC1, undistorted-cloud index per pixel (-1 empty)
  cv::Mat img_mask;                // CV_8UC1, valid-pixel mask
  cv::Mat img_dx;                  // CV_32FC1, d/du intensity (for feature selection)
  cv::Mat img_dy;                  // CV_32FC1, d/dv intensity
  PhotoCloud::Ptr points_corrected;// deskewed scan in LiDAR(end) frame, with intensity
  std::vector<M4D> T_Li_Lk_vec;    // per-IMU-substep distort transforms (LiDAR end -> sub i)
  std::vector<int> vec_idx;        // point index -> T_Li_Lk_vec slot
  std::vector<int> proj_idx;       // per-pixel point-index list (DUPLICATE_POINTS stride)
};

// --- A tracked photometric feature (patch of pixels). ROS2 port of COIN feature_manager.h:23. ---
struct Feature {
  int life_time {0};
  V2D center {V2D::Zero()};
  std::vector<double> intensities;  // reference intensity per patch pixel (first-detection frame)
  std::vector<V3D> p;               // global-frame 3D position per patch pixel
  std::vector<V2D> uv;              // pixel coords in last frame
};

// --- Bilinear image sampler (ROS2 port of COIN common_lib.h:286; was templated on T). ---
template <typename T>
inline double getSubPixelValue(const cv::Mat& img, double x, double y) {
  if (x < 0) x = 0;
  if (y < 0) y = 0;
  if (x >= img.cols) x = img.cols - 1;
  if (y >= img.rows) y = img.rows - 1;

  const int x_lower = static_cast<int>(x);
  const int x_upper = std::min(x_lower + 1, img.cols - 1);
  const int y_lower = static_cast<int>(y);
  const int y_upper = std::min(y_lower + 1, img.rows - 1);

  const double xx = x - std::floor(x);
  const double yy = y - std::floor(y);

  return (1 - xx) * (1 - yy) * img.ptr<T>(y_lower)[x_lower] +
         xx * (1 - yy) * img.ptr<T>(y_lower)[x_upper] +
         (1 - xx) * yy * img.ptr<T>(y_upper)[x_lower] +
         xx * yy * img.ptr<T>(y_upper)[x_upper];
}

}  // namespace photometric

#endif  // ELLIPSELIO_PHOTOMETRIC_COMMON_H_
