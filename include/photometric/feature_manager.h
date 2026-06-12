// Copyright (c) 2024, Patrick Pfreundschuh
// https://opensource.org/license/bsd-3-clause
//
// ROS2 port: namespace photometric, explicit-param constructor, no ros:: deps.
// struct Feature lives in photometric_common.h — not redefined here.

#ifndef ELLIPSELIO_PHOTOMETRIC_FEATURE_MANAGER_H_
#define ELLIPSELIO_PHOTOMETRIC_FEATURE_MANAGER_H_

#include <string>
#include <vector>
#include <algorithm>

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>

#include "photometric/photometric_common.h"
#include "photometric/projector.h"

namespace photometric {

class FeatureManager {
 public:
    // Explicit-parameter constructor — no ROS NodeHandle.
    // projector   : shared Projector instance (supplies rows/cols and projection ops)
    // margin      : border margin in pixels excluded from detection/tracking  [default 10]
    // min_range   : minimum LiDAR range for valid points (m)                 [default 0.3]
    // max_range   : maximum LiDAR range for valid points (m)                 [default 20.0]
    // grad_min    : minimum gradient magnitude to be considered a candidate  [default 20.0]
    // ncc_threshold    : NCC score below which a feature is dropped          [default 0.3]
    // range_threshold  : range-diff (m) above which a point is occluded      [default 0.2]
    // suppression_radius : non-maximum suppression and mask radius (px)      [default 10]
    // patch_size  : side length of the NCC patch (must be odd)               [default 5]
    // num_features: target total feature count                                [default 65]
    // max_lifetime: frames after which a feature is forcibly dropped         [default 30]
    // mode        : "comp" | "strongest" | "random"                          [default "comp"]
    // debug       : enable debug visualization (no publishing in ROS2 port)  [default false]
    FeatureManager(std::shared_ptr<Projector> projector,
                   int margin            = 10,
                   double min_range      = 0.3,
                   double max_range      = 20.0,
                   double grad_min       = 20.0,
                   double ncc_threshold  = 0.3,
                   double range_threshold = 0.2,
                   int suppression_radius = 10,
                   int patch_size        = 5,
                   int num_features      = 65,
                   int max_lifetime      = 30,
                   const std::string& mode = "comp",
                   bool debug            = false);

    // Main entry point — call once per LiDAR frame.
    // frame : current LidarFrame (images + corrected cloud)
    // V     : complementary-direction vectors for "comp" feature selection
    // T_GL  : 4×4 rigid transform, global←LiDAR(end-of-frame) in SE3
    void updateFeatures(const LidarFrame& frame, const std::vector<V3D>& V, const M4D& T_GL);

    const std::vector<Feature>& features() const { return features_; }

    const int patchSize()   const { return patch_size_; }
    const double minRange() const { return min_range_; }
    const double maxRange() const { return max_range_; }
    const int margin()      const { return marg_size_; }
    const int nRemoved()    const { return n_removed_last_; }
    const int nAdded()      const { return n_added_last_; }

 private:
    void detectFeatures(const LidarFrame& frame, const std::vector<V3D>& V, const M4D& T_GL);

    void detectFeaturesComp(const LidarFrame& frame, const std::vector<V3D>& V, const int n_features,
                            std::vector<cv::Point>& features_uv, std::vector<V3D>& features_p) const;

    void detectFeaturesStrong(const LidarFrame& frame, const int n_features,
                              std::vector<cv::Point>& features_uv, std::vector<V3D>& features_p) const;

    void detectFeaturesRandom(const LidarFrame& frame, const int n_features,
                              std::vector<cv::Point>& features_uv, std::vector<V3D>& features_p) const;

    void trackFeatures(const LidarFrame& frame, const M4D& T_GL);

    void removeFeatures(const std::vector<int>& idx);

    void updateMask();

    bool IsFov(const cv::Point2f& pt) const;

    std::vector<std::string> feature_modes = {"comp", "strongest", "random"};
    std::shared_ptr<Projector> projector_;
    std::vector<Feature> features_;
    Eigen::MatrixXi patch_idx_;
    cv::Mat img_prev_;
    cv::Mat mask_;
    cv::Mat mask_margin_;

    double min_range_;
    double max_range_;
    double thr_gradmin_;
    double ncc_threshold_;
    double range_threshold_;
    int suppression_radius_;
    int patch_size_;
    int rows_, cols_;
    int num_features_;
    int max_lifetime_;
    int marg_size_;
    int n_removed_last_;
    int n_added_last_;
    std::string mode_;
    bool debug_;
};

}  // namespace photometric

#endif  // ELLIPSELIO_PHOTOMETRIC_FEATURE_MANAGER_H_
