// Copyright (c) 2024, Patrick Pfreundschuh
// https://opensource.org/license/bsd-3-clause

#include "photometric/projector.h"
#include <stdexcept>

#define DUPLICATE_POINTS 10

namespace photometric {

Projector::Projector(int rows, int cols, double beam_offset_mm,
                     const std::vector<double>& beam_altitude_angles_deg) {
    rows_ = static_cast<size_t>(rows);
    cols_ = static_cast<size_t>(cols);
    beam_offset_m_ = beam_offset_mm;
    elevation_angles_ = beam_altitude_angles_deg;

    for (auto& angle : elevation_angles_) {
        angle *= M_PI/180.;
    }

    double fy = - static_cast<double>(rows_) / fabs(elevation_angles_[0] -
        elevation_angles_[elevation_angles_.size() - 1]);
    double fx = - static_cast<double>(cols_) / (2 * M_PI);
    double cy = rows_ / 2;
    double cx = cols_ / 2;

    K_ << fx, 0, cx,
          0, fy, cy,
          0, 0, 1;

    // mm to m
    beam_offset_m_ *= 1e-3;
}

size_t Projector::vectorIndexFromRowCol(const size_t row, const size_t col) const {
    return (row * cols_ + col) * DUPLICATE_POINTS;
}

void Projector::createImages(LidarFrame& frame) const {
    frame.img_intensity = cv::Mat::zeros(rows_, cols_, CV_32FC1);
    frame.img_range = cv::Mat::zeros(rows_, cols_, CV_32FC1);
    frame.img_idx = cv::Mat::ones(rows_, cols_, CV_32SC1) * (-1);
    frame.img_mask = cv::Mat::zeros(rows_, cols_, CV_8UC1);
    frame.proj_idx = std::vector<int>(rows_*cols_*DUPLICATE_POINTS, 0);

    // Create a projected image from the undistorted point cloud by splatting each point
    // through projectPoint (spherical projection with beam-offset model).
    #ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for
    #endif
    for (size_t j = 0; j < frame.points_corrected->points.size(); ++j) {
        const V3D p_Lk = frame.points_corrected->points[j].getVector3fMap().cast<double>();
        V2D px;
        if (!projectPoint(p_Lk, px)) continue;
        int u = std::round(px.x());
        int v = std::round(px.y());
        if (u < 0 || u >= static_cast<int>(cols_) || v < 0 || v >= static_cast<int>(rows_)) continue;

        double range = p_Lk.norm();
        frame.points_corrected->points[j].normal_y = static_cast<float>(range);
        frame.img_range.ptr<float>(v)[u] = static_cast<float>(range);
        frame.img_intensity.ptr<float>(v)[u] = frame.points_corrected->points[j].intensity;
        frame.img_idx.ptr<int>(v)[u] = static_cast<int>(j);
        frame.img_mask.ptr<uint8_t>(v)[u] = 255;

        size_t start_idx = vectorIndexFromRowCol(v, u);
        size_t offset = frame.proj_idx[start_idx] + 1;
        if (offset >= DUPLICATE_POINTS) continue;
        size_t idx = start_idx + offset;
        frame.proj_idx[idx] = j;
        frame.proj_idx[start_idx] = offset;
    }
}

bool Projector::projectPoint(const V3D& point, V2D& uv) const {
    // Spherical projection model that accounts for the beam offset, as described in the paper figure 2
    const double L = sqrt(point.x()*point.x() + point.y()*point.y()) - beam_offset_m_;
    const double R = sqrt(point.z()*point.z() + L*L);
    const double phi = atan2(point.y(), point.x());
    const double theta = asin(point.z()/R);
    uv.x() = K_(0, 0) * phi + K_(0, 2);

    // Instead of directly using theta, we use the lookup table to find the corresponding row and interpolate
    // for subpixel accuracy

    if (theta > elevation_angles_[0]) {
        uv.y() = 0;
        return false;
    } else if (theta < elevation_angles_[rows_ - 1]) {
        uv.y() = rows_ - 1;
        return false;
    }

    // Angle above
    auto greater = (std::upper_bound(elevation_angles_.rbegin(), elevation_angles_.rend(), theta) + 1).base();
    // Angle below
    auto smaller = greater + 1;
    if (greater == elevation_angles_.end()) {
        uv.y() = rows_ - 1;
    } else {
        // Interpolate pixel
        uv.y() = std::distance(elevation_angles_.begin(), greater);
        uv.y() += (*greater - theta) / (*greater - *smaller);
    }

    return isFOV(uv);
}

bool Projector::isFOV(const V2D& uv) const {
    return (uv.x() >= 0 && uv.x() <= cols_ - 1 && uv.y() >= 0 && uv.y() <= rows_ - 1);
}

void Projector::projectionJacobian(const V3D& p, Eigen::MatrixXd& du_dp) const {
    // Calculate projection jacobian with respect to 3D point position, as expressed in formula 8 in paper
    double rxy = p.head<2>().norm();
    double L = rxy - beam_offset_m_;
    double R2 = L*L + p.z()*p.z();
    double irxy = 1./rxy;
    double irxy2 = irxy * irxy;
    double fx_irxy2 = K_(0,0) * irxy2;

    du_dp = Eigen::MatrixXd::Zero(2,3);
    du_dp << -fx_irxy2 * p.y(), fx_irxy2 * p.x(), 0,
        -K_(1,1)*p.x()*p.z()/(L*R2), -K_(1,1)*p.y()*p.z()/(L*R2), K_(1,1)*L/R2;
}

bool Projector::projectUndistortedPoint(const LidarFrame& frame, const V3D& p_L_k, V3D& p_L_i, V2D& uv,
    int& distortion_idx, bool round) const {
    // Project to image
    V2D uv_k;
    if (!projectPoint(p_L_k, uv_k)) {
        return false;
    }

    if (round) {
        uv_k(0) = std::round(uv_k(0));
        uv_k(1) = std::round(uv_k(1));
    }

    distortion_idx = -1;
    int row = uv_k(1);
    int col = uv_k(0);

    int idx = vectorIndexFromRowCol(row, col);
    // Look up the index of the undistortion transformation that belongs to this pixel
    if (frame.proj_idx[idx] == 0) {
        row = 0;
        while (row < frame.img_intensity.rows) {
            idx = vectorIndexFromRowCol(row, col);
            if (frame.proj_idx[idx] > 0) {
                break;
            } else {
                ++row;
            }
        }
    }

    if (row >= frame.img_intensity.rows) {
        return false;
    }

    // If multiple points project to pixel in the undistortion map, we select the closest one to the feature point
    if(frame.proj_idx[idx] > 1) {
        float min_dist = std::numeric_limits<float>::max();
        for (int i = 1; i <= frame.proj_idx[idx]; i++) {
            const int j = frame.proj_idx[idx + i];
            const V3D p_cand = frame.points_corrected->points[j].getVector3fMap().cast<double>();
            const V3D diff = p_L_k - p_cand;
            const float dist = diff.norm();
            if (dist < min_dist) {
                min_dist = dist;
                distortion_idx = j;
            }
        }
    } else {
        distortion_idx = frame.proj_idx[idx + 1];
    }

    if (distortion_idx < 0) {
        return false;
    }

    // Lookup the inverse (T_Li_Lk) of the undistortion transformation (T_Lk_Li)
    const M4D& T_Li_Lk = frame.T_Li_Lk_vec[frame.vec_idx[distortion_idx]];

    // Express what the feature point would have been expressed in the lidar frame at the time of the
    // respective point, basically "distort" the point to distorted lidar frame
    p_L_i = T_Li_Lk.block<3,3>(0,0) * p_L_k + T_Li_Lk.block<3,1>(0,3);

    // Now we can project this point to the actual dense image (which is recorded in the distorted frame)
    if (!projectPoint(p_L_i, uv)) {
        return false;
    }

    return true;
}

}  // namespace photometric
