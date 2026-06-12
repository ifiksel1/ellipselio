// Copyright (c) 2024, Patrick Pfreundschuh
// https://opensource.org/license/bsd-3-clause

#ifndef COIN_LIO_PROJECTOR_H_
#define COIN_LIO_PROJECTOR_H_

#include "photometric/photometric_common.h"

namespace photometric {

class Projector {
  public:
    Projector(int rows, int cols, double beam_offset_mm,
              const std::vector<double>& beam_altitude_angles_deg);

    void createImages(LidarFrame& frame) const;
    void projectionJacobian(const V3D& p, Eigen::MatrixXd& du_dp) const;
    bool projectUndistortedPoint(const LidarFrame& frame, const V3D& p_L_k, V3D& p_L_i, V2D& uv, int& distortion_idx,
      bool round = false) const;
    bool projectPoint(const V3D& point, V2D& uv) const;
    bool isFOV(const V2D& uv) const;
    int rows() const {return rows_;}
    int cols() const {return cols_;}

  private:
    size_t vectorIndexFromRowCol(const size_t row, const size_t col) const;
    double beam_offset_m_;
    size_t rows_;
    size_t cols_;
    std::vector<double> elevation_angles_;
    M3D K_;
};

}  // namespace photometric

#endif  // COIN_LIO_PROJECTOR_H_
