// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#ifndef K4A_DEPTH_UNDISTORTION_H
#define K4A_DEPTH_UNDISTORTION_H

#include <k4a/k4a.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <algorithm>

namespace undistort {

#define INVALID INT32_MIN

typedef struct _pinhole_t
{
    float px;
    float py;
    float fx;
    float fy;

    int width;
    int height;
} pinhole_t;

typedef struct _coordinate_t
{
  int x;
  int y;
  float weight[4];
} coordinate_t;

typedef enum
{
  INTERPOLATION_NEARESTNEIGHBOR, /**< Nearest neighbor interpolation */
  INTERPOLATION_BILINEAR,        /**< Bilinear interpolation */
  INTERPOLATION_BILINEAR_DEPTH   /**< Bilinear interpolation with invalidation when neighbor contain invalid
                                                  data with value 0 */
} interpolation_t;

class K4ADepthUndistortion
{
 public:
  K4ADepthUndistortion(const k4a_calibration_t* calibration,
                       const k4a_calibration_type_t camera,
                       k4a_device_configuration_t config,
                       interpolation_t interpolation_type);
  ~K4ADepthUndistortion();

  void Undistort(const k4a_image_t& depth, k4a_image_t& undistort) const;

 private:
  void compute_xy_range(const int width,
                        const int height,
                        float &x_min,
                        float &x_max,
                        float &y_min,
                        float &y_max) const;

  pinhole_t create_pinhole_from_xy_range() const;

  void create_undistortion_lut(const pinhole_t *pinhole,
                               k4a_image_t lut,
                               interpolation_t type) const;

  void remap(const k4a_image_t src,
             const k4a_image_t lut,
             k4a_image_t dst,
             interpolation_t type) const;

 private:
  const k4a_calibration_t *calibration_;
  const k4a_calibration_type_t camera_;
  const interpolation_t interpolation_type_;
  const k4a_device_configuration_t config_;
};

}

#endif
