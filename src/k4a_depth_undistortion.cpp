#include "azure_kinect_ros_driver/k4a_depth_undistortion.h"

using namespace undistort;

K4ADepthUndistortion::K4ADepthUndistortion(const k4a_calibration_t *calibration,
                                           const k4a_calibration_type_t camera,
                                           k4a_device_configuration_t config,
                                           interpolation_t interpolation_type) :
                                               calibration_(calibration),
                                               camera_(camera),
                                               config_(config),
                                               interpolation_type_(interpolation_type) {}

K4ADepthUndistortion::~K4ADepthUndistortion() {}

void K4ADepthUndistortion::Undistort(const k4a_image_t& depth,
                                     k4a_image_t& undistort) const
{
  _pinhole_t pinhole = create_pinhole_from_xy_range();

  k4a_image_t lut = NULL;
  k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                    pinhole.width,
                    pinhole.height,
                    pinhole.width * (int)sizeof(coordinate_t),
                    &lut);
  create_undistortion_lut(&pinhole, lut, interpolation_type_);

  k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                   pinhole.width,
                   pinhole.height,
                   pinhole.width * (int)sizeof(uint16_t),
                   &undistort);
  remap(depth, lut, undistort, interpolation_type_);
}

// Compute a conservative bounding box on the unit plane in which all the points have valid projections
void K4ADepthUndistortion::compute_xy_range(const int width,
                                            const int height,
                                            float &x_min,
                                            float &x_max,
                                            float &y_min,
                                            float &y_max) const
{
  // Step outward from the centre point until we find the bounds of valid projection
  const float step_u = 0.25f;
  const float step_v = 0.25f;
  const float min_u = 0;
  const float min_v = 0;
  const float max_u = (float)width - 1;
  const float max_v = (float)height - 1;
  const float center_u = 0.5f * width;
  const float center_v = 0.5f * height;

  int valid;
  k4a_float2_t p;
  k4a_float3_t ray;

  // search x_min
  for (float uv[2] = { center_u, center_v }; uv[0] >= min_u; uv[0] -= step_u)
  {
    p.xy.x = uv[0];
    p.xy.y = uv[1];
    k4a_calibration_2d_to_3d(calibration_, &p, 1.f, camera_, camera_, &ray, &valid);

    if (!valid)
    {
      break;
    }
    x_min = ray.xyz.x;
  }

  // search x_max
  for (float uv[2] = { center_u, center_v }; uv[0] <= max_u; uv[0] += step_u)
  {
    p.xy.x = uv[0];
    p.xy.y = uv[1];
    k4a_calibration_2d_to_3d(calibration_, &p, 1.f, camera_, camera_, &ray, &valid);

    if (!valid)
    {
        break;
    }
    x_max = ray.xyz.x;
  }

  // search y_min
  for (float uv[2] = { center_u, center_v }; uv[1] >= min_v; uv[1] -= step_v)
  {
    p.xy.x = uv[0];
    p.xy.y = uv[1];
    k4a_calibration_2d_to_3d(calibration_, &p, 1.f, camera_, camera_, &ray, &valid);

    if (!valid)
    {
        break;
    }
    y_min = ray.xyz.y;
  }

  // search y_max
  for (float uv[2] = { center_u, center_v }; uv[1] <= max_v; uv[1] += step_v)
  {
    p.xy.x = uv[0];
    p.xy.y = uv[1];
    k4a_calibration_2d_to_3d(calibration_, &p, 1.f, camera_, camera_, &ray, &valid);

    if (!valid)
    {
        break;
    }
    y_max = ray.xyz.y;
  }
}

pinhole_t K4ADepthUndistortion::create_pinhole_from_xy_range() const
{
  int width = calibration_->depth_camera_calibration.resolution_width;
  int height = calibration_->depth_camera_calibration.resolution_height;
  if (camera_ == K4A_CALIBRATION_TYPE_COLOR)
  {
    width = calibration_->color_camera_calibration.resolution_width;
    height = calibration_->color_camera_calibration.resolution_height;
  }

  float x_min = 0, x_max = 0, y_min = 0, y_max = 0;
  compute_xy_range(width, height, x_min, x_max, y_min, y_max);

  pinhole_t pinhole;

  float fx = 1.f / (x_max - x_min);
  float fy = 1.f / (y_max - y_min);
  float px = -x_min * fx;
  float py = -y_min * fy;

  pinhole.fx = fx * width;
  pinhole.fy = fy * height;
  pinhole.px = px * width;
  pinhole.py = py * height;
  pinhole.width = width;
  pinhole.height = height;

  return pinhole;
}

void K4ADepthUndistortion::create_undistortion_lut(const pinhole_t *pinhole,
                                                   k4a_image_t lut,
                                                   interpolation_t type) const
{
  coordinate_t *lut_data = (coordinate_t *)(void *)k4a_image_get_buffer(lut);

  k4a_float3_t ray;
  ray.xyz.z = 1.f;

  int src_width = calibration_->depth_camera_calibration.resolution_width;
  int src_height = calibration_->depth_camera_calibration.resolution_height;
  if (camera_ == K4A_CALIBRATION_TYPE_COLOR)
  {
    src_width = calibration_->color_camera_calibration.resolution_width;
    src_height = calibration_->color_camera_calibration.resolution_height;
  }

  for (int y = 0, idx = 0; y < pinhole->height; y++)
  {
    ray.xyz.y = ((float)y - pinhole->py) / pinhole->fy;

    for (int x = 0; x < pinhole->width; x++, idx++)
    {
      ray.xyz.x = ((float)x - pinhole->px) / pinhole->fx;

      k4a_float2_t distorted;
      int valid;
      k4a_calibration_3d_to_2d(calibration_, &ray, camera_, camera_, &distorted, &valid);

      coordinate_t src;
      if (type == INTERPOLATION_NEARESTNEIGHBOR)
      {
        // Remapping via nearest neighbor interpolation
        src.x = (int)floorf(distorted.xy.x + 0.5f);
        src.y = (int)floorf(distorted.xy.y + 0.5f);
      }
      else if (type == INTERPOLATION_BILINEAR || type == INTERPOLATION_BILINEAR_DEPTH)
      {
        // Remapping via bilinear interpolation
        src.x = (int)floorf(distorted.xy.x);
        src.y = (int)floorf(distorted.xy.y);
      }
      else
      {
        printf("Unexpected interpolation type!\n");
        exit(-1);
      }

      if (valid && src.x >= 0 && src.x < src_width && src.y >= 0 && src.y < src_height)
      {
        lut_data[idx] = src;

        if (type == INTERPOLATION_BILINEAR || type == INTERPOLATION_BILINEAR_DEPTH)
        {
          // Compute the floating point weights, using the distance from projected point src to the
          // image coordinate of the upper left neighbor
          float w_x = distorted.xy.x - src.x;
          float w_y = distorted.xy.y - src.y;
          float w0 = (1.f - w_x) * (1.f - w_y);
          float w1 = w_x * (1.f - w_y);
          float w2 = (1.f - w_x) * w_y;
          float w3 = w_x * w_y;

          // Fill into lut
          lut_data[idx].weight[0] = w0;
          lut_data[idx].weight[1] = w1;
          lut_data[idx].weight[2] = w2;
          lut_data[idx].weight[3] = w3;
        }
      }
      else
      {
        lut_data[idx].x = INVALID;
        lut_data[idx].y = INVALID;
      }
    }
  }
}

void K4ADepthUndistortion::remap(const k4a_image_t src,
                                 const k4a_image_t lut,
                                 k4a_image_t dst,
                                 interpolation_t type) const
{
    int src_width = k4a_image_get_width_pixels(src);
    int dst_width = k4a_image_get_width_pixels(dst);
    int dst_height = k4a_image_get_height_pixels(dst);

    uint16_t *src_data = (uint16_t *)(void *)k4a_image_get_buffer(src);
    uint16_t *dst_data = (uint16_t *)(void *)k4a_image_get_buffer(dst);
    coordinate_t *lut_data = (coordinate_t *)(void *)k4a_image_get_buffer(lut);

    memset(dst_data, 0, (size_t)dst_width * (size_t)dst_height * sizeof(uint16_t));

    for (int i = 0; i < dst_width * dst_height; i++)
    {
        if (lut_data[i].x != INVALID && lut_data[i].y != INVALID)
        {
            if (type == INTERPOLATION_NEARESTNEIGHBOR)
            {
                dst_data[i] = src_data[lut_data[i].y * src_width + lut_data[i].x];
            }
            else if (type == INTERPOLATION_BILINEAR || type == INTERPOLATION_BILINEAR_DEPTH)
            {
                const uint16_t neighbors[4]{ src_data[lut_data[i].y * src_width + lut_data[i].x],
                                             src_data[lut_data[i].y * src_width + lut_data[i].x + 1],
                                             src_data[(lut_data[i].y + 1) * src_width + lut_data[i].x],
                                             src_data[(lut_data[i].y + 1) * src_width + lut_data[i].x + 1] };

                if (type == INTERPOLATION_BILINEAR_DEPTH)
                {
                    // If the image contains invalid data, e.g. depth image contains value 0, ignore the bilinear
                    // interpolation for current target pixel if one of the neighbors contains invalid data to avoid
                    // introduce noise on the edge. If the image is color or ir images, user should use
                    // INTERPOLATION_BILINEAR
                    if (neighbors[0] == 0 || neighbors[1] == 0 || neighbors[2] == 0 || neighbors[3] == 0)
                    {
                        continue;
                    }

                    // Ignore interpolation at large depth discontinuity without disrupting slanted surface
                    // Skip interpolation threshold is estimated based on the following logic:
                    // - angle between two pixels is: theta = 0.234375 degree (120 degree / 512) in binning resolution
                    // mode
                    // - distance between two pixels at same depth approximately is: A ~= sin(theta) * depth
                    // - distance between two pixels at highly slanted surface (e.g. alpha = 85 degree) is: B = A /
                    // cos(alpha)
                    // - skip_interpolation_ratio ~= sin(theta) / cos(alpha)
                    // We use B as the threshold that to skip interpolation if the depth difference in the triangle is
                    // larger than B. This is a conservative threshold to estimate largest distance on a highly slanted
                    // surface at given depth, in reality, given distortion, distance, resolution difference, B can be
                    // smaller
                    const float skip_interpolation_ratio = 0.04693441759f;
                    float depth_min = std::min(std::min(neighbors[0], neighbors[1]),
                                               std::min(neighbors[2], neighbors[3]));
                    float depth_max = std::max(std::max(neighbors[0], neighbors[1]),
                                               std::max(neighbors[2], neighbors[3]));
                    float depth_delta = depth_max - depth_min;
                    float skip_interpolation_threshold = skip_interpolation_ratio * depth_min;
                    if (depth_delta > skip_interpolation_threshold)
                    {
                        continue;
                    }
                }

                dst_data[i] = (uint16_t)(neighbors[0] * lut_data[i].weight[0] + neighbors[1] * lut_data[i].weight[1] +
                                         neighbors[2] * lut_data[i].weight[2] + neighbors[3] * lut_data[i].weight[3] +
                                         0.5f);
            }
            else
            {
                printf("Unexpected interpolation type!\n");
                exit(-1);
            }
        }
    }
}