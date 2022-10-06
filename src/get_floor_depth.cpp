#include <k4a/k4a.h>
#include <k4a/k4a.hpp>
#include <k4a/k4atypes.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include<unistd.h>

bool GetK4ADevice(k4a::device& device_) {
  uint32_t k4a_device_count = k4a::device::get_installed_count();
  std::cout << "Found " << k4a_device_count << " sensors\n";
  if (k4a_device_count < 1) {
    return false;
  }

  k4a::device device;
  try
  {
    device = k4a::device::open(0);
  }
  catch (...)
  {
    std::cout << "Failed to open K4A device at index 0\n";
    return false;
  }

  device_ = std::move(device);
  return true;
}

int main(int argc, char** argv)
{
  float sensor_height = 900.f; // millimeters
  size_t plane_width = 2018;
  k4a::device device;

  bool result = GetK4ADevice(device);
  if (!result) {
    return 1;
  }

  k4a::calibration k4a_calibration = device.get_calibration(K4A_DEPTH_MODE_WFOV_UNBINNED,
                                                            K4A_COLOR_RESOLUTION_1536P);

  k4a_calibration_type_t camera = K4A_CALIBRATION_TYPE_DEPTH;

  cv::Mat depth = cv::Mat::zeros(1024, 1024, CV_16UC1);
  cv::Mat sampling = cv::Mat::zeros(plane_width, plane_width, CV_16UC1);

  k4a_float3_t ray;
  ray.xyz.y = sensor_height;
  const size_t y = sensor_height;
  for (size_t j = 0; j < plane_width; j++) {
    ray.xyz.x = floorf(j - plane_width / 2.f);
    for (size_t i = 0; i < plane_width; i++) {
      ray.xyz.z = 1 + i;
      k4a_float2_t distorted;
      bool valid = k4a_calibration.convert_3d_to_2d(ray, camera, camera, &distorted);

      if (valid) {
        const float rsq = ray.xyz.x*ray.xyz.x + ray.xyz.y*ray.xyz.y + ray.xyz.z*ray.xyz.z;
        const float r = std::sqrt(rsq);
        std::cout << "point: " << ray.xyz.x << " " << ray.xyz.y << " " << ray.xyz.z << std::endl
                  << "distorted: " << distorted.xy.x << " " << distorted.xy.y << std::endl
                  << "depth: " << r << std::endl;
        const int col = (int)floorf(distorted.xy.x + 0.5);
        const int row = (int)floorf(distorted.xy.y + 0.5);
        if (row > 1024 || col > 1024) {
          continue;
        }
        const float depth_value = depth.at<ushort>(row, col);
        if ( depth_value < 250 && r > 250) {
          depth.at<ushort>(row, col) = (ushort)(r+0.5);
        } else {
          depth.at<ushort>(row, col) = (ushort)std::min(r, depth_value);
        }
        sampling.at<ushort>(plane_width - i - 1, j) = 255;
      }
    }
  }
  std::cout << "finished collecting samples\n";

  k4a::capture capture;
  k4a_device_configuration_t k4a_configuration;
  k4a_configuration.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
  k4a_configuration.subordinate_delay_off_master_usec = 0;
  k4a_configuration.color_resolution = K4A_COLOR_RESOLUTION_OFF;
  k4a_configuration.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
  k4a_configuration.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
  k4a_configuration.camera_fps = K4A_FRAMES_PER_SECOND_5;
  k4a_configuration.synchronized_images_only = false;

  device.start_cameras(&k4a_configuration);
  device.get_capture(&capture, std::chrono::milliseconds(K4A_WAIT_INFINITE));
  while (capture.get_depth_image() == nullptr) {
    std::cout << "no depth image captured\n";
    usleep(500);
  }
  k4a::image k4a_depth_frame = capture.get_depth_image();
  cv::Mat cv_depth_frame(k4a_depth_frame.get_height_pixels(), k4a_depth_frame.get_width_pixels(), CV_16UC1,
                         k4a_depth_frame.get_buffer(), (size_t)k4a_depth_frame.get_stride_bytes());

  std::cout << "captured depth frame size: " << cv_depth_frame.cols << " X " << cv_depth_frame.rows << std::endl;

  std::ofstream myfile;
  myfile.open("expected_floor_depth.csv");
  myfile << cv::format(depth, cv::Formatter::FMT_CSV) << std::endl;
  myfile.close();
  std::cout << "saved expected depth map\n";

  myfile.open("observed_depth.csv");
  myfile << cv::format(cv_depth_frame, cv::Formatter::FMT_CSV) << std::endl;
  myfile.close();
  std::cout << "saved observed depth map\n";

  cv::Mat diff = cv_depth_frame - depth;
  myfile.open("depth_diff.csv");
  myfile << cv::format(cv_depth_frame, cv::Formatter::FMT_CSV) << std::endl;
  myfile.close();
  std::cout << "saved diff depth map\n";

  // cv::Mat mask, depth_grey;
  // cv::cvtColor(depth, depth_grey, cv::COLOR_BGR2GRAY);
  // std::cout << "thresholding\n";
  // std::cout << "depth type: " << depth.type() << std::endl
  //           << "depth channels: " << depth.channels() << std::endl
  //           << "depth size: " << depth.size() << std::endl
  //           << "mask type: " << mask.type() << std::endl
  //           << "mask channels: " << mask.channels() << std::endl
  //           << "mask size: " << mask.size() << std::endl;
  // cv::threshold(depth_grey, mask, 1, 255, cv::THRESH_BINARY);
  // std::cout << "finished thresholding\n";;
  // mask.convertTo(mask, CV_8UC1);

  const double norm = cv::norm(diff, cv::NORM_L1);
  std::cout << "total difference: " << norm << std::endl;

  cv::imwrite("valid_spaces.jpg", sampling);

  return 0;
}