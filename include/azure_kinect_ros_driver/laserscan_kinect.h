#pragma once

#include <vector>
#include <string>
#include <mutex>
#include <list>
#include <fstream>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <image_geometry/pinhole_camera_model.h>

#include <azure_kinect_ros_driver/math.h>

#include <cv_bridge/cv_bridge.h>

namespace laserscan_kinect {

class LaserScanKinect {
 public:
  LaserScanKinect(): scan_msg_(new sensor_msgs::LaserScan()) { }
  ~LaserScanKinect() = default;

  /**
   * @brief prepareLaserScanMsg converts depthimage and prepare new LaserScan message
   *
   * @param depth_msg Message that contains depth image which will be converted to LaserScan.
   * @param info_msg Message which contains depth sensor parameters.
   *
   * @return Return pointer to LaserScan message.
   */
  sensor_msgs::LaserScanPtr getLaserScanMsg(
    const sensor_msgs::ImagePtr& depth_msg,
    const sensor_msgs::CameraInfo& info_msg);
  /**
   * @brief setOutputFrame sets the frame to output laser scan
   * @param frame
   */
  void setOutputFrame(const std::string& frame) { output_frame_id_ = frame; }
  /**
   * @brief setMinRange sets depth sensor min range
   *
   * @param rmin Minimum sensor range (below it is death zone) in meters.
   */

  void setMinRange(const float rmin);
  /**
   * @brief setMaxRange sets depth sensor max range
   *
   * @param rmax Maximum sensor range in meters.
   */
  void setMaxRange(const float rmax);
  /**
   * @brief setScanHeight sets height of depth image which will be used in conversion process
   *
   * @param scan_height Height of used part of depth image in pixels.
   */
  void setScanHeight(const int scan_height);
  /**
   * @brief setScanRadius sets the radius of the depth image which will be used in conversion process
   * 
   * @param scan_radius Radius of used part of depth image in pixels.
   */
   void setScanRadius(const int scan_radius);
   /**
    * @brief setApplyScanRadius sets the scan type, when applied a circular cut is added
    * 
    * @param apply
    */
   void setApplyScanRadius(const bool apply) {apply_scan_radius_ = apply;}
  /**
   * @brief setDepthImgRowStep
   *
   * @param row_step
   */
  void setDepthImgRowStep(const int row_step);
  /**
   * @brief setCamModelUpdate sets the camera parameters
   *
   * @param enable
   */
  void setCamModelUpdate(const bool enable) { cam_model_update_ = enable; }
  /**
   * @brief setSensorMountHeight sets the height of sensor mount (in meters)
   */
  void setSensorMountHeight(const float height);
  /**
   * @brief setSensorTiltAngle sets the sensor tilt angle (in degrees)
   *
   * @param angle
   */
  void setSensorTiltAngle(const float angle);
  /**
   * @brief setGroundRemove enables or disables the feature which remove ground from scan
   *
   * @param enable
   */
  void setGroundRemove(const bool enable) { ground_remove_enable_ = enable; }
  /**
   * @brief setGroundMargin sets the floor margin (in meters)
   *
   * @param margin
   */
  void setGroundMargin(const float margin);
  /**
   * @brief setOverheadRemove enables or disables the feature which remove overhead from scan
   *
   * @param enable
   */
  void setOverheadRemove(const bool enable) { overhead_remove_enable_ = enable; }
  /**
   * @brief setOverheadClearance sets the overhead clearance (in meters)
   *
   * @param margin
   */
  void setOverheadClearance(const float clearance);
  /**
   * @brief setTiltCompensation enables or disables the feature which compensates sensor tilt
   *
   * @param enable
   */
  void setTiltCompensation(const bool enable) { tilt_compensation_enable_ = enable; }
 /**
  * @brief setScanConfigurated sets the configuration status
  *
  * @param enable
  */
  void setScanConfigurated(const bool configured) { is_scan_msg_configured_ = configured; }
  /**
   * @brief setPublishDbgImgEnable
   * @param enable
   */
  void setPublishDbgImgEnable(const bool enable) { publish_dbg_image_ = enable; }
  /**
   * @brief setUseFloorDepthMap
   * @param enable
   */
  void setUseFloorDepthMap(const bool enable) { use_floor_depth_map_ = enable; }

  void setResolution(const size_t height, const size_t width) {
    resolution_.first = height;
    resolution_.second = width;
  }

  bool loadFloorDepthMap(const std::string fname);

  void setThreadsNum(unsigned threads_num);

  bool getPublishDbgImgEnable() const { return publish_dbg_image_; }

  sensor_msgs::ImagePtr getDbgImage() const;

 protected:

  /**
  * @brief calcGroundDistancesForImgRows calculate coefficients used in ground removing from scan
  *
  * @param vertical_fov
  */
  void calcGroundDistancesForImgRows(double vertical_fov);
  /**
  * @brief calcClearanceDistancesForImgRows calculate coefficients used in ground removing from scan
  *
  * @param vertical_fov
  */
  void calcClearanceDistancesForImgRows(double vertical_fov);
  /**
  * @brief calcTiltCompensationFactorsForImgRows calculate factors used in tilt compensation
  *
  * @param vertical_fov
  */
  void calcTiltCompensationFactorsForImgRows(double vertical_fov);
  /**
  * @brief calcScanMsgIndexForImgCols
  *
  * @param depth_msg
  */
  void calcScanMsgIndexForImgCols(const sensor_msgs::ImagePtr& depth_msg);
  /**
  * @brief getSmallestValueInColumn finds smallest values in depth image columns
    */
  template <typename T>
  float getSmallestValueInColumn(const sensor_msgs::ImagePtr &depth_msg, int col);
  /**
  * @brief convertDepthToPolarCoords converts depth map to 2D
  */
  template <typename T>
  void convertDepthToPolarCoords(const sensor_msgs::ImagePtr& depth_msg);

  sensor_msgs::ImagePtr prepareDbgImage(const sensor_msgs::ImagePtr& depth_msg,
    const std::list<std::pair<int, int>>& min_dist_points_indices);

private:
  // ROS parameters configurated with configuration file or dynamic_reconfigure
  std::string output_frame_id_;           ///< Output frame_id for laserscan message.
  float range_min_{0};                    ///< Stores the current minimum range to use
  float range_max_{0};                    ///< Stores the current maximum range to use
  unsigned scan_height_{0};               ///< Number of pixel rows used to scan computing
  bool apply_scan_radius_{false};
  unsigned scan_radius_{0};
  unsigned depth_img_row_step_{0};        ///< Row step in depth map processing
  bool  cam_model_update_{false};         ///< If continously calibration update required
  float sensor_mount_height_{0};          ///< Height of sensor mount from ground
  float sensor_tilt_angle_{0};            ///< Angle of sensor tilt
  bool  ground_remove_enable_{false};     ///< Determines if remove ground from output scan
  float ground_margin_{0};                ///< Margin for floor remove feature (in meters)
  bool  overhead_remove_enable_{false};   ///< Determines if remove overhead from output scan
  float clearance_{0};                    ///< Margin for overhead remove feature (in meters)
  bool  tilt_compensation_enable_{false}; ///< Determines if tilt compensation feature is on
  bool  publish_dbg_image_{false};        ///< Determines if debug image should be published
  unsigned threads_num_{1};               ///< Determines threads number used in image processing
  bool  use_floor_depth_map_{false};      ///< Determines if ground removal uses pre-currated floor depth map
  bool  loaded_floor_depth_{false};       ///< Determines if ground map was successfully loaded
  std::pair<size_t, size_t> resolution_;  ///< Expected depth image resolution

  /// Published scan message
  sensor_msgs::LaserScanPtr scan_msg_;

  /// Class for managing CameraInfo messages
  image_geometry::PinholeCameraModel cam_model_;

  /// Determines if laser scan message is configurated
  bool is_scan_msg_configured_{false};

  /// Calculated laser scan msg indexes for each depth image column
  std::vector<unsigned> scan_msg_index_;

  /// Calculated maximal distances for measurements not included as floor
  std::vector<float> dist_to_ground_corrected;

  /// Calculated maximal distances for measurements not included as overhead clearance
  std::vector<float> dist_to_clear_corrected;

  /// Calculated sensor tilt compensation factors
  std::vector<float> tilt_compensation_factor_;

  /// The vertical offset of image based on calibration data
  int image_vertical_offset_{0};

  cv::Mat floor_map_;

  sensor_msgs::ImagePtr dbg_image_;
  std::list<std::pair<int, int>> min_dist_points_indices_;

  std::mutex points_indices_mutex_;
  std::mutex scan_msg_mutex_;
};

} // namespace laserscan_kinect