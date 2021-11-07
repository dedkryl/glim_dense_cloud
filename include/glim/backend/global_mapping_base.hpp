#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include <glim/backend/sub_map.hpp>

namespace glim {

/**
 * @brief Global mapping base class
 *
 */
class GlobalMappingBase {
public:
  virtual ~GlobalMappingBase() {}

  /**
   * @brief Insert an image
   * @param stamp   Timestamp
   * @param image   Image
   */
  virtual void insert_image(const double stamp, const cv::Mat& image);

  /**
   * @brief Insert an IMU frame
   * @param stamp         Timestamp
   * @param linear_acc    Linear acceleration
   * @param angular_vel   Angular velocity
   */
  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);

  /**
   * @brief Insert a SubMap
   * @param submap  SubMap
   */
  virtual void insert_submap(const SubMap::Ptr& submap);

  /**
   * @brief Request to perform optimization
   */
  virtual void optimize();

  /**
   * @brief Save the mapping result
   * @param path  Save path
   */
  virtual void save(const std::string& path) {}
};
}