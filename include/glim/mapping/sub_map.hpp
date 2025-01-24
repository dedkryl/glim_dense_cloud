#pragma once

#include <memory>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/odometry/estimation_frame.hpp>

namespace glim {

/**
 * @brief SubMap
 *
 */
struct SubMap {
public:
  using Ptr = std::shared_ptr<SubMap>;
  using ConstPtr = std::shared_ptr<const SubMap>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Remove point clouds of the odometry estimation frames (to save memory)
   */
  void drop_frame_points();

  /// @brief Get the origin frame
  EstimationFrame::ConstPtr optim_odom_frame() const { return optim_odom_frames[optim_odom_frames.size() / 2]; }

  /// @brief Get the origin odometry frame
  EstimationFrame::ConstPtr origin_odom_frame() const { return origin_odom_frames[optim_odom_frames.size() / 2]; }

  /**
   * @brief Get the custom data and cast it to the specified type.
   * @note  This method does not check the type of the custom data.
   * @param key  Key of the custom data
   * @return T*  Pointer to the custom data. nullptr if not found.
   */
  template <typename T>
  T* get_custom_data(const std::string& key) {
    const auto found = custom_data.find(key);
    if (found == custom_data.end()) {
      return nullptr;
    }
    return reinterpret_cast<T*>(found->second.get());
  }

  /**
   * @brief Get the custom data and cast it to the specified type.
   * @note  This method does not check the type of the custom data.
   * @param key  Key of the custom data
   * @return T*  Pointer to the custom data. nullptr if not found.
   */
  template <typename T>
  const T* get_custom_data(const std::string& key) const {
    const auto found = custom_data.find(key);
    if (found == custom_data.end()) {
      return nullptr;
    }
    return reinterpret_cast<const T*>(found->second.get());
  }

  /**
   * @brief Save the submap
   * @param path  Save path
   */
  void save(const std::string& path) const;

  /**
   * @brief  Load a submap from storage
   * @param path    Load path
   * @return SubMap::Ptr Loaded SubMap
   * @return nullptr if failed to load
   */
  static SubMap::Ptr load(const std::string& path);

public:
  int id;  ///< submap ID

  Eigen::Isometry3d T_world_origin;       ///< frame[frame.size() / 2] pose w.r.t. the world
  Eigen::Isometry3d T_origin_endpoint_L;  ///< frame.front() pose w.r.t. the origin
  Eigen::Isometry3d T_origin_endpoint_R;  ///< frame.back() pose w.r.t. the origin

  gtsam_points::PointCloud::Ptr merged_keyframe;                         ///< Merged keyframes in submap frame , deskewed
  ////////////
  std::vector<double> stamps_to_merge;
  ////////////
  std::vector<gtsam_points::GaussianVoxelMap::Ptr> voxelmaps;  ///< Multi-resolution voxelmaps

  std::vector<EstimationFrame::ConstPtr> optim_odom_frames;       ///< Optimized odometry frames, not deskewed
  std::vector<EstimationFrame::ConstPtr> origin_odom_frames;  ///< Original odometry frames, not deskewed

  std::unordered_map<std::string, std::shared_ptr<void>> custom_data;  ///< User-defined custom data
};

}  // namespace  glim
