#pragma once

#include <any>
#include <memory>
#include <random>
#include <boost/shared_ptr.hpp>
#include <glim/mapping/global_mapping_base.hpp>

////////////////////////////
//Save to LAS PDAL library headers
#include <pdal/PointView.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/Options.hpp>
#include <pdal/StageFactory.hpp>
#include <io/BufferReader.hpp>

////////////////////////////

#include <glk/io/ply_io.hpp>



namespace gtsam {
class Values;
class NonlinearFactorGraph;
}  // namespace gtsam

namespace gtsam_points {
class ISAM2Ext;
class StreamTempBufferRoundRobin;
struct ISAM2ResultExt;
}  // namespace gtsam_points

namespace glim {

class IMUIntegration;

/**
 * @brief Global mapping parameters
 */
struct GlobalMappingParams {
public:
  GlobalMappingParams();
  ~GlobalMappingParams();

public:
  bool enable_gpu;
  bool enable_imu;
  bool enable_optimization;
  bool enable_between_factors;
  std::string between_registration_type;

  std::string registration_error_factor_type;
  double submap_voxel_resolution;
  double submap_voxel_resolution_max;
  double submap_voxel_resolution_dmin;
  double submap_voxel_resolution_dmax;
  int submap_voxelmap_levels;
  double submap_voxelmap_scaling_factor;

  double randomsampling_rate;
  double max_implicit_loop_distance;
  double min_implicit_loop_overlap;

  bool use_isam2_dogleg;
  double isam2_relinearize_skip;
  double isam2_relinearize_thresh;

  double init_pose_damping_scale;
};

/**
 * @brief Global mapping
 */
class GlobalMapping : public GlobalMappingBase {
public:
  GlobalMapping(const GlobalMappingParams& params = GlobalMappingParams());
  virtual ~GlobalMapping();

  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) override;
  virtual void insert_submap(const SubMap::Ptr& submap) override;

  virtual void find_overlapping_submaps(double min_overlap) override;
  virtual void optimize() override;

  virtual void save(const std::string& path) override;
  virtual std::vector<Eigen::Vector4d> export_points() override;
  void export_points_for_las(std::vector<Eigen::Vector4d>&, std::vector<double>&);
  //debug only
  void print_submap_structure();

  void based_on_legacy_save_ply(const std::string& path);
  void based_on_legacy_save_las(const std::string& path);

  void save_trajectory_ply(const std::string& path);
  void save_trajectory_text(const std::string& path);
  
  void another_save_ply(const std::string& path);
  std::vector<Eigen::Vector4d> another_export_points();
  void another_save_ply_extended(const std::string& path);
  bool fillPLYData(glk::PLYData& ply);

   void another_save_las(const std::string& path);
   bool fillView(pdal::PointViewPtr view);  
   bool fillViewTimesPointsLas(pdal::PointViewPtr view);  

  /**
   * @brief Load a mapping result from a dumped directory
   * @param path Input dump path
   */
  bool load(const std::string& path);

private:
  void insert_submap(int current, const SubMap::Ptr& submap);

  boost::shared_ptr<gtsam::NonlinearFactorGraph> create_between_factors(int current) const;
  boost::shared_ptr<gtsam::NonlinearFactorGraph> create_matching_cost_factors(int current) const;

  void update_submaps();
  gtsam_points::ISAM2ResultExt update_isam2(const gtsam::NonlinearFactorGraph& new_factors, const gtsam::Values& new_values);

  void recover_graph() override;
  std::pair<gtsam::NonlinearFactorGraph, gtsam::Values> recover_graph(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values) const;

private:
  using Params = GlobalMappingParams;
  Params params;

  std::mt19937 mt;

  std::unique_ptr<IMUIntegration> imu_integration;
  std::any stream_buffer_roundrobin;

  std::vector<SubMap::Ptr> submaps;
  std::vector<gtsam_points::PointCloud::ConstPtr> subsampled_submaps;

  std::unique_ptr<gtsam::Values> new_values;
  std::unique_ptr<gtsam::NonlinearFactorGraph> new_factors;

  std::unique_ptr<gtsam_points::ISAM2Ext> isam2;

  std::shared_ptr<void> tbb_task_arena;
};
}  // namespace glim