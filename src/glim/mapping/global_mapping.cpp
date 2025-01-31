#include <glim/mapping/global_mapping.hpp>

#include <unordered_set>
#include <spdlog/spdlog.h>
#include <boost/filesystem.hpp>

#include <gtsam/base/serialization.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam_points/config.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/point_cloud_gpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_gpu.hpp>
#include <gtsam_points/factors/linear_damping_factor.hpp>
#include <gtsam_points/factors/rotate_vector3_factor.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>
#include <gtsam_points/optimizers/isam2_ext.hpp>
#include <gtsam_points/optimizers/isam2_ext_dummy.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/cuda/stream_temp_buffer_roundrobin.hpp>

#include <glim/util/config.hpp>
#include <glim/util/serialization.hpp>
#include <glim/common/imu_integration.hpp>
#include <glim/mapping/callbacks.hpp>


#ifdef GTSAM_USE_TBB
#include <tbb/task_arena.h>
#endif

namespace glim {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::E;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

using Callbacks = GlobalMappingCallbacks;

GlobalMappingParams::GlobalMappingParams() {
  Config config(GlobalConfig::get_config_path("config_global_mapping"));

  enable_imu = config.param<bool>("global_mapping", "enable_imu", true);
  enable_optimization = config.param<bool>("global_mapping", "enable_optimization", true);

  enable_between_factors = config.param<bool>("global_mapping", "create_between_factors", false);
  between_registration_type = config.param<std::string>("global_mapping", "between_registration_type", "GICP");
  registration_error_factor_type = config.param<std::string>("global_mapping", "registration_error_factor_type", "VGICP");
  submap_voxel_resolution = config.param<double>("global_mapping", "submap_voxel_resolution", 1.0);
  submap_voxel_resolution_max = config.param<double>("global_mapping", "submap_voxel_resolution_max", submap_voxel_resolution);
  submap_voxel_resolution_dmin = config.param<double>("global_mapping", "submap_voxel_resolution_dmin", 5.0);
  submap_voxel_resolution_dmax = config.param<double>("global_mapping", "submap_voxel_resolution_dmax", 20.0);

  submap_voxelmap_levels = config.param<int>("global_mapping", "submap_voxelmap_levels", 2);
  submap_voxelmap_scaling_factor = config.param<double>("global_mapping", "submap_voxelmap_scaling_factor", 2.0);

  randomsampling_rate = config.param<double>("global_mapping", "randomsampling_rate", 1.0);
  max_implicit_loop_distance = config.param<double>("global_mapping", "max_implicit_loop_distance", 100.0);
  min_implicit_loop_overlap = config.param<double>("global_mapping", "min_implicit_loop_overlap", 0.1);

  enable_gpu = registration_error_factor_type.find("GPU") != std::string::npos;

  use_isam2_dogleg = config.param<bool>("global_mapping", "use_isam2_dogleg", false);
  isam2_relinearize_skip = config.param<int>("global_mapping", "isam2_relinearize_skip", 1);
  isam2_relinearize_thresh = config.param<double>("global_mapping", "isam2_relinearize_thresh", 0.1);

  init_pose_damping_scale = config.param<double>("global_mapping", "init_pose_damping_scale", 1e10);
}

GlobalMappingParams::~GlobalMappingParams() {}

GlobalMapping::GlobalMapping(const GlobalMappingParams& params) : params(params) {
#ifndef GTSAM_POINTS_USE_CUDA
  if (params.enable_gpu) {
    logger->error("GPU-based factors cannot be used because GLIM is built without GPU option!!");
  }
#endif

  imu_integration.reset(new IMUIntegration);

  new_values.reset(new gtsam::Values);
  new_factors.reset(new gtsam::NonlinearFactorGraph);

  gtsam::ISAM2Params isam2_params;
  if (params.use_isam2_dogleg) {
    gtsam::ISAM2DoglegParams dogleg_params;
    isam2_params.setOptimizationParams(dogleg_params);
  }
  isam2_params.relinearizeSkip = params.isam2_relinearize_skip;
  isam2_params.setRelinearizeThreshold(params.isam2_relinearize_thresh);

  if (params.enable_optimization) {
    isam2.reset(new gtsam_points::ISAM2Ext(isam2_params));
  } else {
    isam2.reset(new gtsam_points::ISAM2ExtDummy(isam2_params));
  }

#ifdef GTSAM_POINTS_USE_CUDA
  stream_buffer_roundrobin = std::make_shared<gtsam_points::StreamTempBufferRoundRobin>(64);
#endif

#ifdef GTSAM_USE_TBB
  tbb_task_arena = std::make_shared<tbb::task_arena>(1);
#endif
}

GlobalMapping::~GlobalMapping() {}

void GlobalMapping::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Callbacks::on_insert_imu(stamp, linear_acc, angular_vel);
  if (params.enable_imu) {
    imu_integration->insert_imu(stamp, linear_acc, angular_vel);
  }
}

void GlobalMapping::insert_submap(const SubMap::Ptr& submap) {
  logger->debug("insert_submap id={} |frame|={}", submap->id, submap->merged_keyframe->size());

  const int current = submaps.size();
  const int last = current - 1;
  insert_submap(current, submap);

  gtsam::Pose3 current_T_world_submap = gtsam::Pose3::Identity();
  gtsam::Pose3 last_T_world_submap = gtsam::Pose3::Identity();

  if (current != 0) {
    if (isam2->valueExists(X(last))) {
      last_T_world_submap = isam2->calculateEstimate<gtsam::Pose3>(X(last));
    } else {
      last_T_world_submap = new_values->at<gtsam::Pose3>(X(last));
    }

    const Eigen::Isometry3d T_origin0_endpointR0 = submaps[last]->T_origin_endpoint_R;
    const Eigen::Isometry3d T_origin1_endpointL1 = submaps[current]->T_origin_endpoint_L;
    const Eigen::Isometry3d T_endpointR0_endpointL1 = submaps[last]->origin_odom_frames.back()->T_world_sensor().inverse() * submaps[current]->origin_odom_frames.front()->T_world_sensor();
    const Eigen::Isometry3d T_origin0_origin1 = T_origin0_endpointR0 * T_endpointR0_endpointL1 * T_origin1_endpointL1.inverse();

    current_T_world_submap = last_T_world_submap * gtsam::Pose3(T_origin0_origin1.matrix());
  } else {
    current_T_world_submap = gtsam::Pose3(submap->T_world_origin.matrix());
  }

  new_values->insert(X(current), current_T_world_submap);
  submap->T_world_origin = Eigen::Isometry3d(current_T_world_submap.matrix());

  Callbacks::on_insert_submap(submap);

  submap->drop_frame_points();

  if (current == 0) {
    new_factors->emplace_shared<gtsam_points::LinearDampingFactor>(X(0), 6, params.init_pose_damping_scale);
  } else {
    new_factors->add(*create_between_factors(current));
    new_factors->add(*create_matching_cost_factors(current));
  }

  if (params.enable_imu) {
    logger->debug("create IMU factor");
    if (submap->origin_odom_frames.front()->frame_id != FrameID::IMU) {
      logger->warn("odom frames are not estimated in the IMU frame while global mapping requires IMU estimation");
    }

    // Local velocities
    const gtsam::imuBias::ConstantBias imu_biasL(submap->optim_odom_frames.front()->imu_bias);
    const gtsam::imuBias::ConstantBias imu_biasR(submap->optim_odom_frames.back()->imu_bias);

    const Eigen::Vector3d v_origin_imuL = submap->T_world_origin.linear().inverse() * submap->optim_odom_frames.front()->v_world_imu;
    const Eigen::Vector3d v_origin_imuR = submap->T_world_origin.linear().inverse() * submap->optim_odom_frames.back()->v_world_imu;

    const auto prior_noise3 = gtsam::noiseModel::Isotropic::Precision(3, 1e6);
    const auto prior_noise6 = gtsam::noiseModel::Isotropic::Precision(6, 1e6);

    if (current > 0) {
      new_values->insert(E(current * 2), gtsam::Pose3((submap->T_world_origin * submap->T_origin_endpoint_L).matrix()));
      new_values->insert(V(current * 2), (submap->T_world_origin.linear() * v_origin_imuL).eval());
      new_values->insert(B(current * 2), imu_biasL);

      new_factors->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(current), E(current * 2), gtsam::Pose3(submap->T_origin_endpoint_L.matrix()), prior_noise6);
      new_factors->emplace_shared<gtsam_points::RotateVector3Factor>(X(current), V(current * 2), v_origin_imuL, prior_noise3);
      new_factors->emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(current * 2), imu_biasL, prior_noise6);
      new_factors->emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(B(current * 2), B(current * 2 + 1), gtsam::imuBias::ConstantBias(), prior_noise6);
    }

    new_values->insert(E(current * 2 + 1), gtsam::Pose3((submap->T_world_origin * submap->T_origin_endpoint_R).matrix()));
    new_values->insert(V(current * 2 + 1), (submap->T_world_origin.linear() * v_origin_imuR).eval());
    new_values->insert(B(current * 2 + 1), imu_biasR);

    new_factors->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(current), E(current * 2 + 1), gtsam::Pose3(submap->T_origin_endpoint_R.matrix()), prior_noise6);
    new_factors->emplace_shared<gtsam_points::RotateVector3Factor>(X(current), V(current * 2 + 1), v_origin_imuR, prior_noise3);
    new_factors->emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(current * 2 + 1), imu_biasR, prior_noise6);

    if (current != 0) {
      const double stampL = submaps[last]->optim_odom_frames.back()->stamp;
      const double stampR = submaps[current]->optim_odom_frames.front()->stamp;

      int num_integrated;
      const int imu_read_cursor = imu_integration->integrate_imu(stampL, stampR, imu_biasL, &num_integrated);
      imu_integration->erase_imu_data(imu_read_cursor);

      if (num_integrated < 2) {
        logger->warn("insufficient IMU data between submaps (global_mapping)!!");
        new_factors->emplace_shared<gtsam::BetweenFactor<gtsam::Vector3>>(V(last * 2 + 1), V(current * 2), gtsam::Vector3::Zero(), gtsam::noiseModel::Isotropic::Precision(3, 1.0));
      } else {
        new_factors
          ->emplace_shared<gtsam::ImuFactor>(E(last * 2 + 1), V(last * 2 + 1), E(current * 2), V(current * 2), B(last * 2 + 1), imu_integration->integrated_measurements());
      }
    }
  }

  Callbacks::on_smoother_update(*isam2, *new_factors, *new_values);
  auto result = update_isam2(*new_factors, *new_values);
  Callbacks::on_smoother_update_result(*isam2, result);

  new_values.reset(new gtsam::Values);
  new_factors.reset(new gtsam::NonlinearFactorGraph);

  update_submaps();
  Callbacks::on_update_submaps(submaps);
}

void GlobalMapping::insert_submap(int current, const SubMap::Ptr& submap) {
  submap->voxelmaps.clear();

  // Adaptively determine the voxel resolution based on the median distance
  const int max_scan_count = 256;
  const double dist_median = gtsam_points::median_distance(submap->merged_keyframe, max_scan_count);
  const double p = std::max(0.0, std::min(1.0, (dist_median - params.submap_voxel_resolution_dmin) / (params.submap_voxel_resolution_dmax - params.submap_voxel_resolution_dmin)));
  const double base_resolution = params.submap_voxel_resolution + p * (params.submap_voxel_resolution_max - params.submap_voxel_resolution);

  // Create frame and voxelmaps
  gtsam_points::PointCloud::ConstPtr subsampled_submap;
  if (params.randomsampling_rate > 0.99) {
    subsampled_submap = submap->merged_keyframe;
  } else {
    subsampled_submap = gtsam_points::random_sampling(submap->merged_keyframe, params.randomsampling_rate, mt);
  }

#ifdef GTSAM_POINTS_USE_CUDA
  if (params.enable_gpu && !submap->merged_keyframe->points_gpu) {
    submap->merged_keyframe = gtsam_points::PointCloudGPU::clone(*submap->merged_keyframe);
  }

  if (params.enable_gpu) {
    if (params.randomsampling_rate > 0.99) {
      subsampled_submap = submap->merged_keyframe;
    } else {
      subsampled_submap = gtsam_points::PointCloudGPU::clone(*subsampled_submap);
    }

    for (int i = 0; i < params.submap_voxelmap_levels; i++) {
      const double resolution = base_resolution * std::pow(params.submap_voxelmap_scaling_factor, i);
      auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(resolution);
      voxelmap->insert(*submap->merged_keyframe);
      submap->voxelmaps.push_back(voxelmap);
    }
  }
#endif

  if (submap->voxelmaps.empty()) {
    for (int i = 0; i < params.submap_voxelmap_levels; i++) {
      const double resolution = base_resolution * std::pow(params.submap_voxelmap_scaling_factor, i);
      auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(resolution);
      voxelmap->insert(*subsampled_submap);
      submap->voxelmaps.push_back(voxelmap);
    }
  }

  submaps.push_back(submap);
  subsampled_submaps.push_back(subsampled_submap);
}

void GlobalMapping::find_overlapping_submaps(double min_overlap) {
  if (submaps.empty()) {
    return;
  }

  // Between factors are Vector2i actually. A bad use of Vector3i
  std::unordered_set<Eigen::Vector3i, gtsam_points::Vector3iHash> existing_factors;
  for (const auto& factor : isam2->getFactorsUnsafe()) {
    if (factor->keys().size() != 2) {
      continue;
    }

    gtsam::Symbol sym1(factor->keys()[0]);
    gtsam::Symbol sym2(factor->keys()[1]);
    if (sym1.chr() != 'x' || sym2.chr() != 'x') {
      continue;
    }

    existing_factors.emplace(sym1.index(), sym2.index(), 0);
  }

  gtsam::NonlinearFactorGraph new_factors;

  for (int i = 0; i < submaps.size(); i++) {
    for (int j = i + 1; j < submaps.size(); j++) {
      if (existing_factors.count(Eigen::Vector3i(i, j, 0))) {
        continue;
      }

      const Eigen::Isometry3d delta = submaps[i]->T_world_origin.inverse() * submaps[j]->T_world_origin;
      const double dist = delta.translation().norm();
      if (dist > params.max_implicit_loop_distance) {
        continue;
      }

      const double overlap = gtsam_points::overlap_auto(submaps[i]->voxelmaps.back(), subsampled_submaps[j], delta);
      if (overlap < min_overlap) {
        continue;
      }

      if (false) {
      }
#ifdef GTSAM_POINTS_USE_CUDA
      else if (std::dynamic_pointer_cast<gtsam_points::GaussianVoxelMapGPU>(submaps[i]->voxelmaps.back()) && subsampled_submaps[j]->points_gpu) {
        const auto stream_buffer = std::any_cast<std::shared_ptr<gtsam_points::StreamTempBufferRoundRobin>>(stream_buffer_roundrobin)->get_stream_buffer();
        const auto& stream = stream_buffer.first;
        const auto& buffer = stream_buffer.second;
        for (const auto& voxelmap : submaps[i]->voxelmaps) {
          new_factors.emplace_shared<gtsam_points::IntegratedVGICPFactorGPU>(X(i), X(j), voxelmap, subsampled_submaps[j], stream, buffer);
        }
      }
#endif
      else {
        for (const auto& voxelmap : submaps[i]->voxelmaps) {
          new_factors.emplace_shared<gtsam_points::IntegratedVGICPFactor>(X(i), X(j), voxelmap, subsampled_submaps[j]);
        }
      }
    }
  }

  logger->info("new overlapping {} submap pairs found", new_factors.size());

  gtsam::Values new_values;
  Callbacks::on_smoother_update(*isam2, new_factors, new_values);
  auto result = update_isam2(new_factors, new_values);
  Callbacks::on_smoother_update_result(*isam2, result);

  update_submaps();
  Callbacks::on_update_submaps(submaps);
}

void GlobalMapping::optimize() {
  if (isam2->empty()) {
    return;
  }

  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;
  Callbacks::on_smoother_update(*isam2, new_factors, new_values);
  auto result = update_isam2(new_factors, new_values);

  Callbacks::on_smoother_update_result(*isam2, result);

  update_submaps();
  Callbacks::on_update_submaps(submaps);
}

boost::shared_ptr<gtsam::NonlinearFactorGraph> GlobalMapping::create_between_factors(int current) const {
  auto factors = gtsam::make_shared<gtsam::NonlinearFactorGraph>();
  if (current == 0 || !params.enable_between_factors) {
    return factors;
  }

  const int last = current - 1;
  const gtsam::Pose3 init_delta = gtsam::Pose3((submaps[last]->T_world_origin.inverse() * submaps[current]->T_world_origin).matrix());

  if (params.between_registration_type == "NONE") {
    factors->add(gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(last), X(current), init_delta, gtsam::noiseModel::Isotropic::Precision(6, 1e6)));
    return factors;
  }

  gtsam::Values values;
  values.insert(X(0), gtsam::Pose3::Identity());
  values.insert(X(1), init_delta);

  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), gtsam::Pose3::Identity(), gtsam::noiseModel::Isotropic::Precision(6, 1e6));

  auto factor = gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(X(0), X(1), submaps[last]->merged_keyframe, submaps[current]->merged_keyframe);
  factor->set_max_correspondence_distance(0.5);
  factor->set_num_threads(2);
  graph.add(factor);

  logger->debug("--- LM optimization ---");
  gtsam_points::LevenbergMarquardtExtParams lm_params;
  lm_params.setlambdaInitial(1e-12);
  lm_params.setMaxIterations(10);
  lm_params.callback = [this](const auto& status, const auto& values) { logger->debug(status.to_string()); };

#ifdef GTSAM_USE_TBB
  auto arena = static_cast<tbb::task_arena*>(tbb_task_arena.get());
  arena->execute([&] {
#endif
    gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, values, lm_params);
    values = optimizer.optimize();

#ifdef GTSAM_USE_TBB
  });
#endif

  const gtsam::Pose3 estimated_delta = values.at<gtsam::Pose3>(X(1));
  const auto linearized = factor->linearize(values);
  const auto H = linearized->hessianBlockDiagonal()[X(1)] + 1e6 * gtsam::Matrix6::Identity();

  factors->add(gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(last), X(current), estimated_delta, gtsam::noiseModel::Gaussian::Information(H)));
  return factors;
}

boost::shared_ptr<gtsam::NonlinearFactorGraph> GlobalMapping::create_matching_cost_factors(int current) const {
  auto factors = gtsam::make_shared<gtsam::NonlinearFactorGraph>();
  if (current == 0) {
    return factors;
  }

  const auto& current_submap = submaps.back();

  double previous_overlap = 0.0;
  for (int i = 0; i < current; i++) {
    const double dist = (submaps[i]->T_world_origin.translation() - current_submap->T_world_origin.translation()).norm();
    if (dist > params.max_implicit_loop_distance) {
      continue;
    }

    const Eigen::Isometry3d delta = submaps[i]->T_world_origin.inverse() * current_submap->T_world_origin;
    const double overlap = gtsam_points::overlap_auto(submaps[i]->voxelmaps.back(), current_submap->merged_keyframe, delta);

    previous_overlap = i == current - 1 ? overlap : previous_overlap;
    if (overlap < params.min_implicit_loop_overlap) {
      continue;
    }

    if (params.registration_error_factor_type == "VGICP") {
      for (const auto& voxelmap : submaps[i]->voxelmaps) {
        factors->emplace_shared<gtsam_points::IntegratedVGICPFactor>(X(i), X(current), voxelmap, subsampled_submaps[current]);
      }
    }
#ifdef GTSAM_POINTS_USE_CUDA
    else if (params.registration_error_factor_type == "VGICP_GPU") {
      const auto stream_buffer = std::any_cast<std::shared_ptr<gtsam_points::StreamTempBufferRoundRobin>>(stream_buffer_roundrobin)->get_stream_buffer();
      const auto& stream = stream_buffer.first;
      const auto& buffer = stream_buffer.second;
      for (const auto& voxelmap : submaps[i]->voxelmaps) {
        factors->emplace_shared<gtsam_points::IntegratedVGICPFactorGPU>(X(i), X(current), voxelmap, subsampled_submaps[current], stream, buffer);
      }
    }
#endif
    else {
      logger->warn("unknown registration error type ({})", params.registration_error_factor_type);
    }
  }

  if (previous_overlap < std::max(0.25, params.min_implicit_loop_overlap)) {
    logger->warn("previous submap has only a small overlap with the current submap ({})", previous_overlap);
    logger->warn("create a between factor to prevent the submap from being isolated");
    const int last = current - 1;
    const gtsam::Pose3 init_delta = gtsam::Pose3((submaps[last]->T_world_origin.inverse() * submaps[current]->T_world_origin).matrix());
    factors->add(gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(last), X(current), init_delta, gtsam::noiseModel::Isotropic::Precision(6, 1e6)));
  }

  return factors;
}

void GlobalMapping::update_submaps() {
  for (int i = 0; i < submaps.size(); i++) {
    submaps[i]->T_world_origin = Eigen::Isometry3d(isam2->calculateEstimate<gtsam::Pose3>(X(i)).matrix());
  }
}

gtsam_points::ISAM2ResultExt GlobalMapping::update_isam2(const gtsam::NonlinearFactorGraph& new_factors, const gtsam::Values& new_values) {
  gtsam_points::ISAM2ResultExt result;

  gtsam::Key indeterminant_nearby_key = 0;
  try {
#ifdef GTSAM_USE_TBB
    auto arena = static_cast<tbb::task_arena*>(tbb_task_arena.get());
    arena->execute([&] {
#endif
      result = isam2->update(new_factors, new_values);
#ifdef GTSAM_USE_TBB
    });
#endif
  } catch (const gtsam::IndeterminantLinearSystemException& e) {
    logger->error("GlobalMapping::update_isam2 --> isam2->update Exception : an indeterminant linear system exception was caught during global map optimization!!");
    logger->error(e.what());
///////////////////////////////
    logger->error("Prevent cycle: NOT reset isam2 --> terminate process!");
    exit(1);
///////////////////////////////    
    indeterminant_nearby_key = e.nearbyVariable();
  } catch (const std::exception& e) {
    logger->error("an exception was caught during global map optimization!!");
    logger->error(e.what());
  }

  if (indeterminant_nearby_key != 0) {
    const gtsam::Symbol symbol(indeterminant_nearby_key);
    if (symbol.chr() == 'v' || symbol.chr() == 'b' || symbol.chr() == 'e') {
      indeterminant_nearby_key = X(symbol.index() / 2);
    }
    logger->warn("insert a damping factor at {} to prevent corruption", std::string(gtsam::Symbol(indeterminant_nearby_key)));

    gtsam::Values values = isam2->getLinearizationPoint();
    gtsam::NonlinearFactorGraph factors = isam2->getFactorsUnsafe();
    factors.emplace_shared<gtsam_points::LinearDampingFactor>(indeterminant_nearby_key, 6, 1e4);

    gtsam::ISAM2Params isam2_params;
    if (params.use_isam2_dogleg) {
      gtsam::ISAM2DoglegParams dogleg_params;
      isam2_params.setOptimizationParams(dogleg_params);
    }
    isam2_params.relinearizeSkip = params.isam2_relinearize_skip;
    isam2_params.setRelinearizeThreshold(params.isam2_relinearize_thresh);

    if (params.enable_optimization) {
      isam2.reset(new gtsam_points::ISAM2Ext(isam2_params));
    } else {
      isam2.reset(new gtsam_points::ISAM2ExtDummy(isam2_params));
    }

    logger->warn("reset isam2");
    return update_isam2(factors, values);
  }

  return result;
}

void GlobalMapping::save(const std::string& path) {
  optimize();

  boost::filesystem::create_directories(path);

  gtsam::NonlinearFactorGraph serializable_factors;
  std::unordered_map<std::string, gtsam::NonlinearFactor::shared_ptr> matching_cost_factors;

  for (const auto& factor : isam2->getFactorsUnsafe()) {
    bool serializable = !boost::dynamic_pointer_cast<gtsam_points::IntegratedMatchingCostFactor>(factor)
#ifdef GTSAM_POINTS_USE_CUDA
                        && !boost::dynamic_pointer_cast<gtsam_points::IntegratedVGICPFactorGPU>(factor)
#endif
      ;

    if (serializable) {
      serializable_factors.push_back(factor);
    } else {
      const gtsam::Symbol symbol0(factor->keys()[0]);
      const gtsam::Symbol symbol1(factor->keys()[1]);
      const std::string key = std::to_string(symbol0.index()) + "_" + std::to_string(symbol1.index());

      matching_cost_factors[key] = factor;
    }
  }

  logger->info("serializing factor graph to {}/graph.bin", path);
  serializeToBinaryFile(serializable_factors, path + "/graph.bin");
  serializeToBinaryFile(isam2->calculateEstimate(), path + "/values.bin");

  std::ofstream ofs(path + "/graph.txt");
  ofs << "num_submaps: " << submaps.size() << std::endl;
  ofs << "num_all_frames: " << std::accumulate(submaps.begin(), submaps.end(), 0, [](int sum, const SubMap::ConstPtr& submap) { return sum + submap->optim_odom_frames.size(); }) << std::endl;

  ofs << "num_matching_cost_factors: " << matching_cost_factors.size() << std::endl;
  for (const auto& factor : matching_cost_factors) {
    std::string type;

    if (boost::dynamic_pointer_cast<gtsam_points::IntegratedGICPFactor>(factor.second)) {
      type = "gicp";
    } else if (boost::dynamic_pointer_cast<gtsam_points::IntegratedVGICPFactor>(factor.second)) {
      type = "vgicp";
    }
#ifdef GTSAM_POINTS_USE_CUDA
    else if (boost::dynamic_pointer_cast<gtsam_points::IntegratedVGICPFactorGPU>(factor.second)) {
      type = "vgicp_gpu";
    }
#endif

    gtsam::Symbol symbol0(factor.second->keys()[0]);
    gtsam::Symbol symbol1(factor.second->keys()[1]);
    ofs << "matching_cost " << type << " " << symbol0.index() << " " << symbol1.index() << std::endl;
  }

  std::ofstream odom_lidar_ofs(path + "/odom_lidar.txt");
  std::ofstream traj_lidar_ofs(path + "/traj_lidar.txt");

  std::ofstream odom_imu_ofs(path + "/odom_imu.txt");
  std::ofstream traj_imu_ofs(path + "/traj_imu.txt");

  const auto write_tum_frame = [](std::ofstream& ofs, const double stamp, const Eigen::Isometry3d& pose) {
    const Eigen::Quaterniond quat(pose.linear());
    const Eigen::Vector3d trans(pose.translation());
    ofs << boost::format("%.9f %.6f %.6f %.6f %.6f %.6f %.6f %.6f") % stamp % trans.x() % trans.y() % trans.z() % quat.x() % quat.y() % quat.z() % quat.w() << std::endl;
  };

  for (int i = 0; i < submaps.size(); i++) {
    for (const auto& frame : submaps[i]->origin_odom_frames) {
      write_tum_frame(odom_lidar_ofs, frame->stamp, frame->T_world_lidar);
      write_tum_frame(odom_imu_ofs, frame->stamp, frame->T_world_imu);
    }

    const Eigen::Isometry3d T_world_endpoint_L = submaps[i]->T_world_origin * submaps[i]->T_origin_endpoint_L;
    const Eigen::Isometry3d T_odom_lidar0 = submaps[i]->optim_odom_frames.front()->T_world_lidar;
    const Eigen::Isometry3d T_odom_imu0 = submaps[i]->optim_odom_frames.front()->T_world_imu;

    for (const auto& frame : submaps[i]->optim_odom_frames) {
      const Eigen::Isometry3d T_world_imu = T_world_endpoint_L * T_odom_imu0.inverse() * frame->T_world_imu;
      const Eigen::Isometry3d T_world_lidar = T_world_imu * frame->T_lidar_imu.inverse();

      write_tum_frame(traj_imu_ofs, frame->stamp, T_world_imu);
      write_tum_frame(traj_lidar_ofs, frame->stamp, T_world_lidar);
    }

    submaps[i]->save((boost::format("%s/%06d") % path % i).str());
  }

  logger->info("saving config");
  GlobalConfig::instance()->dump(path + "/config");
  //if
  based_on_legacy_save_ply(path);
  based_on_legacy_save_las(path);
  //another_save_ply(path);
  //another_save_ply_thicker(path);
  //another_save_ply_extended(path);
  //another_save_las(path);

  //save_trajectory_ply(path);
  //save_trajectory_text(path); 
}

void GlobalMapping::based_on_legacy_save_ply(const std::string& path)
{
  //////////////////////////////////////////////////////////////////////
  logger->info("Original export points and save to PLY");
  auto exported_points = export_points();
  std::string ply_file_name = path + "/glim_ply.ply"; 
  logger->info(std::string("Writing to file : ") + ply_file_name);
  glk::save_ply_binary(ply_file_name, exported_points.data(), exported_points.size());
////////////////////////////////////////////////////////////////////////  
}

std::vector<Eigen::Vector4d> GlobalMapping::export_points() {
 
  int num_all_points = 0;
  for (const auto& submap : submaps) {
    num_all_points += submap->merged_keyframe->size();
  }

  std::vector<Eigen::Vector4d> all_points;
  all_points.reserve(num_all_points);

  for (const auto& submap : submaps) {
     std::transform(submap->merged_keyframe->points, submap->merged_keyframe->points + submap->merged_keyframe->size(), std::back_inserter(all_points), [&](const Eigen::Vector4d& p) {
      return submap->T_world_origin * p;
    });
  }

  return all_points;
}

void GlobalMapping::based_on_legacy_save_las(const std::string& path)
{
  //////////////////////////////////////////////////////////////////////
  logger->info("Artif export points and save to LAS");
  using namespace pdal;
  std::string las_file_name = path + "/times_points.las"; 
  Options options;
  options.add("filename", las_file_name);
  options.add("extra_dims", "all");
  options.add("minor_version", 4);

  PointTable table;
  table.layout()->registerDim(Dimension::Id::GpsTime);
  table.layout()->registerDim(Dimension::Id::X);
  table.layout()->registerDim(Dimension::Id::Y);
  table.layout()->registerDim(Dimension::Id::Z);
 
  logger->info(std::string("Writing to file : ") + las_file_name);
  PointViewPtr view(new PointView(table));
  if(!view)
  {
    logger->error("Unable to const PointView");
    return;
  }

  if(!fillViewTimesPointsLas(view))
    return;

  BufferReader reader;
  reader.addView(view);

  StageFactory factory;

  // StageFactory always "owns" stages it creates. They'll be destroyed with the factory.
  Stage *writer = factory.createStage("writers.las");
  if(!writer)
  {
        std::cout << "Unable to create writer..." << std::endl;
        return;
  }

  writer->setInput(reader);
  writer->setOptions(options);
  writer->prepare(table);
  writer->execute(table);
}

bool GlobalMapping::fillViewTimesPointsLas(pdal::PointViewPtr view)
{
  std::vector<Eigen::Vector4d> all_points;
  std::vector<double> all_times;
  export_points_for_las(all_points, all_times);

  if(all_points.size() != all_times.size())
  {
    logger->error("Error! array sizes are not equal!");
    return false;
  }
  
  try
  {
    for (size_t l = 0; l< all_points.size();l ++)
    {
      view->setField(pdal::Dimension::Id::GpsTime, l , all_times[l]);
      view->setField(pdal::Dimension::Id::X, l , all_points[l].x());
      view->setField(pdal::Dimension::Id::Y, l , all_points[l].y());
      view->setField(pdal::Dimension::Id::Z, l , all_points[l].z());
    }
 
  }
  catch(const std::exception& e)
  {
    logger->error("GlobalMapping::fillViewTimesPointsLas error: {}",e.what());
    return false;
  }
  logger->info("GlobalMapping::fillViewTimesPointsLas all_points count = {}", all_points.size());
  return true;
}


void GlobalMapping::export_points_for_las(std::vector<Eigen::Vector4d>& all_points, std::vector<double>& all_times)
{
  int num_all_points = 0;
  for (const auto& submap : submaps) {
    num_all_points += submap->merged_keyframe->size();
  }

  all_points.reserve(num_all_points);

  for (const auto& submap : submaps) {
     std::transform(submap->merged_keyframe->points, submap->merged_keyframe->points + submap->merged_keyframe->size(), std::back_inserter(all_points), [&](const Eigen::Vector4d& p) {
      return submap->T_world_origin * p;
    });
  }

  //std::cout << " print stamps_to_merge :"  << std::endl;
  //int sm_num = 0;
  std::vector<double> submap_times;
  double point_time = 0;
  for (const auto& submap : submaps) {
 
    int points_count = submap->merged_keyframe->size();
    //std::cout << " --------------------- " << sm_num++ << " points_count = " << points_count << std::endl;
    submap_times.resize(points_count);
    int stamps_count = submap->stamps_to_merge.size();
    int submap_times_cursor = 0;
    for(int i = 0; i < stamps_count; i++)
    {
      point_time = submap->stamps_to_merge.at(i);
      //std::cout 
      //<< " stamps_to_merge size = " << submap->stamps_to_merge.size() 
      //<< " points_count/stamps_count = " << points_count/stamps_count << " point_time = "
      //<< std::fixed << std::setprecision(9) << point_time << std::endl;
      for(size_t k = 0;k < points_count/stamps_count + 1;k++)
      {
        point_time+=0.00002;//step
        //std::cout << std::fixed << std::setprecision(9) << point_time << std::endl;
        if(submap_times_cursor < submap_times.size())
            submap_times[submap_times_cursor++] = point_time;
      }    
    }

    all_times.insert(all_times.end(), submap_times.begin(), submap_times.end() );
    submap_times.clear();  
  }

  //debug below - do remove
  std::cout << " all_times.size() = " << all_times.size() <<  " all_points size = " << all_points.size() << std::endl;
  if(std::find(all_times.begin(), all_times.end(), 0.0) == all_times.end())
    std::cout << " all_times does not contain 0.0 value" << std::endl;
  else
    std::cout << " Err : all_times contains 0.0 value" << std::endl;
  
  std::set<double> all_times_set;
  int l = 0;
  for (auto &&t : all_times)
  {
    if(t == 0.0)
      std::cout << l << std::endl;

    l++; 
    all_times_set.insert(t);
  }

  if(all_times_set.size() == all_times.size())
    std::cout << " all_times does not contain equal values" << std::endl;
  else
    std::cout << " all_times contains equal values" << std::endl;
  //Checks if the elements in range [first, last) are sorted in non-descending order.
  std::cout << " all_times is sorted in non-descending order from begin to end : " 
  << std::boolalpha << std::is_sorted(all_times.cbegin(), all_times.cend()) 
  << std::endl; 
  //debug above - do remove
}

void GlobalMapping::another_save_ply(const std::string& path)
{
  //////////////////////////////////////////////////////////////////////
  logger->info("Another export points and save to PLY");
  auto exported_points = another_export_points();
  std::string ply_file_name = path + "/another_ply.ply"; 
  logger->info(std::string("Writing to file : ") + ply_file_name);
  glk::save_ply_binary(ply_file_name, exported_points.data(), exported_points.size());
////////////////////////////////////////////////////////////////////////  
}


std::vector<Eigen::Vector4d> GlobalMapping::another_export_points()
{
  int num_all_points = 0;
  for (const auto& submap : submaps) {
    for (const auto& fs : submap->optim_odom_frames) {
      num_all_points += fs->frame->points->size();
    }
  }

  logger->debug("another_export_points num_all_points : {}", num_all_points);


  std::vector<Eigen::Vector4d> all_points;
  all_points.reserve(num_all_points);

  for (const auto& submap : submaps) {
    for (const auto& fs : submap->optim_odom_frames) {
      std::transform(fs->frame->points, fs->frame->points + fs->frame->size(), std::back_inserter(all_points), [&](const Eigen::Vector4d& p) {
        return submap->T_world_origin * p;
      });
    }
  }

  logger->debug("vector<Eigen::Vector4d> all_points size {}", all_points.size());
  

  return all_points;
}



void GlobalMapping::another_save_ply_extended(const std::string& path)
{
  //////////////////////////////////////////////////////////////////////
  logger->info("Another expended export points and save to PLY");
  glk::PLYData ply;
  if(!fillPLYData(ply))
    return;
  std::string ply_file_name = path + "/another_extended_ply.ply"; 
  logger->info(std::string("Writing to file : ") + ply_file_name);
  glk::save_ply_binary(ply_file_name, ply);
////////////////////////////////////////////////////////////////////////  
}

bool GlobalMapping::fillPLYData(glk::PLYData& ply)
{
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> points;
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> normals;
  int num_points = 0;
  int num_normals = 0;

  try
  {

    int num_all_points = 0;
    for (const auto& submap : submaps) {
      for (const auto& fs : submap->optim_odom_frames) {
        num_all_points += fs->frame->points->size();
      }
    }

    logger->debug("fillPLYData num_all_points : {}", num_all_points);


    int i = 0;
    for (const auto& submap : submaps) {
      for (const auto& fs : submap->optim_odom_frames) {
        //NB: timestamp not used in  PLY format
        if(fs->frame->has_points())
        {
          for(size_t k = 0; k< fs->frame->size(); k++)
          {
            Eigen::Vector4d p = *(fs->frame->points + k);
            Eigen::Vector4d pp = submap->T_world_origin*p;
            num_points++;
            points.push_back(pp);
          }
        }
        else
          logger->warn("Point must have coords");

        if(!fs->raw_frame->intensities.empty())
        {
          /*
          for(size_t k = 0; k< fs->frame->size(); k++)
          {
            double sum = std::accumulate(fs->raw_frame->intensities.begin(), fs->raw_frame->intensities.end(), 0);
            ply.intensities.push_back(sum/fs->raw_frame->intensities.size());
          }
          */
          for(size_t k = 0; k< fs->frame->size(); k++)
          {
            if(fs->raw_frame->intensities.size() == fs->frame->size())
            {
              ply.intensities.push_back(fs->raw_frame->intensities.at(k));
            }
            else
              logger->error("Smth going wrong: frame and intensities have different sizes!");
          }
        }
        else
          logger->warn("Point must have Intensity");

        if(fs->frame->has_normals())
        {
          for(size_t k = 0; k< fs->frame->size(); k++)
          {
            Eigen::Vector4d n = *(fs->frame->normals + k);
            num_normals++;
            normals.push_back(n);
          }
        }
        else
          logger->warn("Point must have Normals");
  
        i++;
        
      }
    }

    ply.vertices.resize(num_points);
    for (int i = 0; i < num_points; i++) {
      ply.vertices[i] = points[i].template head<3>().template cast<float>();
    }

    if(num_normals == num_points)
    {
      ply.normals.resize(num_points);
      for (int i = 0; i < num_points; i++) {
        ply.normals[i] = normals[i].template head<3>().template cast<float>();
      }
    }
    else
         logger->warn("GlobalMapping::fillPLYData warning : 'num_normals != num_points'");   

    if(num_points != ply.intensities.size())
    {
      logger->warn("GlobalMapping::fillPLYData warning : 'num_normals != num_points'"); 
    }

    logger->debug("fillPLYData vertices : {}",ply.vertices.size());
    logger->debug("fillPLYData normals : {}",ply.normals.size());
    logger->debug("fillPLYData intensities : {}",ply.intensities.size());

  }
  catch(const std::exception& e)
  {
    logger->error("GlobalMapping::fillPLYData error: {}",e.what());
    return false;
  }
  return true;
}


void GlobalMapping::another_save_las(const std::string& path)
{
/*
в LAS должны быть все данные
XYZ, нормали (они же угол излучения), GPS-время, интенсивность
*/
/*
PDAL:
In addition to the X, Y, Z coordinates of a point, PDAL can write many other attributes. Their full
list and types are in [Dimension.json]. Most of these are not enabled by default. To enable them, the
LAS file format minor version should be set to 4, the value of the `extra_dims` writer option should be `all`,
and the attributes should be registered with the function ``registerDim()``.

*/

  //////////////////////////////////////////////////////////////////////
  logger->info("Another points  save to LAS");
  using namespace pdal;
  std::string las_file_name = path + "/another_las.las"; 
  Options options;
  options.add("filename", las_file_name);
  options.add("extra_dims", "all");
  options.add("minor_version", 4);

  PointTable table;
  table.layout()->registerDim(Dimension::Id::GpsTime);
  table.layout()->registerDim(Dimension::Id::X);
  table.layout()->registerDim(Dimension::Id::Y);
  table.layout()->registerDim(Dimension::Id::Z);
  table.layout()->registerDim(Dimension::Id::Intensity);
  table.layout()->registerDim(Dimension::Id::NormalX);
  table.layout()->registerDim(Dimension::Id::NormalY);
  table.layout()->registerDim(Dimension::Id::NormalZ);

  logger->info(std::string("Writing to file : ") + las_file_name);
  PointViewPtr view(new PointView(table));
  if(!view)
  {
    logger->error("Unable to const PointView");
    return;
  }

  if(!fillView(view))
    return;

  BufferReader reader;
  reader.addView(view);

  StageFactory factory;

  // StageFactory always "owns" stages it creates. They'll be destroyed with the factory.
  Stage *writer = factory.createStage("writers.las");
  if(!writer)
  {
        std::cout << "Unable to create writer..." << std::endl;
        return;
  }

  writer->setInput(reader);
  writer->setOptions(options);
  writer->prepare(table);
  writer->execute(table);
////////////////////////////////////////////////////////////////////////  
}



bool GlobalMapping::fillView(pdal::PointViewPtr view)
{
  struct Point
  {
    Point(){
      time = 0, x = 0, y = 0, z = 0, intensity = 0, nx = 0, ny = 0, nz = 0;
    }
    double time;
    double x;
    double y;
    double z;
    double intensity;
    double nx;
    double ny;
    double nz;
  };

  std::vector<Point> point_vector;
  
  try
  {
    
    for (const auto& submap : submaps) {
      for (const auto& fs : submap->optim_odom_frames) {
        Point point;
        for(size_t k = 0; k< fs->frame->size(); k++)
        {
          point.time = fs->raw_frame->stamp;
          
          if(fs->frame->has_points())
          {
            Eigen::Vector4d p = *(fs->frame->points + k);
            Eigen::Vector4d pp = submap->T_world_origin*p;
            point.x = pp[0];
            point.y = pp[1];
            point.z = pp[2];
          }
          else
            logger->error("Point must have coords");
          
          if(!fs->raw_frame->intensities.empty())
          {
            if(fs->raw_frame->intensities.size() == fs->frame->size())
            {
               point.intensity = fs->raw_frame->intensities.at(k);
            }
            else
              logger->error("Smth going wrong: frame and intensities have different sizes!");
          }
          else
            logger->error("Point must have Intensity");
          
          if(fs->frame->has_normals())
          {
            Eigen::Vector4d n = *(fs->frame->normals + k);
            point.nx = n[0];
            point.ny = n[1];
            point.nz = n[2];
          }
          else
            logger->error("Point must have Normals");
                    
          point_vector.push_back(point); 
        }
      }
    }

    for (size_t l = 0; l< point_vector.size();l ++)
    {
      view->setField(pdal::Dimension::Id::GpsTime, l , point_vector[l].time);
      view->setField(pdal::Dimension::Id::X, l , point_vector[l].x);
      view->setField(pdal::Dimension::Id::Y, l , point_vector[l].y);
      view->setField(pdal::Dimension::Id::Z, l , point_vector[l].z);
      view->setField(pdal::Dimension::Id::Intensity, l , point_vector[l].intensity);
      view->setField(pdal::Dimension::Id::NormalX, l , point_vector[l].nx);
      view->setField(pdal::Dimension::Id::NormalY, l , point_vector[l].ny);
      view->setField(pdal::Dimension::Id::NormalZ, l , point_vector[l].nz);
    }
 
  }
  catch(const std::exception& e)
  {
    logger->error("GlobalMapping::fillView error: {}",e.what());
    return false;
  }
  logger->error("GlobalMapping::fillView points count = {}", point_vector.size());
  return true;
}




void GlobalMapping::save_trajectory_text(const std::string& path)
{
  /*
  //like traj_imu.txt above
  logger->info("Save trajectory to TXT");
  std::ofstream ofs(path + "/trajectory.txt");
  ofs << boost::format("Timestamp \t X \t Y \t Z") << std::endl;
  for (int i = 0; i < submaps.size(); i++) {
    for (const auto& frame : submaps[i]->origin_odom_frames) {
        const Eigen::Vector3d trans(frame->T_world_imu.translation());
        ofs << boost::format("%.9f \t %.6f \t %.6f \t %.6f") % frame->stamp % trans.x() % trans.y() % trans.z() << std::endl;
      }
   }
  */
  //like save_trajectory_ply below
  logger->info("Save trajectory to TXT");
  std::vector<Eigen::Vector3f> traj;
  std::vector<double> times;
  for (const auto& submap : submaps) {
    const Eigen::Isometry3d T_world_endpoint_L = submap->T_world_origin * submap->T_origin_endpoint_L;
    const Eigen::Isometry3d T_odom_imu0 = submap->optim_odom_frames.front()->T_world_imu;
    for (const auto& frame : submap->optim_odom_frames) {
      const Eigen::Isometry3d T_world_imu = T_world_endpoint_L * T_odom_imu0.inverse() * frame->T_world_imu;
      traj.emplace_back(T_world_imu.translation().cast<float>());
      times.emplace_back(frame->stamp);
    }
  }
  
  std::string txt_file_name = path + "/trajectory.txt"; 
  logger->info(std::string("Writing to file : ") + txt_file_name);
  std::ofstream ofs(txt_file_name);
  ofs << boost::format("Timestamp \t\t X \t\t Y \t\t Z") << std::endl;
  size_t i = 0;
  for (auto& t : traj) {
        ofs << boost::format("%.9f \t %.6f \t %.6f \t %.6f") % times.at(i) % t[0] % t[1] % t[2] << std::endl;
        i++;
   }
}

void GlobalMapping::save_trajectory_ply(const std::string& path)
{
  logger->info("Save trajectory to PLY");
  std::vector<Eigen::Vector3f> traj;
  for (const auto& submap : submaps) {
    const Eigen::Isometry3d T_world_endpoint_L = submap->T_world_origin * submap->T_origin_endpoint_L;
    const Eigen::Isometry3d T_odom_imu0 = submap->optim_odom_frames.front()->T_world_imu;
    for (const auto& frame : submap->optim_odom_frames) {
      const Eigen::Isometry3d T_world_imu = T_world_endpoint_L * T_odom_imu0.inverse() * frame->T_world_imu;
      traj.emplace_back(T_world_imu.translation().cast<float>());
    }
  }
  std::string ply_file_name = path + "/trajectory.ply"; 
  logger->info(std::string("Writing to file : ") + ply_file_name);
  glk::save_ply_binary(ply_file_name, traj.data(), traj.size());
////////////////////////////////////////////////////////////////////////  
}


#include <type_traits>
//debug only not-safe trace
void GlobalMapping::print_submap_structure()
{
  int submap_index = 0;//debug
  for (const auto& submap : submaps) {
    
    submap_index++;
    if(submap_index == 1)//random submap
    {
      logger->info("Submap {} data size below",submap_index);

      logger->info("for random submap voxelmaps.size =  {}", submap->voxelmaps.size());// std::vector<gtsam_points::GaussianVoxelMap::Ptr> voxelmaps;  ///< Multi-resolution voxelmaps
      
      ////////////////////
      logger->info("for random submap frames (Optimized odometry frames) size =  {}", submap->optim_odom_frames.size());//std::vector<EstimationFrame::ConstPtr> frames;       ///< Optimized odometry frames
      if(submap->optim_odom_frames.at(1)->raw_frame)
      {
        logger->info("Optimized odometry frame 1 has raw_frame(PreprocessedFrame) ");//PreprocessedFrame::ConstPtr
        logger->info("Optimized odometry frame 1 raw_frame stamp = {}", submap->optim_odom_frames.at(1)->raw_frame->stamp); // Timestamp at the beginning of the scan
        logger->info("Optimized odometry frame 1 raw_frame scan_end_time = {}", submap->optim_odom_frames.at(1)->raw_frame->scan_end_time);  // Timestamp at the end of the scan

        logger->info("Optimized odometry frame 1 raw_frame times.size = {}",submap->optim_odom_frames.at(1)->raw_frame->times.size());            // Point timestamps w.r.t. the first pt
        logger->info("Optimized odometry frame 1 raw_frame intensities.size = {}", submap->optim_odom_frames.at(1)->raw_frame->intensities.size());      // Point intensities
        logger->info("Optimized odometry frame 1 raw_frame points.size = {}",submap->optim_odom_frames.at(1)->raw_frame->points.size());  // Points (homogeneous coordinates)

        logger->info("Optimized odometry frame 1 raw_frame k_neighbors = {}",submap->optim_odom_frames.at(1)->raw_frame->k_neighbors);             // Number of neighbors of each point
        logger->info("Optimized odometry frame 1 raw_frame neighbors.size = {}",submap->optim_odom_frames.at(1)->raw_frame->neighbors.size());  // Points (homogeneous coordinates)

        logger->info("Optimized odometry frame 1 raw_frame raw_points.size = {}",submap->optim_odom_frames.at(1)->raw_frame->raw_points->size());
        logger->info("Optimized odometry frame 1 raw_frame raw_points stamp = {}",submap->optim_odom_frames.at(1)->raw_frame->raw_points->stamp);
        logger->info("Optimized odometry frame 1 raw_frame raw_points times.size = {}",submap->optim_odom_frames.at(1)->raw_frame->raw_points->times.size());
 
        logger->info("Optimized odometry frame 1 raw_frame raw_points intensities.size = {}",submap->optim_odom_frames.at(1)->raw_frame->raw_points->intensities.size());
        logger->info("Optimized odometry frame 1 raw_frame raw_points times.points = {}",submap->optim_odom_frames.at(1)->raw_frame->raw_points->points.size());
        logger->info("Optimized odometry frame 1 raw_frame raw_points colors.size = {}",submap->optim_odom_frames.at(1)->raw_frame->raw_points->colors.size());

      }
      else
      {
        logger->info("Optimized odometry frame 1 has not raw_frame");//PreprocessedFrame::ConstPtr
      }
      
      if(submap->optim_odom_frames.at(1)->frame)
      {
        logger->info("Optimized odometry frame 1 frame member (PointCloud) size =  {}",submap->optim_odom_frames.at(1)->frame->size());//gtsam_points::PointCloud::ConstPtr frame
          logger->info("Optimized odometry frame 1 frame->has_times = {}",submap->optim_odom_frames.at(1)->frame->has_times());        ///< Check if the point cloud has per-point timestamps
          logger->info("Optimized odometry frame 1 frame->has_points = {}",submap->optim_odom_frames.at(1)->frame->has_points());       ///< Check if the point cloud has points
          logger->info("Optimized odometry frame 1 frame->has_normal = {}",submap->optim_odom_frames.at(1)->frame->has_normals());      ///< Check if the point cloud has point normals
          logger->info("Optimized odometry frame 1 frame->has_covs = {}",submap->optim_odom_frames.at(1)->frame->has_covs());         ///< Check if the point cloud has point covariances
          logger->info("Optimized odometry frame 1 frame->has_intensities = {}",submap->optim_odom_frames.at(1)->frame->has_intensities());  ///< Check if the point cloud has point intensities

          logger->info("Optimized odometry frame 1 frame->has_times_gpu = {}",submap->optim_odom_frames.at(1)->frame->has_times_gpu());        ///< Check if the point cloud has per-point timestamps on GPU
          logger->info("Optimized odometry frame 1 frame->has_points_gpu = {}",submap->optim_odom_frames.at(1)->frame->has_points_gpu());       ///< Check if the point cloud has points on GPU
          logger->info("Optimized odometry frame 1 frame->has_normals_gpu = {}",submap->optim_odom_frames.at(1)->frame->has_normals_gpu());      ///< Check if the point cloud has point normals on GPU
          logger->info("Optimized odometry frame 1 frame->has_covs_gpu = {}",submap->optim_odom_frames.at(1)->frame->has_covs_gpu());         ///< Check if the point cloud has point covariances on GPU
          logger->info("Optimized odometry frame 1 frame->has_intensities_gpu = {}",submap->optim_odom_frames.at(1)->frame->has_intensities_gpu());  ///< Check if the point cloud has point intensities on GPU

          logger->info("Submap {} Optimized odometry frame 1 frame size {}",submap_index, submap->optim_odom_frames.at(1)->frame->size());

          for (size_t i = 0; i < submap->optim_odom_frames.at(1)->frame->size(); i++)
          {
            if(i == 5)//random point in frame (PointCloud)
            {
              Eigen::Vector4d* point = submap->optim_odom_frames.at(1)->frame->points + i;
      
              for (auto &&p : *point)
              {
                logger->info("Submap 1  , point 5 in Optimized odometry frame 1 frame p = {}", p); 
              }
            }
         }
        
      }
      else
      {
        logger->info("Optimized odometry frame 1 has not frame (PointCloud)");
      }
      ////////////////////////
      
      logger->info("for random submap odom_frames ( Original odometry frames) size =  {}", submap->origin_odom_frames.size());//std::vector<EstimationFrame::ConstPtr> odom_frames;  ///< Original odometry frames
      if(submap->origin_odom_frames.at(1)->raw_frame)
      {
        logger->info("Original odometry frame 1 has raw_frame");//PreprocessedFrame::ConstPtr
      }
      else
      {
        logger->info("Original odometry frame 1 has not raw_frame");//PreprocessedFrame::ConstPtr
      }
      
      if(submap->origin_odom_frames.at(1)->frame)
        logger->info("Original odometry frame 1 frame member (PointCloud) size =  {}",submap->origin_odom_frames.at(1)->frame->size());//gtsam_points::PointCloud::ConstPtr frame
      else
        logger->info("Original odometry frame 1 has not frame (PointCloud)");

      ////////////////////////////////////

      logger->info("submap->merged_keyframe is PointCloud = {}", std::is_same_v<decltype(submap->merged_keyframe),  gtsam_points::PointCloud::Ptr>);
      logger->info("submap->merged_keyframe is PointCloudCPU = {}", std::is_same_v<decltype(submap->merged_keyframe),  gtsam_points::PointCloudCPU::Ptr>);
      logger->info("submap->merged_keyframe is PointCloudGPU = {}", std::is_same_v<decltype(submap->merged_keyframe),  gtsam_points::PointCloudGPU::Ptr>);
      
      //gtsam_points::PointCloudCPU* temp = (gtsam_points::PointCloudCPU*)submap->merged_keyframe.get();
      //Не помогло!! Разбирайся - почему в EstimationFrame::ConstPtr OdometryEstimationIMU::insert_frame()
      /*
          covariance_estimation->estimate(points_imu, raw_frame->neighbors, normals, covs);
          auto frame = std::make_shared<gtsam_points::PointCloudCPU>(points_imu);
          frame->add_covs(covs);
          frame->add_normals(normals);

          а здесь их нет ????
      */
      
  
      logger->info("for random submap frame->has_times = {}",submap->merged_keyframe->has_times());        ///< Check if the point cloud has per-point timestamps
      logger->info("for random submap frame->has_points = {}",submap->merged_keyframe->has_points());       ///< Check if the point cloud has points
      logger->info("for random submap frame->has_normal = {}",submap->merged_keyframe->has_normals());      ///< Check if the point cloud has point normals
      logger->info("for random submap frame->has_covs = {}",submap->merged_keyframe->has_covs());         ///< Check if the point cloud has point covariances
      logger->info("for random submap frame->has_intensities = {}",submap->merged_keyframe->has_intensities());  ///< Check if the point cloud has point intensities

      logger->info("for random submap frame->has_times_gpu = {}",submap->merged_keyframe->has_times_gpu());        ///< Check if the point cloud has per-point timestamps on GPU
      logger->info("for random submap frame->has_points_gpu = {}",submap->merged_keyframe->has_points_gpu());       ///< Check if the point cloud has points on GPU
      logger->info("for random submap frame->has_normals_gpu = {}",submap->merged_keyframe->has_normals_gpu());      ///< Check if the point cloud has point normals on GPU
      logger->info("for random submap frame->has_covs_gpu = {}",submap->merged_keyframe->has_covs_gpu());         ///< Check if the point cloud has point covariances on GPU
      logger->info("for random submap frame->has_intensities_gpu = {}",submap->merged_keyframe->has_intensities_gpu());  ///< Check if the point cloud has point intensities on GPU

      logger->info("Submap {} frame size {}",submap_index, submap->merged_keyframe->size());

      for (size_t i = 0; i < submap->merged_keyframe->size(); i++)
      {
        if(i == 5)//random point in frame (PointCloud)
        {
          Eigen::Vector4d* point = submap->merged_keyframe->points + i;
   
          for (auto &&p : *point)
          {
            logger->info("Submap 1  , point 5 in frame p = {}", p); 
          }
        }
        
      }
    }
  }

  return;

}



bool GlobalMapping::load(const std::string& path) {
  std::ifstream ifs(path + "/graph.txt");
  if (!ifs) {
    logger->error("failed to open {}/graph.txt", path);
    return false;
  }

  std::string token;
  int num_submaps, num_all_frames, num_matching_cost_factors;

  ifs >> token >> num_submaps;
  ifs >> token >> num_all_frames;
  ifs >> token >> num_matching_cost_factors;

  std::vector<std::tuple<std::string, int, int>> matching_cost_factors(num_matching_cost_factors);
  for (int i = 0; i < num_matching_cost_factors; i++) {
    auto& factor = matching_cost_factors[i];
    ifs >> token >> std::get<0>(factor) >> std::get<1>(factor) >> std::get<2>(factor);
  }

  logger->info("Load submaps");
  submaps.resize(num_submaps);
  subsampled_submaps.resize(num_submaps);
  for (int i = 0; i < num_submaps; i++) {
    auto submap = SubMap::load((boost::format("%s/%06d") % path % i).str());
    if (!submap) {
      return false;
    }

    // Adaptively determine the voxel resolution based on the median distance
    const int max_scan_count = 256;
    const double dist_median = gtsam_points::median_distance(submap->merged_keyframe, max_scan_count);
    const double p =
      std::max(0.0, std::min(1.0, (dist_median - params.submap_voxel_resolution_dmin) / (params.submap_voxel_resolution_dmax - params.submap_voxel_resolution_dmin)));
    const double base_resolution = params.submap_voxel_resolution + p * (params.submap_voxel_resolution_max - params.submap_voxel_resolution);

    gtsam_points::PointCloud::Ptr subsampled_submap;
    if (params.randomsampling_rate > 0.99) {
      subsampled_submap = submap->merged_keyframe;
    } else {
      subsampled_submap = gtsam_points::random_sampling(submap->merged_keyframe, params.randomsampling_rate, mt);
    }

    submaps[i] = submap;
    submaps[i]->voxelmaps.clear();
    subsampled_submaps[i] = subsampled_submap;

    if (params.enable_gpu) {
#ifdef GTSAM_POINTS_USE_CUDA
      subsampled_submaps[i] = gtsam_points::PointCloudGPU::clone(*subsampled_submaps[i]);

      for (int j = 0; j < params.submap_voxelmap_levels; j++) {
        const double resolution = base_resolution * std::pow(params.submap_voxelmap_scaling_factor, j);
        auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(resolution);
        voxelmap->insert(*subsampled_submaps[i]);
        submaps[i]->voxelmaps.push_back(voxelmap);
      }
#else
      logger->warn("GPU is enabled for global_mapping but gtsam_points was built without CUDA!!");
#endif
    } else {
      for (int j = 0; j < params.submap_voxelmap_levels; j++) {
        const double resolution = base_resolution * std::pow(params.submap_voxelmap_scaling_factor, j);
        auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(resolution);
        voxelmap->insert(*subsampled_submaps[i]);
        submaps[i]->voxelmaps.push_back(voxelmap);
      }
    }

    Callbacks::on_insert_submap(submap);
  }

  gtsam::Values values;
  gtsam::NonlinearFactorGraph graph;
  bool needs_recover = false;

  try {
    logger->info("deserializing factor graph");
    gtsam::deserializeFromBinaryFile(path + "/graph.bin", graph);
  } catch (boost::archive::archive_exception e) {
    logger->error("failed to deserialize factor graph!!");
    logger->error(e.what());
  } catch (std::exception& e) {
    logger->error("failed to deserialize factor graph!!");
    logger->error(e.what());
    needs_recover = true;
  }

  try {
    logger->info("deserializing values");
    gtsam::deserializeFromBinaryFile(path + "/values.bin", values);
  } catch (boost::archive::archive_exception e) {
    logger->error("failed to deserialize values!!");
    logger->error(e.what());
  } catch (std::exception& e) {
    logger->error("failed to deserialize values!!");
    logger->error(e.what());
    needs_recover = true;
  }

  logger->info("creating matching cost factors");
  for (const auto& factor : matching_cost_factors) {
    const auto type = std::get<0>(factor);
    const auto first = std::get<1>(factor);
    const auto second = std::get<2>(factor);

    if (type == "vgicp" || type == "vgicp_gpu") {
      if (params.enable_gpu) {
#ifdef GTSAM_POINTS_USE_CUDA
        const auto stream_buffer = std::any_cast<std::shared_ptr<gtsam_points::StreamTempBufferRoundRobin>>(stream_buffer_roundrobin)->get_stream_buffer();
        const auto& stream = stream_buffer.first;
        const auto& buffer = stream_buffer.second;

        for (const auto& voxelmap : submaps[first]->voxelmaps) {
          graph.emplace_shared<gtsam_points::IntegratedVGICPFactorGPU>(X(first), X(second), voxelmap, subsampled_submaps[second], stream, buffer);
        }
#else
        logger->warn("GPU is enabled but gtsam_points was built without CUDA!!");
#endif
      } else {
        for (const auto& voxelmap : submaps[first]->voxelmaps) {
          graph.emplace_shared<gtsam_points::IntegratedVGICPFactor>(X(first), X(second), voxelmap, subsampled_submaps[second]);
        }
      }
    } else {
      logger->warn("unsupported matching cost factor type ({})", type);
    }
  }

  const size_t num_factors_before = graph.size();
  const auto remove_loc = std::remove_if(graph.begin(), graph.end(), [](const auto& factor) { return factor == nullptr; });
  graph.erase(remove_loc, graph.end());
  if (graph.size() != num_factors_before) {
    logger->warn("removed {} invalid factors", num_factors_before - graph.size());
    needs_recover = true;
  }

  if (needs_recover) {
    logger->warn("recovering factor graph");
    const auto recovered = recover_graph(graph, values);
    graph.add(recovered.first);
    values.insert_or_assign(recovered.second);
  }

  logger->info("optimize");
  Callbacks::on_smoother_update(*isam2, graph, values);
  auto result = update_isam2(graph, values);
  Callbacks::on_smoother_update_result(*isam2, result);

  update_submaps();
  Callbacks::on_update_submaps(submaps);

  logger->info("done");

  return true;
}

void GlobalMapping::recover_graph() {
  const auto recovered = recover_graph(isam2->getFactorsUnsafe(), isam2->calculateEstimate());
  update_isam2(recovered.first, recovered.second);
}

// Recover the graph by adding missing values and factors
std::pair<gtsam::NonlinearFactorGraph, gtsam::Values> GlobalMapping::recover_graph(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values) const {
  logger->info("recovering graph");
  bool enable_imu = false;
  for (const auto& value : values) {
    const char chr = gtsam::Symbol(value.key).chr();
    enable_imu |= (chr == 'e' || chr == 'v' || chr == 'b');
  }
  for (const auto& factor : graph) {
    enable_imu |= boost::dynamic_pointer_cast<gtsam::ImuFactor>(factor) != nullptr;
  }

  logger->info("enable_imu={}", enable_imu);

  logger->info("creating connectivity map");
  bool prior_exists = false;
  std::unordered_map<gtsam::Key, std::set<gtsam::Key>> connectivity_map;
  for (const auto& factor : graph) {
    if (!factor) {
      continue;
    }

    for (const auto key : factor->keys()) {
      for (const auto key2 : factor->keys()) {
        connectivity_map[key].insert(key2);
      }
    }

    if (factor->keys().size() == 1 && factor->keys()[0] == X(0)) {
      prior_exists |= boost::dynamic_pointer_cast<gtsam_points::LinearDampingFactor>(factor) != nullptr;
    }
  }

  if (!prior_exists) {
    logger->warn("X0 prior is missing");
    new_factors->emplace_shared<gtsam_points::LinearDampingFactor>(X(0), 6, params.init_pose_damping_scale);
  }

  logger->info("fixing missing values and factors");
  const auto prior_noise3 = gtsam::noiseModel::Isotropic::Precision(3, 1e6);
  const auto prior_noise6 = gtsam::noiseModel::Isotropic::Precision(6, 1e6);

  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;
  for (int i = 0; i < submaps.size(); i++) {
    if (!values.exists(X(i))) {
      logger->warn("X{} is missing", i);
      new_values.insert(X(i), gtsam::Pose3(submaps[i]->T_world_origin.matrix()));
    }

    if (connectivity_map[X(i)].count(X(i + 1)) == 0 && i != submaps.size() - 1) {
      logger->warn("X{} -> X{} is missing", i, i + 1);

      const Eigen::Isometry3d delta = submaps[i]->origin_odom_frame()->T_world_sensor().inverse() * submaps[i + 1]->origin_odom_frame()->T_world_sensor();
      new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(i), X(i + 1), gtsam::Pose3(delta.matrix()), prior_noise6);
    }

    if (!enable_imu) {
      continue;
    }

    const auto submap = submaps[i];
    const gtsam::imuBias::ConstantBias imu_biasL(submap->optim_odom_frames.front()->imu_bias);
    const gtsam::imuBias::ConstantBias imu_biasR(submap->optim_odom_frames.back()->imu_bias);
    const Eigen::Vector3d v_origin_imuL = submap->T_world_origin.linear().inverse() * submap->optim_odom_frames.front()->v_world_imu;
    const Eigen::Vector3d v_origin_imuR = submap->T_world_origin.linear().inverse() * submap->optim_odom_frames.back()->v_world_imu;

    if (i != 0) {
      if (!values.exists(E(i * 2))) {
        logger->warn("E{} is missing", i * 2);
        new_values.insert(E(i * 2), gtsam::Pose3((submap->T_world_origin * submap->T_origin_endpoint_L).matrix()));
      }
      if (!values.exists(V(i * 2))) {
        logger->warn("V{} is missing", i * 2);
        new_values.insert(V(i * 2), (submap->T_world_origin.linear() * v_origin_imuL).eval());
      }
      if (!values.exists(B(i * 2))) {
        logger->warn("B{} is missing", i * 2);
        new_values.insert(B(i * 2), imu_biasL);
      }

      if (connectivity_map[X(i)].count(E(i * 2)) == 0) {
        logger->warn("X{} -> E{} is missing", i, i * 2);
        new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(i), E(i * 2), gtsam::Pose3(submap->T_origin_endpoint_L.matrix()), prior_noise6);
      }
      if (connectivity_map[X(i)].count(V(i * 2)) == 0) {
        logger->warn("X{} -> V{} is missing", i, i * 2);
        new_factors.emplace_shared<gtsam_points::RotateVector3Factor>(X(i), V(i * 2), v_origin_imuL, prior_noise3);
      }
      if (connectivity_map[B(i * 2)].count(B(i * 2)) == 0) {
        logger->warn("B{} -> B{} is missing", i * 2, i * 2);
        new_factors.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(i * 2), imu_biasL, prior_noise6);
      }

      if (connectivity_map[B(i * 2)].count(B(i * 2 + 1)) == 0) {
        logger->warn("B{} -> B{} is missing", i * 2, i * 2 + 1);
        new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(B(i * 2), B(i * 2 + 1), gtsam::imuBias::ConstantBias(), prior_noise6);
      }
    }

    if (!values.exists(E(i * 2 + 1))) {
      logger->warn("E{} is missing", i * 2 + 1);
      new_values.insert(E(i * 2 + 1), gtsam::Pose3((submap->T_world_origin * submap->T_origin_endpoint_R).matrix()));
    }
    if (!values.exists(V(i * 2 + 1))) {
      logger->warn("V{} is missing", i * 2 + 1);
      new_values.insert(V(i * 2 + 1), (submap->T_world_origin.linear() * v_origin_imuR).eval());
    }
    if (!values.exists(B(i * 2 + 1))) {
      logger->warn("B{} is missing", i * 2 + 1);
      new_values.insert(B(i * 2 + 1), imu_biasR);
    }

    if (connectivity_map[X(i)].count(E(i * 2 + 1)) == 0) {
      logger->warn("X{} -> E{} is missing", i, i * 2 + 1);
      new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(i), E(i * 2 + 1), gtsam::Pose3(submap->T_origin_endpoint_R.matrix()), prior_noise6);
    }
    if (connectivity_map[X(i)].count(V(i * 2 + 1)) == 0) {
      logger->warn("X{} -> V{} is missing", i, i * 2 + 1);
      new_factors.emplace_shared<gtsam_points::RotateVector3Factor>(X(i), V(i * 2 + 1), v_origin_imuR, prior_noise3);
    }
    if (connectivity_map[B(i * 2 + 1)].count(B(i * 2 + 1)) == 0) {
      logger->warn("B{} -> B{} is missing", i * 2 + 1, i * 2 + 1);
      new_factors.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(i * 2 + 1), imu_biasR, prior_noise6);
    }
  }

  logger->info("recovering done");

  return {new_factors, new_values};
}

}  // namespace glim