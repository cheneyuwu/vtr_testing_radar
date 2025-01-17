#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_msgs/msg/string.hpp"

#include "vtr_common/timing/utils.hpp"
#include "vtr_common/utils/filesystem.hpp"
#include "vtr_lidar/pipeline.hpp"
#include "vtr_logging/logging_init.hpp"
#include "vtr_tactic/pipelines/factory.hpp"
#include "vtr_tactic/rviz_tactic_callback.hpp"
#include "vtr_tactic/tactic.hpp"

#include "yaml-cpp/yaml.h"
#include "vtr_testing_aeva/utils.hpp"
#include <iomanip>
#include <fstream>

namespace fs = std::filesystem;
using namespace vtr;
using namespace vtr::common;
using namespace vtr::logging;
using namespace vtr::tactic;
using namespace vtr::testing;

YAML::Node loadYamlFile(const std::string& file_path) {
  YAML::Node config;
  try {
    config = YAML::LoadFile(file_path);
  } catch (const YAML::BadFile& e) {
    throw std::runtime_error("Unable to open YAML file: " + file_path);
  } catch (const YAML::ParserException& e) {
    throw std::runtime_error("Error parsing YAML file: " + file_path);
  }
  return config;
}

Eigen::MatrixXd readCSVtoEigenXd(std::ifstream &csv) {
  std::string line;
  std::string cell;
  std::vector<std::vector<double>> mat_vec;
  while (std::getline(csv, line)) {
    std::stringstream lineStream(line);
    std::vector<double> row_vec;
    while (std::getline(lineStream, cell, ',')) {
      row_vec.push_back(std::stof(cell));
    }
    mat_vec.push_back(row_vec);
  }
  Eigen::MatrixXd output = Eigen::MatrixXd(mat_vec.size(), mat_vec[0].size());
  for (int i = 0; i < (int)mat_vec.size(); ++i) output.row(i) = Eigen::VectorXd::Map(&mat_vec[i][0], mat_vec[i].size());
  return output;
}

int64_t getStampFromPath(const std::string &path) {
  std::vector<std::string> parts;
  boost::split(parts, path, boost::is_any_of("/"));
  std::string stem = parts[parts.size() - 1];
  boost::split(parts, stem, boost::is_any_of("."));
  int64_t time1 = std::stoll(parts[0]);
  return time1 * 1000;
}

Eigen::Vector3d polarToXYZ(double range, double azimuth, double elevation) {
  double xy = range * cos(elevation);
  return Eigen::Vector3d(xy * cos(azimuth), xy * sin(azimuth), range * sin(elevation));
}

std::vector<Eigen::MatrixXd> loadElevationOrder(const std::string &bo_path) {
  // load values for computing line id from elevation
  std::vector<Eigen::MatrixXd> elevation_order_by_beam_id_;

  // read elevation settings
  std::string path = bo_path; // + "/mean_elevation_beam_order_0";
  std::ifstream csv(path);
  if (!csv) throw std::ios::failure("Error opening csv file");
  Eigen::MatrixXd elevation_order = readCSVtoEigenXd(csv);

  for (int j = 0; j < 4; ++j) {   // 4 beams   
    Eigen::MatrixXd elevation_order_for_this_beam(elevation_order.rows()/4, 2);  // first column is mean elevation, second column is row id
    int h = 0;
    for (int r = 0; r < elevation_order.rows(); ++r) {
      // first column is mean elevation. Second column is beam id
      if (elevation_order(r, 1) == j) {
        elevation_order_for_this_beam(h, 0) = elevation_order(r, 0);
        elevation_order_for_this_beam(h, 1) = r;
        ++h;
      }
    } // end for r
    assert(h == elevation_order.rows()/4);
    elevation_order_by_beam_id_.push_back(elevation_order_for_this_beam);
  } // end for j
  assert(elevation_order_by_beam_id_.size() == 4); // 4 beams

  return elevation_order_by_beam_id_;
}

std::pair<int64_t, Eigen::MatrixXd> load_lidar(const std::string &path, const std::string &bo_path, double start_time, double end_time, int64_t filename) {
  // load Aeries I pointcloud
  std::ifstream ifs(path, std::ios::binary);
  std::vector<char> buffer(std::istreambuf_iterator<char>(ifs), {});
  unsigned float_offset = 4; // float32
  unsigned fields = 7;  // x, y, z, i, r, t, b
  unsigned point_step = float_offset * fields;
  unsigned N = floor(buffer.size() / point_step);

  std::vector<Eigen::VectorXd> points; // Vector to store valid points dynamically

  auto getFloatFromByteArray = [](char *byteArray, unsigned index) -> float {
    return *((float *)(byteArray + index));
  };

  auto elevation_order = loadElevationOrder(bo_path);

  for (unsigned i(0); i < N; i++) {
    int bufpos = i * point_step;
    int offset = 0;

    // Temporary variables
    double x, y, z, radial_velocity, intensity, time_temp;
    int64_t time_keep;
    int beam_id, line_id, face_id, sensor_id;

    // x, y, z
    x = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    y = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    z = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    
    ++offset;
    // Intensity
    intensity = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    // Radial velocity
    radial_velocity = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    // Timestamp
    time_temp = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset); // sec
    double t = double(filename / 1000) * 1.0e-6;
    time_keep = (int64_t)((time_temp + t) * 1e9); // time_keep in format expected by vtr
    time_temp = time_temp + start_time;       // time_temp to check if pt is between start and end time
    ++offset;
    // Beam id
    beam_id = (int)(getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset));
    // Face id - 0 because not available on aeries I
    face_id = 0;
    // Sensor if
    sensor_id = 0;

    // compute elevation
    const double xy = sqrt(x*x + y*y);
    const double elevation = atan2(z, xy);
    
    // determine row by matching by beam_id (0, 1, 2, or 3) and closest elevation to precalculated values
    // note: elevation_order_by_beam_id_[point.beam_id] first column is mean elevation, second column is row id
    const auto ele_diff = elevation_order[beam_id].col(0).array() - elevation;
    double min_val = ele_diff(0)*ele_diff(0);
    size_t min_id = 0;
    for (size_t i = 1; i < ele_diff.rows(); ++i) {
      const auto val = ele_diff(i) * ele_diff(i);
      if (val < min_val) {
        min_val = val;
        min_id = i;
      }
    }

    line_id = elevation_order[beam_id](min_id, 1);

    // Include if within start and end time
    if (time_temp > start_time && time_temp <= end_time) {
        Eigen::VectorXd point(10);
        //std::cout << "time_keep: " << time_keep << std::endl;
        point << x, y, z, radial_velocity, intensity, time_keep, beam_id, line_id, face_id, sensor_id;
        points.push_back(point);
    }
  }

  // Convert vector to Eigen::MatrixXd
  Eigen::MatrixXd pc(points.size(), 10);
  for (size_t k = 0; k < points.size(); ++k) {
    pc.row(k) = points[k];
  }
  return std::make_pair(fields, pc);
}

std::pair<int64_t, Eigen::MatrixXd> load_new_lidar(const std::string &path, double start_time, double end_time, int64_t filename) {
  // load Aeries II pointcloud
  std::ifstream ifs(path, std::ios::binary);
  std::vector<char> buffer(std::istreambuf_iterator<char>(ifs), {});
  unsigned float_offset = 4; // float32
  // 9 point cloud fields, use 10 because point_flags is 64 bits
  unsigned fields = 10;  // x, y, z, radial velocity, intensity, signal quality, reflectivity, time, point_flags (beam_id, line_id, face_id)
  unsigned point_step = float_offset * fields;
  unsigned N = floor(buffer.size() / point_step);

  std::vector<Eigen::VectorXd> points; // Vector to store valid points dynamically

  auto getFloatFromByteArray = [](char *byteArray, unsigned index) -> float {
    return *((float *)(byteArray + index));
  };

  for (unsigned i(0); i < N; i++) {
    int bufpos = i * point_step;
    int offset = 0;

    // Temporary variables
    double x, y, z, radial_velocity, intensity, time_temp;
    int64_t time_keep;
    int beam_id, line_id, face_id, sensor_id;

    // x, y, z
    x = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    y = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    z = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);

    ++offset;
    // Radial velocity
    radial_velocity = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    // Intensity
    intensity = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    // Skip signal quality
    ++offset;
    // Skip reflectivity
    ++offset;
    // Timestamp
    time_temp = (int64_t)(getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset)); // nanosec
    double t = double(filename);
    time_keep = (int64_t)(time_temp + t);
    time_temp = time_temp * 1e-9 + start_time;
    ++offset;
    // Point Flags (64 bit flag, only need first 32 bits)
    uint64_t point_flags = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);

    // Extract flags
    line_id = 63 - (int)((point_flags >> 8) & 0xFF); // flip order
    beam_id = (int)((point_flags >> 16) & 0xF);
    face_id = (int)((point_flags >> 22) & 0xF);

    // Error checks
    if (line_id < 0 || line_id >= 64) continue;
    if (face_id < 0 || face_id > 5) continue;

    // Sensor id - only one sensor
    sensor_id = 0;

    // Include if within start and end time
    if (time_temp > start_time && time_temp <= end_time) {
        Eigen::VectorXd point(10);
        point << x, y, z, radial_velocity, intensity, time_keep, beam_id, line_id, face_id, sensor_id;
        points.push_back(point);
    }
  }

  // Convert vector to Eigen::MatrixXd
  Eigen::MatrixXd pc(points.size(), 10);
  for (size_t k = 0; k < points.size(); ++k) {
    pc.row(k) = points[k];
  }
  return std::make_pair(fields, pc);
}

EdgeTransform load_T_lidar_robot(bool new_lidar) {
  Eigen::Matrix4d T_lidar_vehicle_mat;
  if (new_lidar) {
    // Aeries II boreas
    T_lidar_vehicle_mat << 0.99982945,  0.01750912,  0.00567659, -1.03971349,
                          -0.01754661,  0.99973757,  0.01034526, -0.38788971,
                          -0.00549427, -0.01044368,  0.99993037, -1.69798033,
                           0.0,         0.0,         0.0,         1.0;
  } else {
    // Aeries I boreas
    T_lidar_vehicle_mat << 0.9999366830849237, 0.008341717781538466, 0.0075534496251198685, -1.0119098938516395,
                          -0.008341717774127972, 0.9999652112886684, -3.150635091210066e-05, -0.39658824335171944,
                          -0.007553449599178521, -3.1504388681967066e-05, 0.9999714717963843, -1.697000000000001,
                           0, 0, 0, 1;
  }
  
  EdgeTransform T_lidar_robot(T_lidar_vehicle_mat,                    // transform
                              Eigen::Matrix<double, 6, 6>::Zero());   // covariance
  return T_lidar_robot;
}

bool filecomp (std::string file1, std::string file2) { 
  long long i = std::stoll(file1.substr(0, file1.find(".")));
  long long j = std::stoll(file2.substr(0, file2.find(".")));
  return (i<j); 
}

std::string getFirstFilename(const std::string& dir_path) {
    std::string first_filename;
    std::filesystem::directory_iterator dir_iter(dir_path);
    if (dir_iter != std::filesystem::directory_iterator()) {
        first_filename = dir_iter->path().filename().string();
    }
    return first_filename;
}

Eigen::MatrixXd readGyroToEigenXd(const std::string &file_path, const int64_t& initial_timestamp_micro, const std::string& dataset) {
  // this function is specifically designed for 2 datasets: aeva_boreas and aeva_hq
  if (dataset != "aevaI_boreas" && dataset != "aevaII_boreas") 
    throw std::runtime_error{"[readGyroToEigenXd] unknown dataset specified!"};

  std::ifstream imu_file(file_path);
  std::vector<std::vector<double>> mat_vec;
  if (imu_file.is_open()) {
    std::string line;
    std::getline(imu_file, line);  // header
    std::vector<double> row_vec(4);
    for (; std::getline(imu_file, line);) {
      if (line.empty()) continue;
      std::stringstream ss(line);

      int64_t timestamp = 0;
      double timestamp_sec = 0;
      double r = 0, p = 0, y = 0;
      for (int i = 0; i < 7; ++i) {
        std::string value;
        std::getline(ss, value, ',');

        if (i == 0) {
          timestamp = std::stol(value);
          timestamp_sec = static_cast<double>(timestamp - initial_timestamp_micro)*1e-6;
        } else if (dataset == "aevaI_boreas") {  // Note: r and p are flipped for aeva_boreas
          if (i == 2) 
            r = std::stod(value);
          else if (i == 1)
            p = std::stod(value);
          else if (i == 3)
            y = std::stod(value);
        } else if (dataset == "aevaII_boreas") {
          if (i == 1)
            r = std::stod(value);
          else if (i == 2)
            p = std::stod(value);
          else if (i == 3)
            y = std::stod(value);
        }
      } // end for row
      row_vec[0] = timestamp_sec;
      row_vec[1] = r;
      row_vec[2] = p;
      row_vec[3] = y;
      mat_vec.push_back(row_vec);
    } // end for line
  } // end if
  else {
    throw std::runtime_error{"unable to open file: " + file_path};
  }

  // output eigen matrix
  Eigen::MatrixXd output = Eigen::MatrixXd(mat_vec.size(), mat_vec[0].size());
  for (int i = 0; i < (int)mat_vec.size(); ++i) output.row(i) = Eigen::VectorXd::Map(&mat_vec[i][0], mat_vec[i].size());
  return output;
}

Eigen::Matrix3d toRoll(const double &r) {
  Eigen::Matrix3d roll;
  roll << 1, 0, 0, 0, cos(r), sin(r), 0, -sin(r), cos(r);
  return roll;
}

Eigen::Matrix3d toPitch(const double &p) {
  Eigen::Matrix3d pitch;
  pitch << cos(p), 0, -sin(p), 0, 1, 0, sin(p), 0, cos(p);
  return pitch;
}

Eigen::Matrix3d toYaw(const double &y) {
  Eigen::Matrix3d yaw;
  yaw << cos(y), sin(y), 0, -sin(y), cos(y), 0, 0, 0, 1;
  return yaw;
}

Eigen::Matrix3d rpy2rot(const double &r, const double &p, const double &y) {
  return toRoll(r) * toPitch(p) * toYaw(y);
}

EdgeTransform load_T_enu_lidar_init(const fs::path &path) {
  std::ifstream ifs(path / "applanix" / "aeva_poses.csv", std::ios::in);

  std::string header;
  std::getline(ifs, header);

  std::string first_pose;
  std::getline(ifs, first_pose);

  std::stringstream ss{first_pose};
  std::vector<double> gt;
  for (std::string str; std::getline(ss, str, ',');)
    gt.push_back(std::stod(str));

  Eigen::Matrix4d T_mat = Eigen::Matrix4d::Identity();
  T_mat.block<3, 3>(0, 0) = rpy2rot(gt[7], gt[8], gt[9]);
  T_mat.block<3, 1>(0, 3) << gt[1], gt[2], gt[3];

  EdgeTransform T(T_mat);
  T.setZeroCovariance();

  return T;
}

int main(int argc, char **argv) {
  // disable eigen multi-threading
  Eigen::setNbThreads(1);

  // TO DO: combine with aeva_boreas.yaml! don't want 2 configs
  std::string yaml_file_path = "external/vtr_testing_radar/src/vtr_testing_aeva/config/aeva_boreas.yaml";
  YAML::Node config = loadYamlFile(yaml_file_path);

  // load options
  int init_frame = 0;
  int last_frame = 100000;
  bool aeriesII = config["/**"]["ros__parameters"]["aeriesII"].as<bool>();
  std::vector<double> temp = config["/**"]["ros__parameters"]["preprocessing"]["filtering"]["const_gyro_bias"].as<std::vector<double>>();
  std::vector<Eigen::Vector3d> const_gyro_bias;
  // temporarily commented out -- check if needed
  // for (size_t i = 0; i < temp.size(); i += 3) {
  //   const_gyro_bias.push_back(Eigen::Vector3d(temp[i], temp[i+1], temp[i+2]));
  // }

  rclcpp::init(argc, argv);
  const std::string node_name = "boreas_localization_" + random_string(10);
  auto node = rclcpp::Node::make_shared(node_name);

  // odometry sequence directory
  const auto odo_dir_str =
      node->declare_parameter<std::string>("odo_dir", "/tmp");
  fs::path odo_dir{utils::expand_user(utils::expand_env(odo_dir_str))};

  // localization sequence directory
  const auto loc_dir_str =
      node->declare_parameter<std::string>("loc_dir", "/tmp");
  fs::path loc_dir{utils::expand_user(utils::expand_env(loc_dir_str))};

  // Output directory
  const auto data_dir_str =
      node->declare_parameter<std::string>("data_dir", "/tmp");
  fs::path data_dir{utils::expand_user(utils::expand_env(data_dir_str))};

  // Configure logging
  const auto log_to_file = node->declare_parameter<bool>("log_to_file", false);
  const auto log_debug = node->declare_parameter<bool>("log_debug", false);
  const auto log_enabled = node->declare_parameter<std::vector<std::string>>(
      "log_enabled", std::vector<std::string>{});
  std::string log_filename;
  if (log_to_file) {
    // Log into a subfolder of the data directory (if requested to log)
    auto log_name = "vtr-" + timing::toIsoFilename(timing::clock::now());
    log_filename = data_dir / (log_name + ".log");
  }
  configureLogging(log_filename, log_debug, log_enabled);

  CLOG(WARNING, "test") << "Odometry Directory: " << odo_dir.string();
  CLOG(WARNING, "test") << "Localization Directory: " << loc_dir.string();
  CLOG(WARNING, "test") << "Output Directory: " << data_dir.string();

  std::vector<std::string> parts;
  boost::split(parts, loc_dir_str, boost::is_any_of("/"));
  auto stem = parts.back();
  boost::replace_all(stem, "-", "_");
  CLOG(WARNING, "test") << "Publishing status to topic: "
                        << ("aeva_" + stem + "_lidar_localization");
  const auto status_publisher = node->create_publisher<std_msgs::msg::String>(
      "aeva_" + stem + "_lidar_localization", 1);

  // Pose graph
  auto graph = tactic::Graph::MakeShared((data_dir / "graph").string(), true);

  // Pipeline
  auto pipeline_factory = std::make_shared<ROSPipelineFactory>(node);
  auto pipeline = pipeline_factory->get("pipeline");
  auto pipeline_output = pipeline->createOutputCache();
  // some modules require node for visualization
  pipeline_output->node = node;

  // Tactic Callback
  auto callback = std::make_shared<RvizTacticCallback>(node);

  // Tactic
  auto tactic =
      std::make_shared<Tactic>(Tactic::Config::fromROS(node), pipeline,
                               pipeline_output, graph, callback);
  tactic->setPipeline(PipelineMode::RepeatFollow);
  tactic->addRun();

  // Get the path that we should repeat
  VertexId::Vector sequence;
  sequence.reserve(graph->numberOfVertices());
  CLOG(WARNING, "test") << "Total number of vertices: "
                        << graph->numberOfVertices();
  // Extract the privileged sub graph from the full graph.
  using LocEvaluator = tactic::PrivilegedEvaluator<tactic::GraphBase>;
  auto evaluator = std::make_shared<LocEvaluator>(*graph);
  auto privileged_path = graph->getSubgraph(0ul, evaluator);
  std::stringstream ss;
  ss << "Repeat vertices: ";
  for (auto it = privileged_path->begin(0ul); it != privileged_path->end();
       ++it) {
    ss << it->v()->id() << " ";
    sequence.push_back(it->v()->id());
  }
  CLOG(WARNING, "test") << ss.str();

  /// NOTE: odometry is teach, localization is repeat
  auto T_loc_odo_init = [&]() {
    const auto T_enu_lidar_odo = load_T_enu_lidar_init(odo_dir);
    const auto T_enu_lidar_loc = load_T_enu_lidar_init(loc_dir);

    const auto T_lidar_robot = load_T_lidar_robot(aeriesII); // T_s_v

    return T_lidar_robot.inverse() * T_enu_lidar_loc.inverse() * T_enu_lidar_odo * T_lidar_robot;
  }();
  T_loc_odo_init.setCovariance(Eigen::Matrix<double, 6, 6>::Identity());
  CLOG(WARNING, "test")
      << "Transform from localization to odometry has been set to "
      << T_loc_odo_init.vec().transpose();

  tactic->setPath(sequence, /* trunk sid */ 0, T_loc_odo_init, true);

  // Frame and transforms
  std::string robot_frame = "robot";
  std::string lidar_frame = "lidar";
  
  const auto T_lidar_robot = load_T_lidar_robot(aeriesII);
  CLOG(WARNING, "test") << "Transform from " << robot_frame << " to "
                        << lidar_frame << " has been set to" << T_lidar_robot;

  std::string dir_path_ = loc_dir.string() + "/aeva/";
  std::vector<std::string> filenames_;
  int64_t initial_timestamp_micro_;
  int init_frame_ = 0;
  int curr_frame_ = 0;
  int last_frame_ = std::numeric_limits<int>::max();  // exclusive bound

  auto dir_iter = std::filesystem::directory_iterator(dir_path_);
  std::count_if(begin(dir_iter), end(dir_iter), [&filenames_](auto &entry) {
    if (entry.is_regular_file()) filenames_.emplace_back(entry.path().filename().string());
    return entry.is_regular_file();
  });
  std::sort(filenames_.begin(), filenames_.end(), filecomp);  // custom comparison

  last_frame_ = std::min(last_frame_, last_frame);
  curr_frame_ = std::max((int)0, init_frame);
  init_frame_ = std::max((int)0, init_frame);

  initial_timestamp_micro_ = std::stoll(filenames_[0].substr(0, filenames_[0].find(".")));

  // Gyroscope data
  // initialize gyro
  std::vector<Eigen::MatrixXd> gyro_data_;
  gyro_data_.clear();
  std::string gyro_path = loc_dir.string() + "/applanix/" + "aeva_imu.csv";
  if (aeriesII) {
    // load Aeries II boreas gyro
    gyro_data_.push_back(readGyroToEigenXd(gyro_path, initial_timestamp_micro_, "aevaII_boreas"));
    gyro_data_.back().rightCols<3>() *= -1.0; // flip reference frame

    // compute gyro bias while vehicle is stationary
    const_gyro_bias.push_back(gyro_data_.back().topRightCorner(200, 3).colwise().mean().transpose());
    std::cout << "Gyro bias: " << const_gyro_bias.back().transpose() << std::endl;
  } else {
    // load Aeries I boreas gyro
    gyro_data_.push_back(readGyroToEigenXd(gyro_path, initial_timestamp_micro_, "aevaI_boreas"));
  }

  // to do: move this to config
  std::vector<double> cov_arr; = {1.09504535e-04, 1.73659780e-04, 5.01201833e-05};

  std::vector<Eigen::Matrix3d> gyro_invcov;
  gyro_invcov.resize(1);
  gyro_invcov[0] = Eigen::Matrix3d::Identity();
  gyro_invcov[0](0,0) = 1.0/(cov_arr[0]);
  gyro_invcov[0](1,1) = 1.0/(cov_arr[1]);
  gyro_invcov[0](2,2) = 1.0/(cov_arr[2]);                   

  auto tf_sbc = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
  auto msg =
      tf2::eigenToTransform(Eigen::Affine3d(T_lidar_robot.inverse().matrix()));
  msg.header.frame_id = robot_frame;
  msg.child_frame_id = lidar_frame;
  tf_sbc->sendTransform(msg);

  const auto clock_publisher =
      node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

  // List of lidar data
  std::vector<fs::directory_entry> files;
  for (const auto &dir_entry : fs::directory_iterator{loc_dir / "aeva"})
    if (!fs::is_directory(dir_entry)) files.push_back(dir_entry);
  std::sort(files.begin(), files.end());
  CLOG(WARNING, "test") << "Found " << files.size() << " lidar data";

  // thread handling variables
  TestControl test_control(node);

  // main loop
  int frame = 0;
  auto it = files.begin();
  while (it + 1 != files.end()) {
    if (!rclcpp::ok()) break;
    rclcpp::spin_some(node);
    if (test_control.terminate()) break;
    if (!test_control.play()) continue;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(test_control.delay()));

    const auto filename = getStampFromPath((it)->path().string());
    int64_t time_delta_micro = filename / 1000 - initial_timestamp_micro_;
    double start_time = static_cast<double>(time_delta_micro) / 1e6;

    std::cout << "Frame " << frame << " with timestamp " << filename << std::endl;

    // Note: we peak into future data for the end timestamp for evaluation convenience. An online implementation
    // would need different logic, i.e., use the last timestamp of the pointcloud
    double end_time = start_time + 0.1;
    // Get the name of the next file
    int64_t next_file_name;
    if ((it + 1) != files.end()) {
      next_file_name = getStampFromPath((it + 1)->path().string());
      auto end_time = static_cast<double>(next_file_name - initial_timestamp_micro_) / 1e6;
    }
    int64_t start_name = filename;
    
    double dt = 0;
    Eigen::MatrixXd points;
    if (aeriesII) {
      // load Aeries II boreas pointcloud
      std::tie(std::ignore, points) = load_new_lidar(it->path().string(), start_time, end_time, start_name);
    } else {
      dt = 0.1; // aeries I gyro time sync ~0.1s off
      // Beam Order Path
      std::string bo_path = config["/**"]["ros__parameters"]["bo_path"].as<std::string>();
      // load Aeries I boreas pointcloud
      auto [fields, points] = load_lidar(it->path().string(), bo_path, start_time, end_time, start_name);
    }

    // load gyro data
    std::vector<Eigen::MatrixXd> current_gyro;
    for (int sensorid = 0; sensorid < gyro_data_.size(); ++sensorid) {
      std::vector<int> inds; inds.clear();
      for (int r = 0; r < gyro_data_[sensorid].rows(); ++r) {
        double meas_time = gyro_data_[sensorid](r, 0) - dt;
        if (meas_time >= start_time && meas_time < end_time)
          inds.push_back(r);
      } // end for r

      if (inds.size() == 0) {
        // no measurements
        current_gyro.push_back(Eigen::Matrix<double, 1, 1>());  // 1x1 zero matrix
        continue;
      }

      Eigen::MatrixXd temp_gyro(inds.size(), 4);
      for (int r = 0; r < inds.size(); ++r) {
        temp_gyro(r, 0) = gyro_data_[sensorid](inds[r], 0) - dt; // timestamp
        temp_gyro.row(r).rightCols<3>() = gyro_data_[sensorid].row(inds[r]).rightCols<3>();
        temp_gyro.row(r).rightCols<3>() -= const_gyro_bias[0].transpose();  // apply gyro bias
      }
      current_gyro.push_back(temp_gyro);
    } // end for sensorid  

    // publish clock for sim time
    auto time_msg = rosgraph_msgs::msg::Clock();
    time_msg.clock = rclcpp::Time(start_name);
    clock_publisher->publish(time_msg);

    // Convert message to query_data format and store into query_data
    auto query_data = std::make_shared<lidar::LidarQueryCache>();

    // some modules require node for visualization
    query_data->node = node;

    // set timestamp
    query_data->stamp.emplace(start_name);

    // make up some environment info (not important)
    tactic::EnvInfo env_info;
    env_info.terrain_type = 0;
    query_data->env_info.emplace(env_info);

    // set lidar frame
    query_data->points.emplace(std::move(points));

    // fill in the vehicle to sensor transform and frame name
    query_data->T_s_r.emplace(T_lidar_robot);

    // set gyro data
    query_data->gyro.emplace(current_gyro);
    query_data->gyro_invcov.emplace(gyro_invcov);
    query_data->const_gyro_bias.emplace(const_gyro_bias);

    // set timestamp of first frame [us]
    query_data->initial_timestamp.emplace(initial_timestamp_micro_);

    // set timestamp of next state [ns]
    query_data->next_state_time.emplace(next_file_name);

    // execute the pipeline
    tactic->input(query_data);

    std_msgs::msg::String status_msg;
    status_msg.data = "Finished processing lidar frame " +
                      std::to_string(frame) + " with timestamp " +
                      std::to_string(start_name);
    status_publisher->publish(status_msg);

    ++it;
    ++frame;
  }

  rclcpp::shutdown();

  tactic.reset();
  callback.reset();
  pipeline.reset();
  pipeline_factory.reset();
  CLOG(WARNING, "test") << "Saving pose graph and reset.";
  graph->save();
  graph.reset();
  CLOG(WARNING, "test") << "Saving pose graph and reset. - DONE!";
}