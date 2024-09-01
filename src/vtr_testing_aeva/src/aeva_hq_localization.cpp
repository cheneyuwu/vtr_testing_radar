#include <filesystem>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

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

#include "vtr_testing_aeva/utils.hpp"

#include "yaml-cpp/yaml.h"

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

int64_t utc_to_nanos_of_week(int64_t times_sec) {
    const int64_t NS_PER_SEC = 1e9;
    const double SECONDS_PER_WEEK = 60.0 * 60.0 * 24.0 * 7.0;

    int64_t gps_sec = times_sec - 315964800 + 18;
    int64_t gps_ns = gps_sec * NS_PER_SEC;
    double ns_per_week = SECONDS_PER_WEEK * NS_PER_SEC;

    return static_cast<double>(fmod(gps_ns, ns_per_week));
}

Eigen::Vector3d polarToXYZ(double range, double azimuth, double elevation) {
  double xy = range * cos(elevation);
  return Eigen::Vector3d(xy * cos(azimuth), xy * sin(azimuth), range * sin(elevation));
}

std::pair<int64_t, Eigen::MatrixXd> load_lidar(const std::string &path, double time_delta_sec, int sensor_id, double start_time, double end_time, int64_t filename) {
  std::ifstream ifs(path, std::ios::binary);
  std::vector<char> buffer(std::istreambuf_iterator<char>(ifs), {});
  unsigned float_offset = 4; // float32
  unsigned fields = 11;  // x, y, z, radial velocity, intensity, signal quality, reflectivity, time, beam_id, line_id, face_id
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
    int beam_id, line_id, face_id;

    // x, y, z
    x = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    y = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    z = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);

    // Calculate range, azimuth, and elevation
    double range = sqrt(x * x + y * y + z * z);
    double azimuth = atan2(y, x);
    double xy = sqrt(x * x + y * y);
    double elevation = atan2(z, xy);

    // Apply sensor-specific scaling
    Eigen::Vector3d new_xyz;
    if (sensor_id == 0)
      new_xyz = polarToXYZ(range * 1.0, azimuth * 0.99816, elevation);
    else if (sensor_id == 1)
      new_xyz = polarToXYZ(range * 1.00087107, azimuth * 1.00873727, elevation);
    else if (sensor_id == 2)
      new_xyz = polarToXYZ(range * 1.00272944, azimuth * 1.00761913, elevation);
    else if (sensor_id == 3)
      new_xyz = polarToXYZ(range * 1.00369723, azimuth * 1.00797442, elevation);

    x = new_xyz.x();
    y = new_xyz.y();
    z = new_xyz.z();

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
    time_temp = time_temp * 1e-9 + time_delta_sec;
    ++offset;
    // Beam id
    beam_id = (int)(getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset));
    ++offset;
    // Line id
    line_id = 63 - (int)(getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset)); // Flip order
    ++offset;
    // Face id of polygon mirror (available on aeries II)
    face_id = (int)(getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset));

    // Error checks
    if (line_id < 0 || line_id >= 64) continue;
    if (face_id < 0 || face_id > 5) continue;
    if (sensor_id < 0 || sensor_id > 4) continue;

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

EdgeTransform load_T_lidar_robot(std::vector<std::vector<double>>& xi_sv) {
  std::vector<EdgeTransform> T_lidar_robot;

  for (const auto& xi_sv: xi_sv) {
    const auto T_temp = lgmath::se3::Transformation(Eigen::Matrix<double, 6, 1>(xi_sv.data()));
    EdgeTransform T_lidar_vehicle_mat(T_temp, Eigen::Matrix<double, 6, 6>::Zero());

    T_lidar_robot.push_back(T_lidar_vehicle_mat);
  } 

  return T_lidar_robot[3];
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
  if (dataset != "aeva_boreas" && dataset != "aeva_hq") 
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
        }
        else if (dataset == "aeva_boreas") {  // Note: r and p are flipped for aeva_boreas
          if (i == 2) 
            r = std::stod(value);
          else if (i == 1)
            p = std::stod(value);
          else if (i == 3)
            y = std::stod(value);
        }
        else /* if (dataset == "aeva_hq") */ {
          if (i == 4)
            r = std::stod(value);
          else if (i == 5)
            p = std::stod(value);
          else if (i == 6)
            y = std::stod(value);
        }
      } // end for row
      // std::cout << timestamp_sec << ", " << r << ", " << p << ", " << y << std::endl;
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

bool sync_frames(std::vector<std::vector<std::string>>& filenames, int (&init_frame)[4]) {
  // compare times and find min
  int64_t time_micro[4];
  bool eq_flag = true;  // results in true if all frames are equal to each other
  int64_t min_time = 0; int min_id = 0;
  int64_t tol = 0.015 * 1e6;  // 0.015 seconds to microseconds
  for (int i = 0; i < 4; ++i) {
    std::string& filename = filenames[i][init_frame[i]];
    time_micro[i] = std::stoll(filename.substr(0, filename.find("."))); // string to int
    if (i == 0) {
      min_time = time_micro[0];
      min_id = 0;
      continue;
    }

    // compare i to 0 and update min
    eq_flag = eq_flag && (std::abs(time_micro[i] - time_micro[0]) < tol);
    if (time_micro[i] < min_time) {
      min_time = time_micro[i];
      min_id = i;
    }
  }

  if (eq_flag)
    return false; // exit while loop
  else {
    ++init_frame[min_id];  // increment smallest frame by 1
    return true;  // continue while loop
  }
}

double getOdoInitTime(const std::string dir_odo[4]) {
  std::vector<std::vector<std::string>> filenames_odo;
  int init_frame_odo[4] = {0};
  int curr_frame_odo[4] = {0};
  int last_frame_odo[4] = {std::numeric_limits<int>::max()};  // exclusive bound
  
  // get filenames for each sensor (4 sensors total)
  for (int i = 0; i < 4; ++i) {
    filenames_odo.push_back(std::vector<std::string>());
    auto dir_iter = std::filesystem::directory_iterator(dir_odo[i]);
    last_frame_odo[i] = std::count_if(begin(dir_iter), end(dir_iter), [&](auto &entry) {
      if (entry.is_regular_file()) filenames_odo[i].emplace_back(entry.path().filename().string());
      return entry.is_regular_file();
    });
    std::sort(filenames_odo[i].begin(), filenames_odo[i].end(), filecomp);  // custom comparison
  }

  // the sensor frames are synchronized, but may have an extra frame or two at the start which we need to ignore
  // set init_frame_[x] for each sensor so they all start at the same time
  while (sync_frames(filenames_odo, init_frame_odo)) {};

  // set initial time to keep floats small
  double initial_timestamp_micro = std::stoll(filenames_odo[0][init_frame_odo[0]].substr(0, filenames_odo[0][init_frame_odo[0]].find(".")));
  CLOG(WARNING, "test") << initial_timestamp_micro;
  return initial_timestamp_micro;
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

std::tuple<std::vector<Eigen::Matrix4d>, std::vector<double>, std::vector<Eigen::VectorXd>> csvToSe3Poses(const std::string& filename) {
    std::vector<Eigen::Matrix4d> poses;
    std::vector<double> times_gt;
    std::vector<Eigen::VectorXd> velocities;

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return std::make_tuple(poses, times_gt, velocities);
    }

    std::string line;
    std::getline(file, line); // Skip header line

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::vector<std::string> fields;
        std::string field;

        while (std::getline(ss, field, ',')) {
            fields.push_back(field);
        }

        if (fields.size() < 18) { // Ensure the line has enough columns
            std::cerr << "Unexpected number of columns in line: " << line << std::endl;
            continue;
        }
        
        // poses
        double easting = std::stod(fields[1]);
        double northing = std::stod(fields[2]);
        double altitude = std::stod(fields[3]);
        double roll = std::stod(fields[7]);
        double pitch = std::stod(fields[8]);
        double heading = std::stod(fields[9]);

        Eigen::Matrix4d Ti = Eigen::Matrix4d::Identity();
        Ti.block<3, 3>(0, 0) = rpy2rot(roll, pitch, heading);
        Ti(0, 3) = easting;
        Ti(1, 3) = northing;
        Ti(2, 3) = altitude;

        poses.push_back(Ti);

        // times
        times_gt.push_back(std::stoll(fields[0]));

        // velocities
        double vel_east = std::stod(fields[4]);
        double vel_north = std::stod(fields[5]);
        double vel_up = std::stod(fields[6]);
        double angvel_x = std::stod(fields[10]);
        double angvel_y = std::stod(fields[11]);
        double angvel_z = std::stod(fields[12]);

        Eigen::Vector3d v_inertial(vel_east, vel_north, vel_up);
        Eigen::Vector3d w_body(angvel_x, angvel_y, angvel_z);

        // Transform velocity from inertial frame to body frame
        Eigen::Vector3d v_body = Ti.block<3, 3>(0, 0).transpose() * v_inertial;
        Eigen::VectorXd velocity(6);
        velocity.head<3>() = v_body;
        velocity.tail<3>() = w_body;

        velocities.push_back(velocity);
    }

    file.close();
    return std::make_tuple(poses, times_gt, velocities);
}


Eigen::Matrix4d wnoa_interp(double time1, const Eigen::Matrix4d& T1, const Eigen::VectorXd& w1, double time2, const Eigen::Matrix4d& T2, const Eigen::VectorXd& w2, double timeq) {
    double tau = timeq - time1;
    double dt = time2 - time1;
    double ratio = tau / dt;
    double ratio2 = ratio * ratio;
    double ratio3 = ratio2 * ratio;

    double psi11 = 3.0 * ratio2 - 2.0 * ratio3;
    double psi12 = tau * (ratio2 - ratio);
    double psi21 = 6.0 * (ratio - ratio2) / dt;
    double psi22 = 3.0 * ratio2 - 2.0 * ratio;

    double lambda11 = 1.0 - psi11;
    double lambda12 = tau - dt * psi11 - psi12;
    double lambda21 = -psi21;
    double lambda22 = 1.0 - dt * psi21 - psi22;

    Eigen::VectorXd xi_21 = lgmath::se3::tran2vec(T2 * (T1).inverse());
    Eigen::MatrixXd J_21_inv = lgmath::se3::vec2jacinv(xi_21);
    Eigen::VectorXd xi_i1 = lambda12 * w1 + psi11 * xi_21 + psi12 * J_21_inv * w2;

    Eigen::Matrix4d T_i1 = lgmath::se3::vec2tran(xi_i1);
    return T_i1 * T1;
}

void wnoa_interp_traj(const std::vector<double>& src_times, const std::vector<Eigen::Matrix4d>& T, const std::vector<Eigen::VectorXd>& w, const double& query_time, std::vector<Eigen::Matrix4d>& outT_list) {
    int tail = 0;
    int head = 1;
    while (!(src_times[tail] <= query_time && query_time <= src_times[head])) {
        ++tail;
        ++head;
    }
    Eigen::Matrix4d Tinterp = wnoa_interp(src_times[tail], T[tail], w[tail], src_times[head], T[head], w[head], query_time);
    outT_list.push_back(Tinterp);
}


EdgeTransform load_T_robot_enu_init(const std::string &path, const double start_time_sec) {
  std::string filename = path + "/processed_sbet.csv";

  Eigen::Matrix4d T_r_app;
  Eigen::Matrix4d T_ref;
  T_r_app << 9.99960818e-01,-1.40913767e-03, 8.73943838e-03, 0.00000000e+00,
             1.40913767e-03, 9.99999007e-01, 6.15750237e-06, 0.00000000e+00,
            -8.73943838e-03, 6.15781076e-06, 9.99961810e-01, 0.00000000e+00,
             0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00;
  T_ref << 1, 0, 0, 0, 
           0,-1, 0, 0, 
           0, 0,-1, 0, 
           0, 0, 0, 1;

  auto [poses, times_gt, vels] = csvToSe3Poses(filename);
  double query_time = utc_to_nanos_of_week(start_time_sec) * 1e-9;

  std::vector<Eigen::VectorXd> varpi_gt;
  for (const auto& vel : vels) {
    varpi_gt.push_back(lgmath::se3::tranAd(T_ref) * vel);
  }

  std::vector<Eigen::Matrix4d> raw_Tiv;
  raw_Tiv.reserve(poses.size());
  for (const auto& pose : poses) {
    raw_Tiv.push_back(
      (pose * T_ref.inverse()).inverse()
      );
  }

  std::vector<Eigen::Matrix4d> T_gt;
  wnoa_interp_traj(times_gt, raw_Tiv, varpi_gt, query_time, T_gt);

  std::cout << T_gt[0] << std::endl;

  Eigen::Matrix4d T_mat = T_r_app * (T_gt[0]).inverse(); // T_vi

  std::cout << std::setprecision(20) << start_time_sec * 1e6 << std::endl;
  std::cout << T_mat << std::endl;

  EdgeTransform T(T_mat);
  T.setZeroCovariance();

  return T;
}

int main(int argc, char **argv) {
  // disable eigen multi-threading
  Eigen::setNbThreads(1);

  // To do: combine with aeva_hq.yaml! don't want 2 configs
  std::string yaml_file_path = "external/vtr_testing_radar/src/vtr_testing_aeva/config/aeva_hq_dataset_config.yaml";
  YAML::Node config = loadYamlFile(yaml_file_path);

  // load options
  std::vector<bool> active_lidars = config["dataset_options"]["active_lidars"].as<std::vector<bool>>();
  std::vector<bool> active_gyros = config["dataset_options"]["active_gyros"].as<std::vector<bool>>();
  std::vector<std::vector<double>> xi_sv = config["dataset_options"]["xi_sv"].as<std::vector<std::vector<double>>>();
  std::vector<std::vector<double>> gyro_ivar = config["dataset_options"]["gyro_ivar"].as<std::vector<std::vector<double>>>();
  int nframes_gbias_calib= config["dataset_options"]["nframes_gbias_calib"].as<int>();

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
                        << (stem + "_lidar_localization");
  const auto status_publisher = node->create_publisher<std_msgs::msg::String>(
      stem + "_lidar_localization", 1);

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

  // Frame and transforms
  std::string robot_frame = "robot";
  std::string lidar_frame = "lidar";
  
  const auto T_lidar_robot = load_T_lidar_robot(xi_sv);
  CLOG(WARNING, "test") << "Transform from " << robot_frame << " to "
                        << lidar_frame << " has been set to" << T_lidar_robot;

  // handle 4 sensors
  std::string dir_path_[4];
  std::vector<std::vector<std::string>> filenames_;
  int64_t initial_timestamp_micro_;
  int init_frame_[4] = {0};
  int curr_frame_[4] = {0};
  int last_frame_[4] = {std::numeric_limits<int>::max()};  // exclusive bound

  // we will always index in this order:
  // 0: front-facing sensor, 1: left-facing sensor, 2: right-facing sensor, 3: back-facing sensor
  std::string lnames[4];  // lidar names
  lnames[0] = "front_4320";
  lnames[1] = "left_4386";
  lnames[2] = "right_4347";
  lnames[3] = "back_4363";

  for (uint i = 0; i < 4; ++i) {
    CLOG(WARNING, "test") << loc_dir.string() + "/" + lnames[i] + "/";
    dir_path_[i] = loc_dir.string() + "/" + lnames[i] + "/";
  }

  std::string dir_odo[4];
  for (uint i = 0; i < 4; ++i) {
    CLOG(WARNING, "test") << odo_dir.string() + "/" + lnames[i] + "/";
    dir_odo[i] = odo_dir.string() + "/" + lnames[i] + "/";
  }

  double initial_timestamp_micro_odo = getOdoInitTime(dir_odo);

  // get filenames for each sensor (4 sensors total)
  for (int i = 0; i < 4; ++i) {
    filenames_.push_back(std::vector<std::string>());
    auto dir_iter = std::filesystem::directory_iterator(dir_path_[i]);
    last_frame_[i] = std::count_if(begin(dir_iter), end(dir_iter), [&](auto &entry) {
      if (entry.is_regular_file()) filenames_[i].emplace_back(entry.path().filename().string());
      return entry.is_regular_file();
    });
    std::sort(filenames_[i].begin(), filenames_[i].end(), filecomp);  // custom comparison
  }

  // the sensor frames are synchronized, but may have an extra frame or two at the start which we need to ignore
  // set init_frame_[x] for each sensor so they all start at the same time
  while (sync_frames(filenames_, init_frame_)) {};

  // determine min length to truncate end
  int len = last_frame_[0] - init_frame_[0];
  for (int i = 1; i < 4; ++i) {
    if (last_frame_[i] - init_frame_[i] < len)
      len = last_frame_[i] - init_frame_[i];
  }

  // initialize curr_frame_ for each sensor and make sure lengths are the same 
  for (int i = 0; i < 4; ++i) {
    curr_frame_[i] = init_frame_[i] + std::max((int)0, 0); // hardcoded init frame 0
    last_frame_[i] = init_frame_[i] + len;
  }  

  // set initial time to keep floats small
  initial_timestamp_micro_ = std::stoll(filenames_[0][init_frame_[0]].substr(0, filenames_[0][init_frame_[0]].find(".")));
  CLOG(WARNING, "test") << initial_timestamp_micro_;

  /// NOTE: odometry is teach, localization is repeat
  auto T_loc_odo_init = [&]() {
    const auto T_robot_enu_odo = load_T_robot_enu_init(odo_dir.string(), (double)(initial_timestamp_micro_odo) / 1e6); // T_vi
    const auto T_robot_enu_loc = load_T_robot_enu_init(loc_dir.string(), initial_timestamp_micro_ / 1e6);    // T_vi

    const auto T_lidar_robot = load_T_lidar_robot(xi_sv); // T_s_v of back sensor (best)

    // return T_lidar_robot.inverse() * T_enu_lidar_loc.inverse() * T_enu_lidar_odo * T_lidar_robot; 
    return T_robot_enu_loc * T_robot_enu_odo.inverse();
  }();
  T_loc_odo_init.setCovariance(Eigen::Matrix<double, 6, 6>::Identity());
  CLOG(WARNING, "test")
      << "Transform from localization to odometry has been set to "
      << T_loc_odo_init.vec().transpose();

  tactic->setPath(sequence, /* trunk sid */ 0, T_loc_odo_init, true);


  // Gyroscope data
  std::vector<Eigen::MatrixXd> gyro_data_;
  std::vector<Eigen::Vector3d> const_gyro_bias;
  for (size_t i = 0; i < 4; ++i) {
    std::string gyro_path = loc_dir.string() + "/" + lnames[i] + "_imu.csv";
    gyro_data_.push_back(readGyroToEigenXd(gyro_path, initial_timestamp_micro_, "aeva_hq"));
    gyro_data_.back().rightCols<3>() *= -1.0; // flip reference frame
    LOG(INFO) << "Loaded gyro data " << ". Matrix " 
        << gyro_data_.back().rows() << " x " << gyro_data_.back().cols() << std::endl;

    // calibrate gyro bias using first N measurements while stationary
    const_gyro_bias.push_back(gyro_data_.back().topRightCorner(nframes_gbias_calib, 3).colwise().mean().transpose());
  }

  // initialize gyro
  std::vector<Eigen::Matrix3d> gyro_invcov;

  for (const auto& gyro_ivar: gyro_ivar) {
    Eigen::Matrix3d temp = Eigen::Matrix3d::Zero();
    temp.diagonal() = Eigen::Vector3d(gyro_ivar.data());
    gyro_invcov.push_back(temp);
  }                       

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
  for (const auto &dir_entry : fs::directory_iterator{loc_dir / lnames[3]})
    if (!fs::is_directory(dir_entry)) files.push_back(dir_entry);
  std::sort(files.begin(), files.end());
  CLOG(WARNING, "test") << "Found " << files.size() << " lidar data";

  // thread handling variables
  TestControl test_control(node);

  // main loop
  int frame = 0;
  int last_frame = 1000000;
  auto it = files.begin();
  
  while (it != files.end()) {
    if (!rclcpp::ok()) break;
    rclcpp::spin_some(node);
    if (test_control.terminate()) break;
    if (!test_control.play()) continue;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(test_control.delay()));

    // Use sensor 0 for start/end time
    // Note: we peak into future data for the end timestamp for evaluation convenience. An online implementation
    // would need different logic, i.e., use the last timestamp of the pointcloud
    int tsensorid = 0;
    int tframeid = curr_frame_[tsensorid];
    auto& start_name = filenames_[tsensorid].at(tframeid);  // time in microseconds as string
    auto start_time = static_cast<double>(std::stoll(start_name.substr(0, start_name.find("."))) - initial_timestamp_micro_) / 1e6;  // seconds
    auto end_time = start_time + 0.1;  // this will occur at the last frame, but we know it will approximately be 0.1 seconds
    auto end_name = start_name;
    if (tframeid + 1 < filenames_[tsensorid].size()) {
      end_name = filenames_[tsensorid].at(tframeid + 1);
      end_time = static_cast<double>(std::stoll(end_name.substr(0, end_name.find("."))) - initial_timestamp_micro_) / 1e6;
    }

    // load active sensors
    Eigen::MatrixXd output_frame;
    int total_rows = 0;
    int total_cols = 0;
    for (int sensor_id = 0; sensor_id < 4; ++sensor_id) {
      // frame_id, filename, and start time
      int curr_frame = curr_frame_[sensor_id]++;  // grab frame id and increment after
      auto& filename = filenames_[sensor_id].at(curr_frame);
      int64_t time_delta_micro = std::stoll(filename.substr(0, filename.find("."))) - initial_timestamp_micro_;
      double time_delta_sec = static_cast<double>(time_delta_micro) / 1e6;

      int64_t timestamp = (std::stoll(start_name.substr(0, start_name.find(".")))) * 1000;
      int64_t next_state_time;
      if (end_name == start_name) {
        next_state_time = ((std::stoll(start_name.substr(0, start_name.find(".")))) + 100000) * 1000;
      } else {
        next_state_time = (std::stoll(end_name.substr(0, end_name.find(".")))) * 1000;
      }

      // skip if inactive
      if (active_lidars[sensor_id] == false)
        continue;

      // load and concatenate
      const auto [time, frame] = load_lidar(dir_path_[sensor_id] + filename, time_delta_sec, sensor_id, start_time, end_time, timestamp);

      if (output_frame.size() == 0) {
        // initialize the output_frame to the same column size as frame, but 0 rows initially
        total_cols = frame.cols();
        output_frame.resize(0, total_cols);
      }

      // resize output_frame to accommodate the new frame's rows
      int frame_rows = frame.rows();
      output_frame.conservativeResize(total_rows + frame_rows, total_cols);

      // copy new frame data into the output_frame
      output_frame.block(total_rows, 0, frame_rows, total_cols) = frame;
      total_rows += frame_rows;

    }
    // loop for each sensor
    std::vector<Eigen::MatrixXd> current_gyro;
    for (int sensorid = 0; sensorid < gyro_data_.size(); ++sensorid) {
      if (active_gyros[sensorid] != true) {    // inactive gyro
        current_gyro.push_back(Eigen::MatrixXd(0, 0));  // empty matrix
        continue;
      }

      // find indices for data between start and end times
      std::vector<int> inds; inds.clear();
      for (int r = 0; r < gyro_data_[sensorid].rows(); ++r) {
        double meas_time = gyro_data_[sensorid](r, 0);
        if (meas_time > start_time && meas_time <= end_time)
          inds.push_back(r);
      } // end for r

      if (inds.size() == 0) {   // no measurements
        current_gyro.push_back(Eigen::MatrixXd(0, 0));  // empty matrix
        CLOG(WARNING, "test") << "grabbing gyro " << sensorid << ", no gyro data" << std::endl;
        continue;
      }

      // output
      Eigen::MatrixXd temp_gyro(inds.size(), 4);
      for (int r = 0; r < inds.size(); ++r) {
        temp_gyro(r, 0) = gyro_data_[sensorid](inds[r], 0); // timestamp
        temp_gyro.row(r).rightCols<3>() = gyro_data_[sensorid].row(inds[r]).rightCols<3>() 
            - const_gyro_bias[sensorid].transpose(); // measurement w/ bias comp.
      }
      current_gyro.push_back(temp_gyro);
      CLOG(WARNING, "test") << "grabbing gyro " << sensorid << ", " << current_gyro.back().rows() << " x " << current_gyro.back().cols() 
        << ". Start time: " << current_gyro.back()(0, 0) << ", " << "end time: " << current_gyro.back()(inds.size()-1, 0) << std::endl;
    } // end for sensorid
      
    if (current_gyro.size() != 4)
      throw std::runtime_error("Invalid number of gyro sensors"); 

    int64_t timestamp = (std::stoll(start_name.substr(0, start_name.find(".")))) * 1000;
    int64_t next_state_time;
    if (end_name == start_name) {
      next_state_time = ((std::stoll(start_name.substr(0, start_name.find(".")))) + 100000) * 1000;
    } else {
      next_state_time = (std::stoll(end_name.substr(0, end_name.find(".")))) * 1000;
    }

    CLOG(WARNING, "test") << "Loading lidar frame " << frame
                          << " with timestamp " << timestamp;

    // publish clock for sim time
    auto time_msg = rosgraph_msgs::msg::Clock();
    time_msg.clock = rclcpp::Time(timestamp);
    clock_publisher->publish(time_msg);

    // Convert message to query_data format and store into query_data
    auto query_data = std::make_shared<lidar::LidarQueryCache>();

    // some modules require node for visualization
    query_data->node = node;

    // set timestamp
    query_data->stamp.emplace(timestamp);

    // make up some environment info (not important)
    tactic::EnvInfo env_info;
    env_info.terrain_type = 0;
    query_data->env_info.emplace(env_info);

    // set lidar frame
    query_data->points.emplace(std::move(output_frame));

    // fill in the vehicle to sensor transform and frame name
    query_data->T_s_r.emplace(T_lidar_robot);

    // set gyro data
    query_data->gyro.emplace(current_gyro);
    query_data->gyro_invcov.emplace(gyro_invcov);
    query_data->const_gyro_bias.emplace(const_gyro_bias);

    // set timestamp of first frame [us]
    query_data->initial_timestamp.emplace(initial_timestamp_micro_);

    // set timestamp of next state [ns]
    query_data->next_state_time.emplace(next_state_time);

    // execute the pipeline
    tactic->input(query_data);

    std_msgs::msg::String status_msg;
    status_msg.data = "Finished processing lidar frame " +
                      std::to_string(frame) + " with timestamp " +
                      std::to_string(timestamp);
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