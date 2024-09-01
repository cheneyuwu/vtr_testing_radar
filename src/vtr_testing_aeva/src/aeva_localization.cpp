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

#include "vtr_testing_aeva/utils.hpp"

namespace fs = std::filesystem;
using namespace vtr;
using namespace vtr::common;
using namespace vtr::logging;
using namespace vtr::tactic;
using namespace vtr::testing;

float getFloatFromByteArray(char *byteArray, uint index) {
  return *((float *)(byteArray + index));
}

int64_t getStampFromPath(const std::string &path) {
  std::vector<std::string> parts;
  boost::split(parts, path, boost::is_any_of("/"));
  std::string stem = parts[parts.size() - 1];
  boost::split(parts, stem, boost::is_any_of("."));
  int64_t time1 = std::stoll(parts[0]);
  return time1 * 1000;
}

std::pair<int64_t, Eigen::MatrixXd> load_lidar(const std::string &path) {
  std::ifstream ifs(path, std::ios::binary);
  std::vector<char> buffer(std::istreambuf_iterator<char>(ifs), {});
  uint float_offset = 4; // float32 
  uint fields = 7;  // x, y, z, i, r, t, b_id
  uint point_step = float_offset * fields;
  uint N = floor(buffer.size() / point_step);
  Eigen::MatrixXd pc(Eigen::MatrixXd::Ones(N, fields));
  for (uint i = 0; i < N; ++i) {
    uint bufpos = i * point_step;
    for (uint j = 0; j < fields; ++j) {
      pc(i, j) =
          getFloatFromByteArray(buffer.data(), bufpos + j * float_offset);
    }
  }
  // Add offset to timestamps
  const auto timestamp = getStampFromPath(path);
  double t = double(timestamp / 1000) * 1.0e-6;
  pc.block(0, 5, N, 1).array() += t;

  return std::make_pair(timestamp, std::move(pc));
}

EdgeTransform load_T_robot_lidar(const fs::path &path) {
  std::ifstream ifs(path / "calib" / "T_applanix_aeva.txt", std::ios::in);

  Eigen::Matrix4d T_applanix_lidar_mat;
  for (size_t row = 0; row < 4; row++)
    for (size_t col = 0; col < 4; col++) ifs >> T_applanix_lidar_mat(row, col);

  Eigen::Matrix4d yfwd2xfwd;
  yfwd2xfwd << 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

  EdgeTransform T_robot_lidar(Eigen::Matrix4d(yfwd2xfwd * T_applanix_lidar_mat),
                              Eigen::Matrix<double, 6, 6>::Zero());

  return T_robot_lidar;
}

EdgeTransform load_T_lidar_robot() {
  Eigen::Matrix4d T_lidar_vehicle_mat;
  T_lidar_vehicle_mat << 0.9999366830849237  ,  0.008341717781538466  ,  0.0075534496251198685, -1.0119098938516395 ,
                        -0.008341717774127972,  0.9999652112886684    , -3.150635091210066e-05, -0.39658824335171944,
                        -0.007553449599178521, -3.1504388681967066e-05,  0.9999714717963843   , -1.697000000000001  ,
                         0                   ,  0                     ,  0                    ,  1                  ;
  
  EdgeTransform T_lidar_robot(T_lidar_vehicle_mat,
                              Eigen::Matrix<double, 6, 6>::Zero());
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

Eigen::MatrixXd readBoreasGyroToEigenXd(const std::string &file_path, const int64_t& initial_timestamp_micro) {
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
      for (int i = 0; i < 4; ++i) {
        std::string value;
        std::getline(ss, value, ',');

        if (i == 0) {
          timestamp = std::stol(value);
          timestamp_sec = static_cast<double>(timestamp - initial_timestamp_micro)*1e-6;
        }
        else if (i == 1)
          r = std::stod(value);
        else if (i == 2)
          p = std::stod(value);
        else if (i == 3)
          y = std::stod(value);
      } // end for row
      row_vec[0] = timestamp_sec;
      row_vec[1] = p;
      row_vec[2] = r;
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

std::pair<int64_t, std::vector<Eigen::MatrixXd>> load_gyro(const std::string &path) {
  std::vector<std::string> filenames_;
  int64_t initial_timestamp_micro_;

  std::string dir_path_ = path + "/aeva/";
  auto dir_iter = std::filesystem::directory_iterator(dir_path_);
  std::count_if(begin(dir_iter), end(dir_iter), [&filenames_](auto &entry) {
    if (entry.is_regular_file()) filenames_.emplace_back(entry.path().filename().string());
    return entry.is_regular_file();
  });
  std::sort(filenames_.begin(), filenames_.end(), filecomp);  // custom comparison

  initial_timestamp_micro_ = std::stoll(filenames_[0].substr(0, filenames_[0].find(".")));

  std::vector<Eigen::MatrixXd> gyro_data;
  gyro_data.clear();
  std::string gyro_path = path + "/applanix/" + "aeva_imu.csv";
  gyro_data.push_back(readBoreasGyroToEigenXd(gyro_path, initial_timestamp_micro_));
  CLOG(WARNING, "test") << "Loaded gyro data " << ". Matrix " 
      << gyro_data.back().rows() << " x " << gyro_data.back().cols() << std::endl;

  return std::make_pair(initial_timestamp_micro_, gyro_data);
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

  /// NOTE: odometry is teach, localization is repeat
  auto T_loc_odo_init = [&]() {
    const auto T_enu_lidar_odo = load_T_enu_lidar_init(odo_dir);
    const auto T_enu_lidar_loc = load_T_enu_lidar_init(loc_dir);

    const auto T_lidar_robot = load_T_lidar_robot(); // T_s_v

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
  
  const auto T_lidar_robot = load_T_lidar_robot();
  CLOG(WARNING, "test") << "Transform from " << robot_frame << " to "
                        << lidar_frame << " has been set to" << T_lidar_robot;

  // Gyroscope data
  const auto [initial_timestamp_micro_, gyro] = load_gyro(loc_dir);                        

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
  while (it != files.end()) {
    if (!rclcpp::ok()) break;
    rclcpp::spin_some(node);
    if (test_control.terminate()) break;
    if (!test_control.play()) continue;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(test_control.delay()));

    ///
    const auto [timestamp, points] = load_lidar(it->path().string());

    CLOG(WARNING, "test") << "Loading lidar frame " << frame
                          << " with timestamp " << timestamp;

    // Get the name of the next file
    int64_t next_file_name;
    if ((it + 1) != files.end()) {
      next_file_name = getStampFromPath((it + 1)->path().string());
    }

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
    query_data->points.emplace(std::move(points));

    // fill in the vehicle to sensor transform and frame name
    query_data->T_s_r.emplace(T_lidar_robot);

    // set gyro data
    query_data->gyro.emplace(gyro);

    // set timestamp of first frame [us]
    query_data->initial_timestamp.emplace(initial_timestamp_micro_);

    // set timestamp of next state [ns]
    query_data->next_state_time.emplace(next_file_name);

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