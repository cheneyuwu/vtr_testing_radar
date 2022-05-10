import os
import os.path as osp
import sys
import time
import numpy as np
import numpy.linalg as npla
import scipy.spatial.transform as sptf

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.time_source import CLOCK_TOPIC
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs
import nav_msgs.msg as nav_msgs
import rosgraph_msgs.msg as rosgraph_msgs

np.set_printoptions(precision=6, suppress=True)


def get_T_v0_v1_theta(theta):
  return np.array(
      [[np.cos(theta), -np.sin(theta), 0, 0], [np.sin(theta), np.cos(theta), 0, 0], [0., 0., 1., 0.], [0., 0., 0., 1.]],
      dtype=np.float64)


def pose2tfstamped(pose, stamp, to_frame, from_frame):
  tran = pose[:3, 3]
  rot = sptf.Rotation.from_matrix(pose[:3, :3]).as_quat()

  tfs = geometry_msgs.TransformStamped()
  # The default (fixed) frame in RViz is called 'world'
  tfs.header.frame_id = to_frame
  tfs.header.stamp = stamp
  tfs.child_frame_id = from_frame
  tfs.transform.translation.x = tran[0]
  tfs.transform.translation.y = tran[1]
  tfs.transform.translation.z = tran[2]
  tfs.transform.rotation.x = rot[0]
  tfs.transform.rotation.y = rot[1]
  tfs.transform.rotation.z = rot[2]
  tfs.transform.rotation.w = rot[3]
  return tfs


def roll(r):
  return np.array([[1, 0, 0], [0, np.cos(r), np.sin(r)], [0, -np.sin(r), np.cos(r)]], dtype=np.float64)


def pitch(p):
  return np.array([[np.cos(p), 0, -np.sin(p)], [0, 1, 0], [np.sin(p), 0, np.cos(p)]], dtype=np.float64)


def yaw(y):
  return np.array([[np.cos(y), np.sin(y), 0], [-np.sin(y), np.cos(y), 0], [0, 0, 1]], dtype=np.float64)


def yawPitchRollToRot(y, p, r):
  return roll(r) @ pitch(p) @ yaw(y)


def get_inverse_tf(T):
  """Returns the inverse of a given 4x4 homogeneous transform.
    Args:
        T (np.ndarray): 4x4 transformation matrix
    Returns:
        np.ndarray: inv(T)
    """
  T2 = T.copy()
  T2[:3, :3] = T2[:3, :3].transpose()
  T2[:3, 3:] = -1 * T2[:3, :3] @ T2[:3, 3:]
  return T2


def convert_line_to_pose(line, dim=3):
  """Reads trajectory from list of strings (single row of the comma-separeted groundtruth file). See Boreas
    documentation for format
    Args:
        line (List[string]): list of strings
        dim (int): dimension for evaluation. Set to '3' for 3D or '2' for 2D
    Returns:
        (np.ndarray): 4x4 SE(3) pose
        (int): time in nanoseconds
    """
  # returns T_iv
  line = line.replace('\n', ',').split(',')
  line = [float(i) for i in line[:-1]]
  # x, y, z -> 1, 2, 3
  # roll, pitch, yaw -> 7, 8, 9
  T = np.eye(4, dtype=np.float64)
  T[0, 3] = line[1]  # x
  T[1, 3] = line[2]  # y
  if dim == 3:
    T[2, 3] = line[3]  # z
    T[:3, :3] = yawPitchRollToRot(line[9], line[8], line[7])
  elif dim == 2:
    T[2, 3] = 0
    T[:3, :3] = yawPitchRollToRot(line[9], np.round(line[8] / np.pi) * np.pi, np.round(line[7] / np.pi) * np.pi)
  else:
    raise ValueError('Invalid dim value in convert_line_to_pose. Use either 2 or 3.')
  time = int(line[0])
  return T, time


def enforce_orthog(T, dim=3):
  """Enforces orthogonality of a 3x3 rotation matrix within a 4x4 homogeneous transformation matrix.
    Args:
        T (np.ndarray): 4x4 transformation matrix
        dim (int): dimensionality of the transform 2==2D, 3==3D
    Returns:
        np.ndarray: 4x4 transformation matrix with orthogonality conditions on the rotation matrix enforced.
    """
  if dim == 2:
    if abs(np.linalg.det(T[0:2, 0:2]) - 1) < 1e-10:
      return T
    R = T[0:2, 0:2]
    epsilon = 0.001
    if abs(R[0, 0] - R[1, 1]) > epsilon or abs(R[1, 0] + R[0, 1]) > epsilon:
      print("WARNING: this is not a proper rigid transformation:", R)
      return T
    a = (R[0, 0] + R[1, 1]) / 2
    b = (-R[1, 0] + R[0, 1]) / 2
    s = np.sqrt(a**2 + b**2)
    a /= s
    b /= s
    R[0, 0] = a
    R[0, 1] = b
    R[1, 0] = -b
    R[1, 1] = a
    T[0:2, 0:2] = R
  if dim == 3:
    if abs(np.linalg.det(T[0:3, 0:3]) - 1) < 1e-10:
      return T
    c1 = T[0:3, 1]
    c2 = T[0:3, 2]
    c1 /= np.linalg.norm(c1)
    c2 /= np.linalg.norm(c2)
    newcol0 = np.cross(c1, c2)
    newcol1 = np.cross(c2, newcol0)
    T[0:3, 0] = newcol0
    T[0:3, 1] = newcol1
    T[0:3, 2] = c2
  return T


def read_traj_file_gt(path, T_ab, dim):
  """Reads trajectory from a comma-separated file, see Boreas documentation for format
    Args:
        path (string): file path including file name
        T_ab (np.ndarray): 4x4 transformation matrix for calibration. Poses read are in frame 'b', output in frame 'a'
        dim (int): dimension for evaluation. Set to '3' for 3D or '2' for 2D
    Returns:
        (List[np.ndarray]): list of 4x4 poses
        (List[int]): list of times in microseconds
    """
  with open(path, 'r') as f:
    lines = f.readlines()
  poses = []
  times = []

  T_ab = enforce_orthog(T_ab)
  for line in lines[1:]:
    pose, time = convert_line_to_pose(line, dim)
    # if dim == 2:
    #   # pose[1, 3] = -pose[1, 3]
    #   # T_zdown_zup = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]], dtype=np.float64)
    #   T_zdown_zup = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=np.float64)
    #   pose[:3, :3] = T_zdown_zup @ pose[:3, :3]
    poses += [enforce_orthog(T_ab @ get_inverse_tf(pose))]  # convert T_iv to T_vi and apply calibration
    times += [int(time)]  # microseconds
  return poses, times


class BoreasPlayer(Node):

  def __init__(self) -> None:
    super().__init__("boreas_player")

    self.path_publisher = self.create_publisher(nav_msgs.Path, '/ground_truth_path', 10)
    self.odometry_publisher = self.create_publisher(nav_msgs.Odometry, '/ground_truth_odometry', 10)

    path_to_dataset = '/home/yuchen/ASRL/data/boreas/sequences'
    seq = 'boreas-2022-05-06-15-22'

    self.T_robot_applanix = np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    self.T_applanix_lidar = np.loadtxt(osp.join(path_to_dataset, seq, 'calib', 'T_applanix_lidar.txt'))
    self.T_radar_lidar = np.loadtxt(osp.join(path_to_dataset, seq, 'calib', 'T_radar_lidar.txt'))
    self.T_radar_robot = self.T_radar_lidar @ get_inverse_tf(self.T_applanix_lidar) @ get_inverse_tf(self.T_robot_applanix)

    # filepath = os.path.join(path_to_dataset, seq, 'applanix/lidar_poses.csv')
    # T_radar_lidar = np.loadtxt(osp.join(path_to_dataset, seq, 'calib', 'T_radar_lidar.txt'))
    # T_calib = T_radar_lidar
    # converted_poses, converted_times = read_traj_file_gt(filepath, T_calib, 3)  # T_radar_enu -> T_robot_enu
    # self.simulate_odometry(converted_poses, Time(seconds=0).to_msg(), 'world')

    filepath = os.path.join(path_to_dataset, seq, 'applanix/aeva_poses.csv')
    T_calib = np.identity(4)  # identity since we use radar frame
    poses, times = read_traj_file_gt(filepath, T_calib, 3)
    # self.simulate_odometry(poses, Time(seconds=0).to_msg(), 'world')

    path = self.poses2path(poses, Time(seconds=0).to_msg(), 'world')
    self.path_publisher.publish(path)

  def simulate_odometry(self, poses, stamp, to_frame):

    for T_vi in poses:
      T_iv = get_inverse_tf(T_vi)
      T_v0_v = poses[0] @ T_iv  # T_iv # @ self.T_radar_robot
      print(T_v0_v)
      pose_msg = geometry_msgs.Pose()
      tran = T_v0_v[:3, 3]
      rot = sptf.Rotation.from_matrix(T_v0_v[:3, :3]).as_quat()

      # The default (fixed) frame in RViz is called 'world'
      pose_msg.position.x = tran[0]
      pose_msg.position.y = tran[1]
      pose_msg.position.z = tran[2]
      pose_msg.orientation.x = rot[0]
      pose_msg.orientation.y = rot[1]
      pose_msg.orientation.z = rot[2]
      pose_msg.orientation.w = rot[3]

      odometry = nav_msgs.Odometry()
      odometry.header.frame_id = to_frame
      odometry.header.stamp = stamp
      odometry.pose.pose = pose_msg
      self.odometry_publisher.publish(odometry)
      time.sleep(0.01)

  def poses2path(self, poses, stamp, to_frame):
    paths = nav_msgs.Path()
    paths.header.frame_id = to_frame
    paths.header.stamp = stamp
    for T_vi in poses:
      T_iv = get_inverse_tf(T_vi)
      T_v0_v = poses[0] @ T_iv @ get_T_v0_v1_theta(-np.pi / 6)
      pose_msg = geometry_msgs.PoseStamped()
      tran = T_v0_v[:3, 3]
      rot = sptf.Rotation.from_matrix(T_v0_v[:3, :3]).as_quat()

      # The default (fixed) frame in RViz is called 'world'
      pose_msg.pose.position.x = tran[0]
      pose_msg.pose.position.y = tran[1]
      pose_msg.pose.position.z = tran[2]
      pose_msg.pose.orientation.x = rot[0]
      pose_msg.pose.orientation.y = rot[1]
      pose_msg.pose.orientation.z = rot[2]
      pose_msg.pose.orientation.w = rot[3]
      paths.poses.append(pose_msg)
    return paths


def main(args=None):
  rclpy.init(args=args)
  boreas_player = BoreasPlayer()
  rclpy.spin(boreas_player)
  rclpy.shutdown()


if __name__ == '__main__':
  main()
