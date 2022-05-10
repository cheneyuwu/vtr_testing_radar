import os
import os.path as osp
import argparse
import numpy as np
import numpy.linalg as npla
import csv
import time
import scipy.spatial.transform as sptf
import matplotlib.pyplot as plt

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

from pylgmath import se3op
from pyboreas import BoreasDataset

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.time_source import CLOCK_TOPIC
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs
import nav_msgs.msg as nav_msgs
import rosgraph_msgs.msg as rosgraph_msgs


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


class BagFileParser():

  def __init__(self, bag_file):
    try:
      self.conn = sqlite3.connect(bag_file)
    except Exception as e:
      print('Could not connect: ', e)
      raise Exception('could not connect')

    self.cursor = self.conn.cursor()

    ## create a message (id, topic, type) map
    topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
    self.topic_id = {name_of: id_of for id_of, name_of, type_of in topics_data}
    self.topic_msg_message = {name_of: get_message(type_of) for id_of, name_of, type_of in topics_data}

  # Return messages as list of tuples [(timestamp0, message0), (timestamp1, message1), ...]
  def get_bag_messages(self, topic_name):
    topic_id = self.topic_id[topic_name]
    rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
    return [(timestamp, deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp, data in rows]


def simulate_odometry(publisher, poses, stamp, to_frame):

  for T_w_r in poses:
    pose_msg = geometry_msgs.Pose()
    tran = T_w_r[:3, 3]
    rot = sptf.Rotation.from_matrix(T_w_r[:3, :3]).as_quat()

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
    publisher.publish(odometry)

    time.sleep(0.01)


def poses2path(poses, frame_id="world"):
  paths = nav_msgs.Path()
  paths.header.frame_id = frame_id
  # paths.header.stamp = 0
  for T_w_r in poses:
    pose_msg = geometry_msgs.PoseStamped()
    tran = T_w_r[:3, 3]
    rot = sptf.Rotation.from_matrix(T_w_r[:3, :3]).as_quat()

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


def se3_projection_v3(T1, T2, T):
  r_r1_in_1 = (npla.inv(T) @ T1)[:3, 3:4]
  r_21_in_1 = (npla.inv(T2) @ T1)[:3, 3:4]
  # print(r_r1_in_1.squeeze(), r_21_in_1.squeeze())
  dot_r1_21 = (r_r1_in_1.T @ r_21_in_1)[0, 0]
  dot_21_21 = (r_21_in_1.T @ r_21_in_1)[0, 0]
  alpha = dot_r1_21 / dot_21_21
  return se3op.vec2tran(alpha * se3op.tran2vec(T2 @ npla.inv(T1))) @ T1, alpha


def se3_projection_v2(T1, T2, Tr):
  xi_1r = se3op.tran2vec(T1 @ npla.inv(Tr))
  Jinv_xi_1r = se3op.vec2jacinv(xi_1r)
  xi_21 = se3op.tran2vec(T2 @ npla.inv(T1))
  x = (Jinv_xi_1r @ xi_21)
  b = xi_1r
  alpha = -(b.T @ x)[0, 0] / (x.T @ x)[0, 0]
  return se3op.vec2tran(alpha * se3op.tran2vec(T2 @ npla.inv(T1))) @ T1, alpha


def se3_projection(T1, T2, T):
  pa = T @ npla.inv(T1)
  ba = T2 @ npla.inv(T1)
  # print(T2 @ npla.inv(T1))
  dot_pa_ba = (se3op.tran2vec(pa).T @ se3op.tran2vec(ba))[0, 0]
  dot_ba_ba = (se3op.tran2vec(ba).T @ se3op.tran2vec(ba))[0, 0]
  # print(se3op.tran2vec(pa).squeeze())
  # print(se3op.tran2vec(ba).squeeze())
  t = dot_pa_ba / dot_ba_ba
  return se3op.vec2tran(t * se3op.tran2vec(T2 @ npla.inv(T1))) @ T1, t


def project_poses(loc_poses, odo_poses):
  # need to invert because projection takes T_r_w instead of T_w_r
  odo_poses = [get_inverse_tf(pose) for pose in odo_poses]
  loc_poses = [get_inverse_tf(pose) for pose in loc_poses]

  projected_list = []
  curr_index = 0
  for frame, loc_T_r_w in enumerate(loc_poses):
    T, t = se3_projection_v2(odo_poses[curr_index], odo_poses[curr_index + 1], loc_T_r_w)
    while True:
      if curr_index >= len(odo_poses) - 2:
        break
      T_new, t_new = se3_projection_v2(odo_poses[curr_index + 1], odo_poses[curr_index + 2], loc_T_r_w)
      if abs(t_new - 0.5) < abs(t - 0.5):
        print("t_new: ", t_new, "t: ", t)
        T = T_new
        t = t_new
        curr_index += 1
      else:
        break
    print("frame: ", frame, " curr_index: ", curr_index, "t: ", t)
    projected_list.append(T)

  projected_list = [get_inverse_tf(pose) for pose in projected_list]
  return projected_list


def project_poses_colored(t, rt, query_poses, projected_poses):

  t = np.array(t, dtype=np.float64)
  t -= t[0]
  t /= 1.0e9

  rt = np.array(rt, dtype=np.float64)
  rt -= rt[0]

  e = []
  for query, projected in zip(query_poses, projected_poses):
    e.append(se3op.tran2vec(get_inverse_tf(projected) @ query))
  e = np.array(e)

  assert len(t) == e.shape[0]

  t_seg = []
  e_seg = []
  rt_prev = -1.0
  for i in range(len(t)):
    if rt[i] != rt_prev:
      e_seg.append([])
      t_seg.append([])
      rt_prev = rt[i]
    e_seg[-1].append(e[i])
    t_seg[-1].append(t[i])

  fig, axs = plt.subplots(1, 1, figsize=(6, 3))

  # note: set same range for each column
  clr = ['r', 'g', 'b', 'c', 'm', 'y', 'k']
  for i in range(len(t_seg)):
    t_ = np.array(t_seg[i])
    e_ = np.array(e_seg[i])
    axs.plot(t_, e_[:, 1], color=clr[i % len(clr)], linewidth=1)
  axs.set_ylabel('lat (m)')
  axs.set_xlabel('time (sec)')

  ylim = 1.0
  axs.grid(which='both', linestyle='--', alpha=0.75)
  axs.set_axisbelow(True)
  axs.set_xlim([t[0], t[-1]])
  axs.set_ylim([-ylim, ylim])

  plt.show()


def plot_errors(t, query_poses, projected_poses):
  # poses are in T_w_r
  t = np.array(t, dtype=np.float64)
  t -= t[0]
  t /= 1.0e9

  e = []
  for query, projected in zip(query_poses, projected_poses):
    e.append(se3op.tran2vec(get_inverse_tf(projected) @ query))
  e = np.array(e)

  assert len(t) == e.shape[0]

  fig, axs = plt.subplots(3, 1, figsize=(6, 5))

  # note: set same range for each column
  axs[0].plot(t, e[:, 0], label='x (m)', color='b', linewidth=1)
  axs[0].set_ylabel('lng (m)')
  axs[0].set_xlabel('time (sec)')
  axs[1].plot(t, e[:, 1], label='y (m)', color='b', linewidth=1)
  axs[1].set_ylabel('lat (m)')
  axs[1].set_xlabel('time (sec)')
  # axs[2].plot(t, e[:, 2], label='z (m)', color='b', linewidth=1)
  # axs[2].set_ylabel('z (m)')
  # axs[3].plot(t, e[:, 3], label='r (m)', color='b', linewidth=1)
  # axs[3].set_ylabel('roll (deg)')
  # axs[4].plot(t, e[:, 4], label='p (m)', color='b', linewidth=1)
  # axs[4].set_ylabel('pitch (deg)')
  axs[2].plot(t, e[:, 5], label='y (m)', color='b', linewidth=1)
  axs[2].set_ylabel('yaw (deg)')
  axs[2].set_xlabel('time (sec)')

  ylim = 1.0
  for i in range(3):
    axs[i].grid(which='both', linestyle='--', alpha=0.75)
    axs[i].set_axisbelow(True)
    axs[i].set_xlim([t[0], t[-1]])
    axs[i].set_ylim([-ylim, ylim])

  plt.show()


def main(odo_dir, loc_dir):
  odo_input = osp.basename(odo_dir)
  print("Odometry Run:", odo_input)
  loc_input = osp.basename(loc_dir)
  print("Localization Run:", loc_input)

  data_dir = osp.join(odo_dir, "graph/data")
  if not osp.exists(data_dir):
    return
  print("Looking at odometry data directory:", data_dir)
  # get bag file
  bag_file = '{0}/{1}/{1}_0.db3'.format(osp.abspath(data_dir), "odometry_result")
  parser = BagFileParser(bag_file)
  messages = parser.get_bag_messages("odometry_result")

  odo_result_list = []
  odo_result_dict = dict()
  for _, message in enumerate(messages):
    timestamp = int(message[1].timestamp)
    T_w_r_vec = np.array(message[1].t_world_robot.xi)[..., None]
    T_w_r = se3op.vec2tran(T_w_r_vec)
    odo_result_list.append(T_w_r)
    odo_result_dict[timestamp] = T_w_r

  data_dir = osp.join(loc_dir, "graph/data")
  if not osp.exists(data_dir):
    return
  print("Looking at localization data directory:", data_dir)
  # get bag file
  bag_file = '{0}/{1}/{1}_0.db3'.format(osp.abspath(data_dir), "localization_result")
  parser = BagFileParser(bag_file)
  messages = parser.get_bag_messages("localization_result")

  timestamp_list = []
  vertex_timestamp_list = []
  last_teach_timestamp = -1
  subsampled_odo_result_list = []
  loc_result_list = []
  loc_result_dict = dict()
  for _, message in enumerate(messages):
    timestamp = int(message[1].timestamp)
    teach_timestamp = int(message[1].vertex_timestamp)
    T_r_v_vec = np.array(message[1].t_robot_vertex.xi)[..., None]
    T_r_v = se3op.vec2tran(T_r_v_vec)
    T_w_r = odo_result_dict[teach_timestamp] @ get_inverse_tf(T_r_v)
    #
    timestamp_list.append(timestamp)
    vertex_timestamp_list.append(teach_timestamp)
    #
    if last_teach_timestamp != teach_timestamp:
      subsampled_odo_result_list.append(odo_result_dict[teach_timestamp])
      last_teach_timestamp = teach_timestamp
    #
    loc_result_list.append(T_w_r)
    loc_result_dict[timestamp] = T_w_r

  projected_loc_result_list = project_poses(loc_result_list, subsampled_odo_result_list)

  plot_errors(timestamp_list, loc_result_list, projected_loc_result_list)
  # project_poses_colored(timestamp_list, vertex_timestamp_list, loc_result_list, projected_loc_result_list)

  return

  rclpy.init()
  node = Node("boreas_player")

  odometry_path_publisher = node.create_publisher(nav_msgs.Path, '/odometry_path', 10)
  subsampled_path_publisher = node.create_publisher(nav_msgs.Path, '/subsampled_path', 10)
  localization_path_publisher = node.create_publisher(nav_msgs.Path, '/localization_path', 10)
  projected_path_publisher = node.create_publisher(nav_msgs.Path, '/projected_path', 10)

  odometry_path_publisher.publish(poses2path(odo_result_list, "world"))
  subsampled_path_publisher.publish(poses2path(subsampled_odo_result_list, "world"))
  localization_path_publisher.publish(poses2path(loc_result_list, "world"))
  projected_path_publisher.publish(poses2path(projected_loc_result_list, "world"))

  print("Simulating odometry")
  odometry_publisher = node.create_publisher(nav_msgs.Odometry, '/ground_truth_odometry', 10)
  simulate_odometry(odometry_publisher, loc_result_list, Time(seconds=0).to_msg(), 'world')


if __name__ == "__main__":

  parser = argparse.ArgumentParser()

  # Assuming following path structure:
  # <rosbag name>/metadata.yaml
  # <rosbag name>/<rosbag name>_0.db3
  parser.add_argument('--odo', default=os.getcwd(), type=str, help='path to vtr folder (default: os.getcwd())')
  parser.add_argument('--loc', default=os.getcwd(), type=str, help='path to vtr folder (default: os.getcwd())')

  args = parser.parse_args()

  main(args.odo, args.loc)