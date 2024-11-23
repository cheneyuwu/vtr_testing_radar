import os
import os.path as osp
import argparse
import numpy as np
import pandas as pd
import numpy.linalg as npla
import csv

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

from pylgmath import se3op
from pyboreas import BoreasDataset
import pyboreas.utils.se3_utils_numpy as se3
from pyboreas.eval.odometry_aeva import get_aeva_hq_groundtruth
from pyboreas.utils.odometry import (
    get_sequence_poses,
    get_sequences,
) 

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

    self.topic_type = {name_of: type_of for id_of, name_of, type_of in topics_data}
    self.topic_id = {name_of: id_of for id_of, name_of, type_of in topics_data}
    self.topic_msg_message = {name_of: get_message(type_of) for id_of, name_of, type_of in topics_data}

  # Return messages as list of tuples [(timestamp0, message0), (timestamp1, message1), ...]
  def get_bag_messages(self, topic_name):
    topic_id = self.topic_id[topic_name]
    rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
    return [(timestamp, deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp, data in rows]


def main(dataset_dir, result_dir):
  result_dir = osp.normpath(result_dir)
  odo_input = osp.basename(result_dir)
  loc_inputs = [i for i in os.listdir(result_dir) if (i != odo_input and i.startswith("route"))]
  loc_inputs.sort()
  print("Result Directory:", result_dir)
  print("Odometry Run:", odo_input)
  print("Localization Runs:", loc_inputs)
  print("Dataset Directory:", dataset_dir)
  
  T_lidar_robot = np.array([[ 0.9999366830849237  , 0.008341717781538466 , 0.0075534496251198685,-1.0119098938516395],
                            [-0.008341717774127972, 0.9999652112886684   ,-3.150635091210066e-05,-0.3965882433517194],
                            [-0.007553449599178521,-3.150438868196706e-05, 0.9999714717963843   ,-1.697000000000001 ],
                            [ 0.00000000e+00      , 0.00000000e+00       , 0.00000000e+00       , 1.00000000e+00    ]]).astype(np.float64)
  T_robot_lidar = get_inverse_tf(T_lidar_robot)
  
  # generate ground truth pose dictionary
  ground_truth_poses_odo = dict()
  pdcsv = pd.read_csv(os.path.join(dataset_dir, odo_input, "processed_sbet.csv"))
  
  pred = os.path.join(result_dir, "odometry_result")
  seq = get_sequences(pred, ".txt")
  _, times_pred_odo, _ = get_sequence_poses(pred, seq)
  T_gt_odo, times_gt_odo, seq_lens_gt_odo = get_aeva_hq_groundtruth(pdcsv, times_pred_odo) # T_vi
  
  T_gt_odo = se3.se3_inv(T_lidar_robot @ T_gt_odo) # T_is
  
  precision = 1e7 
  ground_truth_poses_odo.update(
      {int(times_pred_odo[i]): T_gt_odo[i] for i in range(seq_lens_gt_odo[0])})
    
  print("Loaded number of odometry poses: ", len(ground_truth_poses_odo))

  for i, loc_input in enumerate(loc_inputs):
    ground_truth_poses_loc = dict()
    pdcsv = pd.read_csv(os.path.join(dataset_dir, loc_input, "processed_sbet.csv"))

    # Initialize times_pred_loc
    times_pred_loc = []

    loc_dir = osp.join(result_dir, loc_input)
    data_dir = osp.join(loc_dir, "graph/data")
    if not osp.exists(data_dir):
        continue
    print("Looking at result data directory:", data_dir)

    # Get bag file
    bag_file = '{0}/{1}/{1}_0.db3'.format(osp.abspath(data_dir), "localization_result")
    parser = BagFileParser(bag_file)
    messages = parser.get_bag_messages("localization_result")

    # Extract timestamps from messages
    times_pred_loc = [msg[0]/1000 for msg in messages]  # Extracting the first element (timestamp) of each message tuple
    times_pred_loc = np.array(times_pred_loc)

    T_gt_loc, times_gt_loc, seq_lens_gt_loc = get_aeva_hq_groundtruth(pdcsv, times_pred_loc) # T_vi
    
    T_gt_loc = se3.se3_inv(T_lidar_robot @ T_gt_loc)
    
    # build dictionary
    precision = 1e7  # divide by this number to ensure always find the timestamp
    ground_truth_poses_loc.update(
        {int(times_pred_loc[i]): T_gt_loc[i] for i in range(seq_lens_gt_loc[0])})

    print("Loaded number of localization poses: ", len(ground_truth_poses_loc))

    result = []
    errors = np.empty((len(messages), 6))
    for i, message in enumerate(messages):

      test_seq_timestamp = int(int(message[1].timestamp))
      map_seq_timestamp = int(int(message[1].vertex_timestamp))
      T_test_map_vec = np.array(message[1].t_robot_vertex.xi)[..., None]
      T_test_map = se3op.vec2tran(T_test_map_vec) 
      T_test_map_in_lidar = T_lidar_robot @ T_test_map @ T_robot_lidar
      T_map_test_in_lidar = get_inverse_tf(T_test_map_in_lidar)
      T_map_test_in_lidar_res = T_map_test_in_lidar.flatten().tolist()[:12]
      result.append([test_seq_timestamp, map_seq_timestamp] + T_map_test_in_lidar_res)

      if not int(message[1].timestamp / 1000) in ground_truth_poses_loc.keys():
        print("WARNING: time stamp not found 1: ", int(message[1].timestamp))
        continue
      if not int(message[1].vertex_timestamp / 1000) in ground_truth_poses_odo.keys():
        print("WARNING: time stamp not found 2: ", int(message[1].vertex_timestamp))
        continue

      test_seq_timestamp = int(message[1].timestamp / 1000)
      map_seq_timestamp = int(message[1].vertex_timestamp / 1000)
      T_test_map_vec = np.array(message[1].t_robot_vertex.xi)[..., None]
      T_test_map = se3op.vec2tran(T_test_map_vec)
      T_test_map_in_lidar = T_lidar_robot @ T_test_map @ T_robot_lidar
      T_map_test_in_lidar = get_inverse_tf(T_test_map_in_lidar)
      T_test_map_in_lidar_gt = get_inverse_tf(
          ground_truth_poses_loc[test_seq_timestamp]) @ ground_truth_poses_odo[map_seq_timestamp]
      # compute error
      errors[i, :] = se3op.tran2vec(T_map_test_in_lidar @ T_test_map_in_lidar_gt).flatten()

    print(np.mean(np.abs(errors), axis=0))
    print(np.sqrt(np.mean(np.power(errors, 2), axis=0)))

    output_dir = osp.join(result_dir, "localization_result")
    os.makedirs(output_dir, exist_ok=True)
    with open(osp.join(output_dir, loc_input + ".txt"), "+w") as file:
      writer = csv.writer(file, delimiter=' ')
      writer.writerows(result)
      print("Written to file:", osp.join(output_dir, loc_input + ".txt"))


if __name__ == "__main__":

  parser = argparse.ArgumentParser()

  # Assuming following path structure:
  # <rosbag name>/metadata.yaml
  # <rosbag name>/<rosbag name>_0.db3
  parser.add_argument('--dataset', default=os.getcwd(), type=str, help='path to boreas dataset (contains boreas-*)')
  parser.add_argument('--path', default=os.getcwd(), type=str, help='path to vtr folder (default: os.getcwd())')

  args = parser.parse_args()

  main(args.dataset, args.path)