import os
import os.path as osp
import argparse
import numpy as np
import numpy.linalg as npla
import csv

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

from pyboreas import BoreasDataset
from pylgmath import se3op


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
  loc_inputs = [i for i in os.listdir(result_dir) if (i != odo_input and i.startswith("boreas"))]
  loc_inputs.sort()
  print("Result Directory:", result_dir)
  print("Odometry Run:", odo_input)
  print("Localization Runs:", loc_inputs)
  print("Dataset Directory:", dataset_dir)

  # dataset directory and necessary sequences to load
  dataset_odo = BoreasDataset(osp.normpath(dataset_dir), [[odo_input]])

  # generate ground truth pose dictionary
  ground_truth_poses_odo = dict()
  for sequence in dataset_odo.sequences:
    # build dictionary
    precision = 1e7  # divide by this number to ensure always find the timestamp
    ground_truth_poses_odo.update(
        {int(int(frame.timestamp * 1e9) / precision): frame.pose for frame in sequence.radar_frames})
    # need to invert y axis
    for k in ground_truth_poses_odo.keys():
      ground_truth_poses_odo[k][1, 3] = -ground_truth_poses_odo[k][1, 3]
  print("Loaded number of odometry poses: ", len(ground_truth_poses_odo))

  for i, loc_input in enumerate(loc_inputs):

    # dataset directory and necessary sequences to load
    dataset_loc = BoreasDataset(osp.normpath(dataset_dir), [[loc_input]])

    # generate ground truth pose dictionary
    ground_truth_poses_loc = dict()
    for sequence in dataset_loc.sequences:
      # build dictionary
      precision = 1e7  # divide by this number to ensure always find the timestamp
      ground_truth_poses_loc.update(
          {int(int(frame.timestamp * 1e9) / precision): frame.pose for frame in sequence.radar_frames})
      # need to invert y axis
      for k in ground_truth_poses_loc.keys():
        ground_truth_poses_loc[k][1, 3] = -ground_truth_poses_loc[k][1, 3]

    print("Loaded number of localization poses: ", len(ground_truth_poses_loc))

    loc_dir = osp.join(result_dir, loc_input)

    data_dir = osp.join(loc_dir, "graph/data")
    if not osp.exists(data_dir):
      continue
    print("Looking at result data directory:", data_dir)

    # get bag file
    bag_file = '{0}/{1}/{1}_0.db3'.format(osp.abspath(data_dir), "localization_result")
    parser = BagFileParser(bag_file)
    messages = parser.get_bag_messages("localization_result")

    result = []
    errors = np.empty((len(messages), 6))
    for i, message in enumerate(messages):

      test_seq_timestamp = int(int(message[1].timestamp) / 1000)
      map_seq_timestamp = int(int(message[1].vertex_timestamp) / 1000)
      T_test_map_vec = np.array(message[1].t_robot_vertex.xi)[..., None]
      T_test_map = se3op.vec2tran(T_test_map_vec)
      T_test_map_in_radar = T_test_map
      T_map_test_in_radar = npla.inv(T_test_map_in_radar)
      T_map_test_in_radar_res = T_map_test_in_radar.flatten().tolist()[:12]
      result.append([test_seq_timestamp, map_seq_timestamp] + T_map_test_in_radar_res)

      if not int(message[1].timestamp / precision) in ground_truth_poses_loc.keys():
        print("WARNING: time stamp not found 1: ", int(message[1].timestamp / precision))
        continue
      if not int(message[1].vertex_timestamp / precision) in ground_truth_poses_odo.keys():
        print("WARNING: time stamp not found 2: ", int(message[1].vertex_timestamp / precision))
        continue

      test_seq_timestamp = int(message[1].timestamp / precision)
      map_seq_timestamp = int(message[1].vertex_timestamp / precision)
      T_test_map_vec = np.array(message[1].t_robot_vertex.xi)[..., None]
      T_test_map = se3op.vec2tran(T_test_map_vec)
      T_test_map_in_radar = T_test_map
      T_map_test_in_radar = npla.inv(T_test_map_in_radar)
      T_test_map_in_radar_gt = npla.inv(
          ground_truth_poses_loc[test_seq_timestamp]) @ ground_truth_poses_odo[map_seq_timestamp]
      # compute error
      errors[i, :] = se3op.tran2vec(T_map_test_in_radar @ T_test_map_in_radar_gt).flatten()

    print(np.mean(np.abs(errors), axis=0))

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