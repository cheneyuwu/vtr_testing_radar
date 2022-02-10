import os
import os.path as osp
import argparse
import numpy as np
import numpy.linalg as npla
import csv

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

from pylgmath import se3op
from pyboreas import BoreasDataset


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

def main(dataset_dir, result_dir):
  result_dir = osp.normpath(result_dir)
  odo_inputs = [i for i in os.listdir(result_dir) if osp.isdir(osp.join(result_dir, i))]
  print(odo_inputs)

  dataset = BoreasDataset(osp.normpath(dataset_dir), [[x] for x in odo_inputs])

  for seq_num, odo_input in enumerate(odo_inputs):
    odo_dir = osp.join(result_dir, odo_input)

    result_dir = osp.join(odo_dir, "graph/data")
    if not osp.exists(result_dir):
      continue
    print("Looking at result directory:", result_dir)

    # T_applanix_lidar = dataset.sequences[seq_num].calib.T_applanix_lidar
    # T_radar_lidar = dataset.sequences[seq_num].calib.T_radar_lidar
    # T_applanix_radar = T_applanix_lidar @ npla.inv(T_radar_lidar)
    T_robot_applanix = np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    # T_robot_radar = T_robot_applanix @ T_applanix_radar

    # get bag file
    bag_file = '{0}/{1}/{1}_0.db3'.format(osp.abspath(result_dir), "odometry_result")
    parser = BagFileParser(bag_file)
    messages = parser.get_bag_messages("odometry_result")

    result = []
    for _, message in enumerate(messages):
      timestamp = int(message[1].timestamp / 1e3)
      T_w_r_vec = np.array(message[1].t_world_robot.xi)[..., None]
      T_w_r = se3op.vec2tran(T_w_r_vec)
      T_w_a = T_w_r @ T_robot_applanix
      T_a_w_res = npla.inv(T_w_a).flatten().tolist()[:12]
      result.append([timestamp] + T_a_w_res)

    with open(osp.join(result_dir, odo_dir+".txt"), "+w") as file:
      writer = csv.writer(file, delimiter=' ')
      writer.writerows(result)
      print("Written to file:", odo_dir+".txt")


if __name__ == "__main__":

  parser = argparse.ArgumentParser()

  # Assuming following path structure:
  # <rosbag name>/metadata.yaml
  # <rosbag name>/<rosbag name>_0.db3
  parser.add_argument('--dataset', default=os.getcwd(), type=str, help='path to boreas dataset (that contains boreas-xxxx folders)')
  parser.add_argument('--path', default=os.getcwd(), type=str, help='path to vtr folder (default: os.getcwd())')

  args = parser.parse_args()

  main(args.dataset, args.path)