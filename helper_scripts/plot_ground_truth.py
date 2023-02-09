import os
import os.path as osp
import argparse
import numpy as np
import numpy.linalg as npla
import csv
import matplotlib
import matplotlib.pyplot as plt

np.set_printoptions(linewidth=120, precision=3, suppress=True)

matplotlib.use("TkAgg")


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
    T[:3, :3] = yawPitchRollToRot(line[9], 0, 0)
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
    poses += [enforce_orthog(T_ab @ get_inverse_tf(pose))]  # convert T_iv to T_vi and apply calibration
    times += [int(time)]  # microseconds
  return poses, times


def read_traj_file(path):
  """Reads trajectory from a space-separated txt file
    Args:
        path (string): file path including file name
    Returns:
        (List[np.ndarray]): list of 4x4 poses
        (List[int]): list of times in microseconds
    """
  with open(path, "r") as file:
    # read each time and pose to lists
    poses = []
    times = []

    for line in file:
      line_split = line.strip().split()
      values = [float(v) for v in line_split[1:]]
      pose = np.zeros((4, 4), dtype=np.float64)
      pose[0, 0:4] = values[0:4]
      pose[1, 0:4] = values[4:8]
      pose[2, 0:4] = values[8:12]
      pose[3, 3] = 1.0
      poses.append(enforce_orthog(pose))
      times.append(int(line_split[0]))

  return poses, times


def plot_poses_in_2d(T_radar_world, ax):
  T_world_radar = []
  for pose in T_radar_world:
    T_world_radar.append(npla.inv(pose))
  T_world_radar = np.array(T_world_radar)

  # extract x and y coordinates
  xs = T_world_radar[:, 0, 3]
  ys = T_world_radar[:, 1, 3]

  # plot 2d trajectory
  ax.plot(xs, ys)


if __name__ == "__main__":

  path_to_dataset = '/home/yuchen/ASRL/data/boreas/sequences'
  seq = 'boreas-2021-09-02-11-42'
  filepath = os.path.join(path_to_dataset, seq, 'applanix/radar_poses.csv')  # use 'radar_poses.csv' for groundtruth

  T_calib = np.identity(4)  # identity since we use radar frame
  poses, times = read_traj_file_gt(filepath, T_calib, 2)
  fig = plt.figure()
  ax = fig.add_subplot()
  ax.set_title('Groundtruth')
  plot_poses_in_2d(poses, ax)


  # write result to pred file format
  result_dir = "/home/yuchen/ASRL/temp/radar/boreas/boreas-2021-09-02-11-42/odometry_result"
  os.makedirs(result_dir, exist_ok=True)
  filepath = osp.join(result_dir, "boreas-2021-09-02-11-42-gt.txt")
  result = []

  invpose0 = get_inverse_tf(poses[0])
  for time, pose in zip(times, poses):
    pose_res = (pose @ invpose0).flatten().tolist()[:12]  # T_vi_v0
    result.append([time] + pose_res)
  with open(filepath, "+w") as file:
    writer = csv.writer(file, delimiter=' ')
    writer.writerows(result)
    print("Written to file:", filepath)

  fig = plt.figure()
  ax = fig.add_subplot()
  ax.set_title('Prediction')

  filepath = osp.join(result_dir, "boreas-2021-09-02-11-42-gt.txt")
  poses, times = read_traj_file(filepath)
  plot_poses_in_2d(poses, ax)

  filepath = osp.join(result_dir, "boreas-2021-09-02-11-42.txt")
  poses, times = read_traj_file(filepath)
  plot_poses_in_2d(poses, ax)

  plt.show()
