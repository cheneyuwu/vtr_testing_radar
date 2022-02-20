import argparse
import os
from multiprocessing import Pool
import numpy as np
import matplotlib.pyplot as plt


def main(type,time):

  # load errors, location is relative to this script
  scan = np.loadtxt(time + '_scan_' + type + '.txt')
  map = np.loadtxt(time + '_map_' + type + '.txt')
  print(scan.shape)
  print(map.shape)

  # plot of path
  plt.figure(figsize=(6, 6))
  plt.scatter(map[:, 0], map[:, 1], label='Submap')
  plt.scatter(scan[:, 0], scan[:, 1], label='Scan')
  plt.xlabel('x [m]')
  plt.ylabel('y [m]')
  plt.axis('equal')
  plt.legend(loc="upper right")
  plt.savefig(time + '_aligned_' + type + '.pdf', pad_inches=0, bbox_inches='tight')
  plt.close()


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--type', default="radar_radar", type=str, help='which combination of sensors')
  parser.add_argument('--time', default="1631149285501197000", type=str, help='which timestamp to plot')
  args = parser.parse_args()
  main(args.type, args.time)
