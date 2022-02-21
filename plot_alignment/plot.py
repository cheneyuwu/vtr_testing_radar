import argparse
import os
from multiprocessing import Pool
import numpy as np
import matplotlib.pyplot as plt

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.serif": ["Times"],
    'font.size': 10,
}) 

def main(type,time):

  # load errors, location is relative to this script
  scan = np.loadtxt(time + '_scan_' + type + '.txt')
  map = np.loadtxt(time + '_map_' + type + '.txt')
  print(scan.shape)
  print(map.shape)

  # plot of path
  plt.figure(figsize=(4, 4))
  plt.scatter(map[:, 0], map[:, 1], 2, label='Submap')
  plt.scatter(scan[:, 0], scan[:, 1], 2, label='Live Scan')
  plt.xlabel('x (m)')
  plt.ylabel('y (m)')
  plt.axis('equal')
  plt.legend(loc="upper left", prop={'size': 7})
  plt.savefig(time + '_aligned_' + type + '.pdf', pad_inches=0.05, bbox_inches='tight')
  plt.close()


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--type', default="radar_radar", type=str, help='which combination of sensors')
  parser.add_argument('--time', default="1631149285501197000", type=str, help='which timestamp to plot')
  args = parser.parse_args()
  main(args.type, args.time)
