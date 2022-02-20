import argparse
import os
from multiprocessing import Pool
import numpy as np
import matplotlib.pyplot as plt


def main(seq):
  rows = ['lidar-lidar', 'radar-radar', 'radar-lidar']

  fig, axs = plt.subplots(3, 3, figsize=(12, 12))
  for i, row in enumerate(rows):
    print(f"Row {i} corresponds to {row}")

    # load errors, location is relative to this script
    e = np.loadtxt(os.path.join(row, seq + '-err.txt'))

    # note: set same range for each column
    axs[i, 0].hist(e[:, 0], bins=20, range=(-1.5, 1.5))
    axs[i, 0].set_title('Lateral Error (m)')
    axs[i, 1].hist(e[:, 1], bins=20, range=(-1.5, 1.5))
    axs[i, 1].set_title('Longitudinal Error (m)')
    axs[i, 2].hist(e[:, 3], bins=20, range=(0, 3))
    axs[i, 2].set_title('Orientation Error (deg)')

  plt.savefig(os.path.join(seq + '_hist.pdf'), pad_inches=0, bbox_inches='tight')
  plt.close()


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--seq', default="boreas-2020-12-04-14-00", type=str, help='which sequence to plot')
  args = parser.parse_args()
  main(args.seq)
