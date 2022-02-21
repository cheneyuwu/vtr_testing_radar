import argparse
import os
from multiprocessing import Pool
import numpy as np
import matplotlib.pyplot as plt


def main():
  seq = "boreas-2021-01-26-10-59"
  rows = ['lidar-lidar', 'radar-radar', 'radar-lidar']

  fig, axs = plt.subplots(3, 1, figsize=(8, 12))

  for i, row in enumerate(rows):
    print(f"Row {i} corresponds to {row}")

    t = np.loadtxt(os.path.join(row, "timestamps.txt"))
    t -= t[0]
    # load errors, location is relative to this script
    e = np.loadtxt(os.path.join(row, seq + '-err.txt'))

    # note: set same range for each column
    indices = slice(0, t.shape[0], 10)
    axs[i].plot(t[indices], e[indices, 0], label='lateral')
    axs[i].plot(t[indices], e[indices, 1], label='longitudinal')
    axs[i].plot(t[indices], e[indices, 3], label='heading')
    axs[i].set_ylim([-1.8, 1.8])
    axs[i].set_title(row)
    axs[i].set_xlabel('time [ms]')
    axs[i].set_ylabel('error [m or deg]')
    axs[i].legend(loc="lower right")

  plt.savefig(os.path.join(seq + '_time.pdf'), pad_inches=0, bbox_inches='tight')
  plt.close()


if __name__ == '__main__':
  main()
