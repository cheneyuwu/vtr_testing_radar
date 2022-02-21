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

def main(seq):
  rows = ['lidar-lidar', 'radar-radar', 'radar-lidar']

  fig, axs = plt.subplots(3, 3, figsize=(7, 6))
  fig.subplots_adjust(wspace=0.25, hspace=0.25)

  for i, row in enumerate(rows):
    print(f"Row {i} corresponds to {row}")

    # load errors, location is relative to this script
    e = np.loadtxt(os.path.join(row, seq + '-err.txt'))

    # note: set same range for each column
    axs[i, 0].hist(e[:, 0], bins=50, range=(-1, 1))
    axs[i, 1].hist(e[:, 1], bins=50, range=(-1, 1))
    axs[i, 2].hist(e[:, 3], bins=50, range=(0, 2))
    if i==0:
      axs[i, 0].set_title('Lateral Error (m)')
      axs[i, 1].set_title('Longitudinal Error (m)')
      axs[i, 2].set_title('Heading Error (deg)')

  plt.savefig(os.path.join(seq + '_hist.pdf'), pad_inches=0.05, bbox_inches='tight')
  plt.close()


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--seq', default="boreas-2021-01-26-10-59", type=str, help='which sequence to plot')
  args = parser.parse_args()
  main(args.seq)
