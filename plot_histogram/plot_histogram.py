import argparse
import os
from multiprocessing import Pool
import numpy as np
import matplotlib.pyplot as plt

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.serif": ["Times"],
    # 'font.size': 18,
    # 'xtick.labelsize': 18,
    # 'ytick.labelsize': 18,
    'axes.linewidth': 1,
    'axes.labelsize': 12,
})

def main(seq):
  rows = ['lidar-lidar', 'radar-radar', 'radar-lidar']

  fig, axs = plt.subplots(3, 3, figsize=(7, 6))
  fig.subplots_adjust(wspace=0.25, hspace=0.25)

  for i, row in enumerate(rows):
    print(f"Row {i} corresponds to {row}")

    # load errors, location is relative to this script
    e = np.loadtxt(os.path.join(row, seq + '-err.txt'))
    N = len(e[:, 0])
    weights=np.ones(N) / float(N)

    # note: set same range for each column
    y0, _, _ = axs[i, 0].hist(e[:, 0], bins=20, range=(-1, 1), color='b', weights=weights, rwidth=0.7)
    y1, _, _ = axs[i, 1].hist(e[:, 1], bins=20, range=(-1, 1), color='b', weights=weights, rwidth=0.7)
    y2, _, _ = axs[i, 2].hist(e[:, 5], bins=20, range=(-2, 2), color='b', weights=weights, rwidth=0.7)
    axs[i, 0].set_axisbelow(True)
    axs[i, 1].set_axisbelow(True)
    axs[i, 2].set_axisbelow(True)
    axs[i, 0].grid(which='both', linestyle='--', alpha=0.75)
    axs[i, 1].grid(which='both', linestyle='--', alpha=0.75)
    axs[i, 2].grid(which='both', linestyle='--', alpha=0.75)
    axs[i, 0].set_xticks([-1, -0.75, -0.5, -0.25, 0, 0.25, 0.5, 0.75, 1.0])
    axs[i, 1].set_xticks([-1, -0.75, -0.5, -0.25, 0, 0.25, 0.5, 0.75, 1.0])
    axs[i, 2].set_xticks([-2, -1.5, -1, -0.5, 0, 0.5, 1, 1.5, 2.0])

    labels = axs[i, 0].xaxis.get_ticklabels()
    for j in range(len(labels)):
      label = labels[j]
      if j > 0 and j < len(labels) - 1 and j != np.floor(len(labels) / 2):
        label.set_visible(False)

    labels = axs[i, 1].xaxis.get_ticklabels()
    for j in range(len(labels)):
      label = labels[j]
      if j > 0 and j < len(labels) - 1 and j != np.floor(len(labels) / 2):
        label.set_visible(False)

    labels = axs[i, 2].xaxis.get_ticklabels()
    for j in range(len(labels)):
      label = labels[j]
      if j > 0 and j < len(labels) - 1 and j != np.floor(len(labels) / 2):
        label.set_visible(False)

    ylim = np.max([y0, y1, y2]) + 0.025
    axs[i, 0].set_ylim(0, ylim)
    axs[i, 1].set_ylim(0, ylim)
    axs[i, 2].set_ylim(0, ylim)

    if i==0:
      axs[i, 0].set_title('Lateral Error (m)')
      axs[i, 1].set_title('Longitudinal Error (m)')
      axs[i, 2].set_title('Heading Error (deg)')
      axs[i, 0].set_ylabel('Lidar-to-Lidar')
    if i == 1:
      axs[i, 0].set_ylabel('Radar-to-Radar')
    if i == 2:
      axs[i, 0].set_ylabel('Radar-to-Lidar')

  plt.savefig(os.path.join(seq + '_hist.pdf'), pad_inches=0.0, bbox_inches='tight')
  plt.close()


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--seq', default="boreas-2021-01-26-10-59", type=str, help='which sequence to plot')
  args = parser.parse_args()
  main(args.seq)
