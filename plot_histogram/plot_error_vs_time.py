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
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'axes.linewidth': 1,
    'axes.titlesize': 10,
    'axes.labelsize': 10,
})


def setup_figure(row, col, height, width, l=0.0, r=0.0, b=0.0, t=0.0, w=0.0, h=0.0):
  tot_height = row * height
  tot_width = col * width
  # axes spacing
  tot_height = tot_height + (height * h) * (row - 1)
  tot_width = tot_width + (width * w) * (col - 1)
  # left right padding
  tot_height = tot_height / (1 - (b + t))
  tot_width = tot_width / (1 - (l + r))

  fig, axs = plt.subplots(row, col, figsize=(tot_width, tot_height))
  fig.subplots_adjust(left=l, right=1.0 - r, bottom=b, top=1.0 - t)
  fig.subplots_adjust(wspace=w, hspace=h)
  print(f"Figure size (width, height): {fig.get_size_inches()}")
  return fig, axs


def main():
  seq = "boreas-2021-01-26-10-59"
  rows = ['lidar-lidar', 'radar-radar', 'radar-lidar']

  fig, axs = setup_figure(3, 1, 1.0, 4.0, l=0.10, r=0.05, b=0.05, t=0.07, w=0.33, h=0.30)

  ylims = []

  for i, row in enumerate(rows):
    print(f"Row {i} corresponds to {row}")

    t = np.loadtxt(os.path.join(row, "timestamps.txt"))
    print(t[0])
    t -= t[0]
    t /= 1.0e6
    # load errors, location is relative to this script
    e = np.loadtxt(os.path.join(row, seq + '-err.txt'))

    # note: set same range for each column
    indices = slice(0, t.shape[0], 10)
    axs[i].plot(t[indices], e[indices, 0], label='Lateral Error (m)', color='r', linewidth=1)
    axs[i].plot(t[indices], e[indices, 1], label='Longitudinal Error (m)', color='b', linewidth=1)
    # axs[i].plot(t[indices], e[indices, 5], label='Heading Error (deg)')

    axs[i].set_xlim([t[0], t[-1]])
    if i == 0:
      axs[i].set_ylabel('Lidar-to-Lidar')
    if i == 1:
      axs[i].set_ylabel('Radar-to-Radar')
    if i == 2:
      axs[i].set_ylabel('Radar-to-Lidar')

    axs[i].set_axisbelow(True)
    axs[i].grid(which='both', linestyle='--', alpha=0.75)

    ylims.append(np.max([np.max(abs(e[indices, 0])), np.max(abs(e[indices, 1]))]) + 0.025)

    # axs[i].set_ylim([-ylim, ylim])
    # axs[i].set_title(row)
    # axs[i].set_xlabel('Time (ms)')
    # axs[i].set_ylabel('Error')
    # axs[i].legend(loc="lower right")
  ylim = np.max(ylims)
  for i, _ in enumerate(rows):
    axs[i].set_ylim([-ylim, ylim])

  axs[i].set_xlabel('Time (s)')
  axs[i].legend(loc="upper center", ncol=3, bbox_to_anchor=(0.5, -0.4), frameon=False)

  plt.savefig(os.path.join(seq + '_time.pdf'), pad_inches=0.05, bbox_inches='tight')
  plt.close()


if __name__ == '__main__':
  main()
