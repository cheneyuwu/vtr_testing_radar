import argparse
import os
from multiprocessing import Pool
import numpy as np
import matplotlib.pyplot as plt

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.serif": ["Times"],
    # 'font.size': 10,
    'axes.linewidth': 1,
    'axes.labelsize': 12,
})

def main():
  seq = "boreas-2021-01-26-10-59"
  rows = ['lidar-lidar', 'radar-radar', 'radar-lidar']

  fig, axs = plt.subplots(3, 1, figsize=(6, 5))
  # fig.subplots_adjust(left=0.16, right=0.95, bottom=0.1, top=0.93, wspace=0.7, hspace=0.7)
  # fig.subplots_adjust(left=0.1, right=0.9, wspace=0.3, hspace=0.4)

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
  axs[i].legend(loc="upper center", ncol = 3, bbox_to_anchor=(0.5, -0.4))

  plt.savefig(os.path.join(seq + '_time.pdf'), pad_inches=0.05, bbox_inches='tight')
  plt.close()


if __name__ == '__main__':
  main()
