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
  seq = "/ext0/ASRL/temp/lidar/boreas/boreas-2020-11-26-13-58/localization_result/lidar-lidar/boreas-2021-01-26-10-59"

  t = np.loadtxt("timestamps.txt")
  # load errors, location is relative to this script
  e = np.loadtxt(seq + '-err.txt')
  t -= t[0]
  t /= 1.0e6
  print(e[:, 0].shape)
  print(t[:e.shape[0]].shape)
  t = t[:e.shape[0]]

  fig, axs = plt.subplots(3, 1, figsize=(6, 5))

  # note: set same range for each column
  axs[0].plot(t, e[:, 0], label='x (m)', color='b', linewidth=1)
  axs[0].set_ylabel('lat (m)')
  axs[0].set_xlabel('time (sec)')
  axs[1].plot(t, e[:, 1], label='y (m)', color='b', linewidth=1)
  axs[1].set_ylabel('lng (m)')
  axs[1].set_xlabel('time (sec)')
  # axs[2].plot(t, e[:, 2], label='z (m)', color='b', linewidth=1)
  # axs[2].set_ylabel('z (m)')
  # axs[3].plot(t, e[:, 3], label='r (m)', color='b', linewidth=1)
  # axs[3].set_ylabel('roll (deg)')
  # axs[4].plot(t, e[:, 4], label='p (m)', color='b', linewidth=1)
  # axs[4].set_ylabel('pitch (deg)')
  axs[2].plot(t, e[:, 5], label='y (m)', color='b', linewidth=1)
  axs[2].set_ylabel('yaw (deg)')
  axs[2].set_xlabel('time (sec)')


  ylim = 1.0
  for i in range(3):
    axs[i].grid(which='both', linestyle='--', alpha=0.75)
    axs[i].set_axisbelow(True)
    axs[i].set_xlim([t[0], t[-1]])
    axs[i].set_ylim([-ylim, ylim])

  plt.show()


if __name__ == '__main__':
  main()
