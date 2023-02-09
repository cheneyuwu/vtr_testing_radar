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
  rt = np.loadtxt(seq + '-err-times.txt')

  t -= t[0]
  t /= 1.0e6
  print(e[:, 0].shape)
  print(rt.shape)
  rt -= rt[0]
  print(t[:e.shape[0]].shape)
  t = t[:e.shape[0]]

  t_seg = []
  e_seg = []
  rt_prev = -1.0
  for i in range(e.shape[0]):
    if rt[i] != rt_prev:
      e_seg.append([])
      t_seg.append([])
      rt_prev = rt[i]
    e_seg[-1].append(e[i])
    t_seg[-1].append(t[i])

  print(len(e_seg))


  fig, axs = plt.subplots(1, 1, figsize=(6, 3))

  # note: set same range for each column
  clr = ['r', 'g', 'b', 'c', 'm', 'y', 'k']
  for i in range(len(t_seg)):
    t_ = np.array(t_seg[i])
    e_ = np.array(e_seg[i])
    axs.plot(t_, e_[:, 0], color=clr[i % len(clr)], linewidth=1)
  axs.set_ylabel('lat (m)')
  axs.set_xlabel('time (sec)')


  ylim = 1.0
  axs.grid(which='both', linestyle='--', alpha=0.75)
  axs.set_axisbelow(True)
  axs.set_xlim([t[0], t[-1]])
  axs.set_ylim([-ylim, ylim])

  plt.show()


if __name__ == '__main__':
  main()
