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

def main(type,time):

  # load errors, location is relative to this script
  scan = np.loadtxt(time + '_scan_' + type + '.txt')
  map = np.loadtxt(time + '_map_' + type + '.txt')
  print(scan.shape)
  print(map.shape)

  # plot of path
  fig, ax = setup_figure(1, 1, 2.0, 2.0)
  xm = np.mean(scan[:, 0])
  ym = np.mean(scan[:, 1])
  map[:, 0] -= xm
  map[:, 1] -= ym
  scan[:, 0] -= xm
  scan[:, 1] -= ym
  xlim = np.max(np.abs(np.concatenate((map[:, 0], scan[:, 0]))))
  ylim = np.max(np.abs(np.concatenate((map[:, 1], scan[:, 1]))))
  lim = np.max([xlim, ylim]) - 5
  ax.scatter(map[:, 0], map[:, 1], s=1, label='Submap', color='r')
  ax.scatter(scan[:, 0], scan[:, 1], s=1, label='Live Scan', color='b')
  ax.set_xlim(-lim, lim)
  ax.set_ylim(-lim, lim)
  ax.set_aspect('equal')
  ax.set_xlabel('x (m)')
  ax.set_ylabel('y (m)')
  ax.set_xticks([-60., -40., -20., 0., 20., 40., 60.])
  ax.set_yticks([-60., -40., -20., 0., 20., 40., 60.])

  # plt.legend(loc="upper left", prop={'size': 7})
  plt.savefig(time + '_aligned_' + type + '.pdf', pad_inches=0.0, bbox_inches='tight')
  plt.close()


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--type', default="radar_radar", type=str, help='which combination of sensors')
  parser.add_argument('--time', default="1631149285501197000", type=str, help='which timestamp to plot')
  args = parser.parse_args()
  main(args.type, args.time)
