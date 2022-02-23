import argparse
import os
from multiprocessing import Pool
import numpy as np
import matplotlib.pyplot as plt

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.serif": ["Times"],
    'font.size': 13,
})


def phi2C(phi):
  phi = phi / 180.0 * np.pi
  return np.array([[np.cos(phi), -np.sin(phi), 0.], [np.sin(phi), np.cos(phi), 0.], [0., 0., 1.]])


def main(time):

  # load errors, location is relative to this script
  scan = np.loadtxt(time + '_lidar.txt')
  # rotate to better view
  scan = scan @ phi2C(80)
  # crop to 60m
  scan = scan[np.where(np.sqrt(scan[:, 0] ** 2 + scan[:, 1] ** 2) < 60.0)[0]]
  # downsampling by 50%
  scan = scan[np.random.choice(np.arange(scan.shape[0]), int(scan.shape[0] / 2), False)]
  print(scan.shape)



  # plot of path
  plt.figure(figsize=(6, 5))
  xm = np.mean(scan[:, 0])
  ym = np.mean(scan[:, 1])
  scan[:, 0] -= xm
  scan[:, 1] -= ym
  xlim = np.max(np.abs(scan[:, 0]))
  ylim = np.max(np.abs(scan[:, 1]))
  plt.scatter(scan[:, 0], scan[:, 1], c=scan[:, 2], s=1, vmin=-2, vmax=12, cmap='hsv')
  plt.xlabel('x (m)')
  plt.ylabel('y (m)')
  plt.colorbar()
  plt.axis('equal')
  plt.savefig(time + '_cloud.pdf', pad_inches=0.0, bbox_inches='tight')
  plt.close()


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--time', default="1611676836139444000", type=str, help='which timestamp to plot')
  args = parser.parse_args()
  main(args.time)
