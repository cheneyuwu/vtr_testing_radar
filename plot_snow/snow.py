import os
from pyboreas import BoreasDataset
from pyboreas.data.pointcloud import PointCloud
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.transforms as mtf
import matplotlib
import io
import PIL
import argparse

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

def convert_plt_to_img(dpi=250):
  buf = io.BytesIO()
  plt.savefig(buf, format='png', dpi=dpi, bbox_inches='tight', pad_inches=0)
  plt.close()
  buf.seek(0)
  pimg = PIL.Image.open(buf)
  nimg = np.array(pimg)
  img = nimg[:, :, :3]
  return img[:, :, ::-1]

T_applanix_lidar = np.array([[7.274135642622281406e-01, -6.861993198242920533e-01, 0.000000000000000000e+00, 0.000000000000000000e+00],
[6.861993198242920533e-01, 7.274135642622281406e-01, 0.000000000000000000e+00, 0.000000000000000000e+00],
[0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00, 2.100000000000000000e-01],
[0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00]])

ratio = 1.0
w = int(2448 * ratio)
h = int(2048 * ratio)

def plot_snowy(fname):
  if '.txt' not in fname:
    fname += '.txt'
  lid = PointCloud(np.loadtxt(fname))
  lid.transform(T_applanix_lidar)
  bounds = [-45, 45, -45, 45, -0.75, 10]
  lid.passthrough(bounds)
  lid.random_downsample(0.5)
  fig, ax = setup_figure(1, 1, 2.5, 2.5)
  rot = 0
  M = mtf.Affine2D().rotate_deg(rot)+plt.gca().transData
  sct = ax.scatter(lid.points[:, 0], lid.points[:, 1], s=1, c=lid.points[:, 2], vmin=-0.75, vmax=5, transform=M)
  ax.set_aspect('equal')
  ax.set_xlabel('x (m)')
  ax.set_ylabel('y (m)')
  ax.set_xlim(bounds[0], bounds[1])
  ax.set_ylim(bounds[2], bounds[3])
  ax.set_xticks([-40., -20., 0., 20., 40.])
  ax.set_yticks([-40., -20., 0., 20., 40.])  
  fig.colorbar(sct, ax=ax, shrink=0.7)
  plt.savefig(fname.split('.')[0] + '-snowy.pdf', pad_inches=0.0, bbox_inches='tight')
  # img = convert_plt_to_img()
  # cv2.imwrite(fname.split('.')[0] + '-snowy.png', img.astype(np.uint8))

def phi2C(phi):
  phi = phi / 180.0 * np.pi
  return np.array([[np.cos(phi), -np.sin(phi), 0.], [np.sin(phi), np.cos(phi), 0.], [0., 0., 1.]])

def plot_snow_removed(fname):
  if '.txt' not in fname:
    fname += '.txt'

  lid = PointCloud(np.loadtxt(fname))
  lid.transform(T_applanix_lidar)
  bounds = [-45, 45, -45, 45, -0.75, 10]
  lid.passthrough(bounds)
  lid.points = lid.points[lid.points[:, 3] > 0.5]
  # lid.random_downsample(0.5)
  fig, ax = setup_figure(1, 1, 2.5, 2.5)
  rot = 0
  M = mtf.Affine2D().rotate_deg(rot)+plt.gca().transData
  sct = ax.scatter(lid.points[:, 0], lid.points[:, 1], s=1, c=lid.points[:, 2], vmin=-0.75, vmax=5, transform=M)
  ax.set_aspect('equal')
  ax.set_xlabel('x (m)')
  ax.set_ylabel('y (m)')
  ax.set_xlim(bounds[0], bounds[1])
  ax.set_ylim(bounds[2], bounds[3])
  ax.set_xticks([-40., -20., 0., 20., 40.])
  ax.set_yticks([-40., -20., 0., 20., 40.])
  fig.colorbar(sct, ax=ax, shrink=0.7)
  plt.savefig(fname.split('.')[0] + '-snow-removed.pdf', pad_inches=0.0, bbox_inches='tight')
  # img = convert_plt_to_img()
  # cv2.imwrite(fname.split('.')[0] + '-snow-removed.png', img.astype(np.uint8))

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--time', default="1611676836139444000", type=str, help='which timestamp to plot')
  args = parser.parse_args()
  plot_snowy(args.time)
  plot_snow_removed(args.time)
