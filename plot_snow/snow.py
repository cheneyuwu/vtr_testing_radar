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

matplotlib.rcParams.update({
  "text.usetex": True,
  "font.family": "serif",
  "font.serif": ["Times"],
  'font.size': 18,
}) 

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
  bounds = [-45, 45, -45, 30, -0.75, 10]
  lid.passthrough(bounds)
  lid.random_downsample(0.5)
  fig = plt.figure(figsize=(10, 10))
  ax = fig.add_subplot()
  rot = 0
  M = mtf.Affine2D().rotate_deg(rot)+plt.gca().transData
  sct = ax.scatter(lid.points[:, 0], lid.points[:, 1], s=1, c=lid.points[:, 2], vmin=-0.75, vmax=5, transform=M)
  ax.set_aspect('equal')
  ax.set_xlabel('x (m)')
  ax.set_ylabel('y (m)')
  ax.set_xlim(bounds[0], bounds[1])
  ax.set_ylim(bounds[2], bounds[3])
  fig.colorbar(sct, ax=ax, shrink=0.7)
  # plt.savefig(fname.split('.')[0] + '-snowy.pdf', pad_inches=0.0, bbox_inches='tight')
  img = convert_plt_to_img()
  cv2.imwrite(fname.split('.')[0] + '-snowy.png', img.astype(np.uint8))

def phi2C(phi):
  phi = phi / 180.0 * np.pi
  return np.array([[np.cos(phi), -np.sin(phi), 0.], [np.sin(phi), np.cos(phi), 0.], [0., 0., 1.]])

def plot_snow_removed(fname):
  if '.txt' not in fname:
    fname += '.txt'

  lid = PointCloud(np.loadtxt(fname))
  lid.transform(T_applanix_lidar)
  bounds = [-45, 45, -45, 30, -10, 10]
  lid.passthrough(bounds)
  lid.points = lid.points[lid.points[:, 3] > 0.5]
  # lid.random_downsample(0.5)
  fig = plt.figure(figsize=(10, 10))
  ax = fig.add_subplot()
  rot = 0
  M = mtf.Affine2D().rotate_deg(rot)+plt.gca().transData
  sct = ax.scatter(lid.points[:, 0], lid.points[:, 1], s=1, c=lid.points[:, 2], vmin=-0.75, vmax=5, transform=M)
  ax.set_aspect('equal')
  ax.set_xlabel('x (m)')
  ax.set_ylabel('y (m)')
  ax.set_xlim(bounds[0], bounds[1])
  ax.set_ylim(bounds[2], bounds[3])
  fig.colorbar(sct, ax=ax, shrink=0.7)
  # plt.savefig(fname.split('.')[0] + '-snow-removed.pdf', pad_inches=0.0, bbox_inches='tight')
  img = convert_plt_to_img()
  cv2.imwrite(fname.split('.')[0] + '-snow-removed.png', img.astype(np.uint8))

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--time', default="1611676836139444000", type=str, help='which timestamp to plot')
  args = parser.parse_args()
  plot_snowy(args.time)
  plot_snow_removed(args.time)
