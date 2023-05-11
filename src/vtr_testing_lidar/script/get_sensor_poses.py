import sys
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import os
import os.path as osp
import argparse
import numpy as np
from multiprocessing import Pool
import multiprocessing
from bisect import bisect_left

try:
    from pysteam.trajectory.const_vel import Interface as TrajInterface
    from pysteam.evaluable.se3 import SE3StateVar
    from pysteam.evaluable.vspace import VSpaceStateVar
    from pysteam.trajectory.const_vel.variable import Time
    from pylgmath.se3.transformation import Transformation
except Exception as e:
    print("STEAM not found, cannot use trajectory interpolation.")
    TrajectoryInterpolator = None

root = None

def roll(r):
    return np.array([[1, 0, 0], [0, np.cos(r), np.sin(r)], [0, -np.sin(r), np.cos(r)]], dtype=np.float64)

def pitch(p):
    return np.array([[np.cos(p), 0, -np.sin(p)], [0, 1, 0], [np.sin(p), 0, np.cos(p)]], dtype=np.float64)

def yaw(y):
    return np.array([[np.cos(y), np.sin(y), 0], [-np.sin(y), np.cos(y), 0], [0, 0, 1]], dtype=np.float64)

def yawPitchRollToRot(y, p, r):
    Y = yaw(y)
    P = pitch(p)
    R = roll(r)
    C = np.matmul(P, Y)
    return np.matmul(R, C)

def rpy2rot(r, p, y):
    Y = yaw(y)
    P = pitch(p)
    R = roll(r)
    C = np.matmul(P, R)
    return np.matmul(Y, C)

def rotToYawPitchRoll(C, eps = 1e-15):
    i = 2
    j = 1
    k = 0
    c_y = np.sqrt(C[i, i]**2 + C[j, i]**2)
    if c_y > eps:
        r = np.arctan2(C[j, i], C[i, i])
        p = np.arctan2(-C[k, i], c_y)
        y = np.arctan2(C[k, j], C[k, k])
    else:
        r = 0
        p = np.arctan2(-C[k, i], c_y)
        y = np.arctan2(-C[j, k], C[j, j])

    return y, p, r

def get_inverse_tf(T):
    """Returns the inverse of a given 4x4 homogeneous transform.
    Args:
        T (np.ndarray): 4x4 transformation matrix
    Returns:
        np.ndarray: inv(T)
    """
    T2 = np.identity(4, dtype=T.dtype)
    R = T[0:3, 0:3]
    t = T[0:3, 3].reshape(3, 1)
    T2[0:3, 0:3] = R.transpose()
    T2[0:3, 3:] = np.matmul(-1 * R.transpose(), t)
    return T2

def strToTime(tin):
    tstr = str(tin)
    if '.' in tstr:
        return float(tstr)
    t = float(tstr)
    timeconvert = 1e-6
    if len(tstr) != 16 and len(tstr) > 10:
        timeconvert = 10**(-1 * (len(tstr) - 10))
    return t * timeconvert


def get_time_from_filename(file):
    return file.split('.')[0]

def get_applanix_data_at_time(time, gps_times, gps_lines):

    idx = bisect_left(gps_times, time)
    if idx >= len(gps_times):
        idx = len(gps_times) - 1
    d = abs(gps_times[idx] - time)
    if gps_times[idx] < time and idx < len(gps_times) - 1:
        if abs(gps_times[idx + 1] - time) < d:
            idx += 1
    elif gps_times[idx] > time and idx > 0:
        if abs(gps_times[idx - 1] - time) < d:
            idx -= 1

    closest = idx

    def _parse(gps_line):
        return [float(x) for x in gps_line.split(',')]

    gt_time = gps_times[closest]

    def _interpolate(lower, upper, t):
        assert(len(lower) == len(upper))
        tlow = lower[0]
        tupp = upper[0]
        assert(tlow < t and t < tupp)
        delta = tupp - tlow
        if delta == 0:
            return lower
        ratio = (t - tlow) / delta
        out = []
        for low, upp in zip(lower, upper):
            out.append(low + (upp - low) * ratio)
        out[0] = t
        return out

    line = _parse(gps_lines[closest])
    if gt_time < time:
        if closest == len(gps_lines) - 1:
            return line
        line_lower = line
        line_upper = _parse(gps_lines[closest + 1])
        return _interpolate(line_lower, line_upper, time)
    elif gt_time > time:
        if closest == 0:
            return line
        line_lower = _parse(gps_lines[closest - 1])
        line_upper = line
        return _interpolate(line_lower, line_upper, time)
    elif gt_time == time:
        return line

# Adjust orientation and translation from T_enu_applanix --> T_enu_sensor
def adjust_pose_to_sensor(line, T_applanix_sensor):
    T_enu_applanix = np.identity(4, dtype=np.float64)
    T_enu_applanix[0:3, 0:3] = yawPitchRollToRot(line[9], line[8], line[7])
    T_enu_applanix[0, 3] = line[1]
    T_enu_applanix[1, 3] = line[2]
    T_enu_applanix[2, 3] = line[3]
    T_enu_sensor = np.matmul(T_enu_applanix, T_applanix_sensor)
    line[1] = T_enu_sensor[0, 3]
    line[2] = T_enu_sensor[1, 3]
    line[3] = T_enu_sensor[2, 3]
    y, p, r = rotToYawPitchRoll(T_enu_sensor[:3, :3])
    line[7] = r
    line[8] = p
    line[9] = y

    adT_sensor_applanix = Transformation(T_ba=T_applanix_sensor).inverse().adjoint()
    varpi_applanix = np.zeros(6, dtype=np.float64)
    vbar = np.array([line[4], line[5], line[6]], dtype=np.float64)  # in enu
    varpi_applanix[:3] = np.matmul(T_enu_applanix[:3, :3].T, vbar.reshape(3, 1)).squeeze()  # rotate from enu to applanix
    varpi_applanix[3:] = np.array([line[12], line[11], line[10]], dtype=np.float64)     # angular already in applanix
    varpi_sensor = np.matmul(adT_sensor_applanix, varpi_applanix.reshape(6, 1)).squeeze()
    line[4], line[5], line[6] = np.matmul(T_enu_sensor[:3, :3], varpi_sensor[:3].reshape(3, 1)).squeeze()   # rotate back to enu
    line[12], line[11], line[10] = varpi_sensor[3:]     # angular should already be in sensor frame
    return line

def convert_line_to_state(line):
    """Converts a line in lidar_poses.csv to a steam trajectory state (time, pose, velocity)."""
    line = line.replace('\n', '').split(',')
    # gps time
    gps_time = float(line[0]) * 1.0e-6
    # ground truth pose expressed in enu frame
    T_enu_applanix = np.eye(4, dtype=np.float64)
    T_enu_applanix[:3, :3] = yawPitchRollToRot(float(line[9]), float(line[8]), float(line[7]))
    T_enu_applanix[0, 3] = float(line[1])
    T_enu_applanix[1, 3] = float(line[2])
    T_enu_applanix[2, 3] = float(line[3])
    # ground truth velocities in enu frame
    w_applanix_enu_in_enu = np.array([[float(line[4]), float(line[5]), float(line[6]), float(line[12]), float(line[11]), float(line[10])]]).T

    # convert T_enu_applanix to T_applanix_enu to be included in trajectory
    T_applanix_enu = np.linalg.inv(T_enu_applanix)

    # convert w_applanix_enu_in_enu to w_enu_applanix_in_applanix
    tmp = np.eye(6)
    tmp[:3, :3] = T_applanix_enu[:3, :3]
    w_enu_applanix_in_applanix = np.matmul(tmp, -w_applanix_enu_in_enu)

    return gps_time, T_applanix_enu, w_enu_applanix_in_applanix

def convert_state_to_line(state):
    """Converts a steam trajectory state (time, pose, velocity) to a line in *_poses.csv."""
    line = [None] * 13

    gps_time = state[0]
    T_applanix_enu = state[1]
    w_enu_applanix_in_applanix = state[2]

    line[0] = int(gps_time * 1.0e6)

    T_enu_applanix = np.linalg.inv(T_applanix_enu)

    line[1] = T_enu_applanix[0, 3]
    line[2] = T_enu_applanix[1, 3]
    line[3] = T_enu_applanix[2, 3]
    y, p, r = rotToYawPitchRoll(T_enu_applanix[:3, :3])
    line[9] = y
    line[8] = p
    line[7] = r

    # convert w_applanix_enu_in_enu to w_enu_applanix_in_applanix
    tmp = np.eye(6)
    tmp[:3, :3] = T_enu_applanix[:3, :3]
    w_applanix_enu_in_enu = np.matmul(tmp, -w_enu_applanix_in_applanix)

    line[4] = w_applanix_enu_in_enu[0, 0]
    line[5] = w_applanix_enu_in_enu[1, 0]
    line[6] = w_applanix_enu_in_enu[2, 0]
    line[10] = w_applanix_enu_in_enu[5, 0]
    line[11] = w_applanix_enu_in_enu[4, 0]
    line[12] = w_applanix_enu_in_enu[3, 0]

    return line

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', default='/mnt/data1/2020_12_01/',
                        type=str, help='path to /applanix, /lidar, /camera, and /radar')
    parser.add_argument('--trajectory', action='store_true', help='use steam to interpolate poses')
    args = parser.parse_args()
    root = args.root

    cpath = osp.join(root, 'camera')
    rpath = osp.join(root, 'radar')
    apath = osp.join(root, 'applanix', 'gps_post_process.csv')
    camera_times = []
    radar_times = []
    applanix_times = []
    if os.path.exists(rpath):
        radar_times = sorted([get_time_from_filename(f) for f in os.listdir(rpath) if f.endswith('.png')])
    if os.path.exists(cpath):
        camera_times = sorted([get_time_from_filename(f) for f in os.listdir(cpath) if f.endswith('.png')])
    if os.path.exists(apath):
        with open(apath, 'r') as f:
            f.readline()  # header
            lines = f.readlines()
            applanix_times = [float(line.split(',')[0]) for line in lines]

    T_applanix_lidar = np.loadtxt(osp.join(root, 'calib', 'T_applanix_lidar.txt'), dtype=np.float64)
    T_lidar_applanix = get_inverse_tf(T_applanix_lidar)
    T_camera_lidar = np.loadtxt(osp.join(root, 'calib', 'T_camera_lidar.txt'), dtype=np.float64)
    T_lidar_camera = get_inverse_tf(T_camera_lidar)
    T_radar_lidar = np.loadtxt(osp.join(root, 'calib', 'T_radar_lidar.txt'), dtype=np.float64)
    T_lidar_radar = get_inverse_tf(T_radar_lidar)

    f1 = open(osp.join('/home/krb/ASRL/boreas-objects-poses', 'applanix', 'camera_poses_new.csv'), 'w')
    f2 = open(osp.join('/home/krb/ASRL/boreas-objects-poses', 'applanix', 'radar_poses_new.csv'), 'w')
    f3 = open(osp.join('/home/krb/ASRL/boreas-objects-poses', 'applanix', 'gps_post_process_new.csv'), 'w')

    # lid_file = osp.join(root, 'applanix', 'lidar_poses.csv')
    lid_file = "/home/krb/ASRL/boreas-objects-poses/lidar_poses.csv"
    with open(lid_file, 'r') as gf:
        header = gf.readline()
        lid_lines = gf.readlines()
        lid_times = []
        for line in lid_lines:
            lid_times.append(float(line.split(',')[0]) * 1.0e-6)
        f1.write(header)
        f2.write(header)
        f3.write(header)

    if args.trajectory:
        assert TrajInterface, "STEAM not found, cannot use trajectory interpolation."
        print('Building trajectory interpolator...')
        traj = TrajInterface()
        for line in lid_lines:
            gps_time, T_lidar_enu, w_enu_lidar_in_lidar = convert_line_to_state(line)
            traj.add_knot(Time(gps_time), SE3StateVar(Transformation(T_ba=T_lidar_enu)), VSpaceStateVar(w_enu_lidar_in_lidar))

    def _processhelper(time, T_lidar_sensor):
        # Interpolate to get applanix data at the exact GPS time requested
        if args.trajectory:
            T_applanix_enu = traj.get_pose_interpolator(Time(time)).evaluate().matrix()
            w_enu_applanix_in_applanix = traj.get_velocity_interpolator(Time(time)).evaluate()
            state = [time, T_applanix_enu, w_enu_applanix_in_applanix]
            line = convert_state_to_line(state)
        else:
            line = get_applanix_data_at_time(time, lid_times, lid_lines)
            line = line[:13]
        # Incorporate the sensor extrinsics to adjust the pose orientation
        line = adjust_pose_to_sensor(line, T_lidar_sensor)
        line = line[1:]
        outstr = ''
        for item in line:
            outstr += '{},'.format(item)
        outstr = outstr[:-1]    # Get rid of last comma
        outstr += '\n'
        return outstr

    def _processcamera(time):
        outstr = _processhelper(strToTime(time), T_lidar_camera)
        return time + ',' + outstr

    def _processradar(time):
        outstr = _processhelper(strToTime(time), T_lidar_radar)
        return time + ',' + outstr

    def _processapplanix(time):
        outstr = _processhelper(time, T_lidar_applanix)
        return '{},'.format(time) + outstr

    print('Retrieving camera sensor poses...')
    pool = Pool(multiprocessing.cpu_count() - 2)
    # for time in camera_times:
    #     print(time)
    #     outstr = _processcamera(time)
    #     f1.write(outstr)
    outstrs = list(pool.imap(_processcamera, camera_times))
    for outstr in outstrs:
        f1.write(outstr)

    print('Retrieving radar sensor poses...')
    outstrs = list(pool.imap(_processradar, radar_times))
    for outstr in outstrs:
        f2.write(outstr)

    print('Retrieving applanix sensor poses...')
    outstrs = list(pool.imap(_processapplanix, applanix_times))
    for outstr in outstrs:
        f3.write(outstr)

    f1.close()
    f2.close()
    f3.close()
