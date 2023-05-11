import numpy as np
from tqdm import tqdm
from pyboreas.utils.utils import get_transform, get_inverse_tf, rotToYawPitchRoll
from pylgmath.se3.transformation import Transformation

old_path = "/workspace/nas/ASRL/2021-Boreas/boreas-objects-v1/applanix_20230405backup/lidar_poses.csv"
new_path = "/home/krb/ASRL/boreas-objects-poses/boreas-objects-v1-v2.txt"
vel_path = "/home/krb/ASRL/boreas-objects-poses/vtr-2023-04-06_21-11-15_293485029Z-velocities.txt"

# NOTE: the timestamps should line up
fold = open(old_path, 'r')
fnew = open(new_path, 'r')
fvel = open(vel_path, 'r')
header = fold.readline()  # kick out the header
oldlines = fold.readlines()
newlines = fnew.readlines()
vellines = fvel.readlines()
fold.close()
fnew.close()
fvel.close()
assert len(oldlines) == len(newlines) == len(vellines)

# get sequence indices and timestamps:
seq_indices = [0]
t_prev = int(oldlines[0].split(',')[0])
seq_times = [t_prev]
gt = [float(x) for x in oldlines[0].split(',')]
T_enu_lidar0 = get_transform(gt)
T_applanix_lidar = np.loadtxt('/workspace/nas/ASRL/2021-Boreas/boreas-objects-v1/calib/T_applanix_lidar.txt')
T_lidar_applanix = get_inverse_tf(T_applanix_lidar)

T_robot_applanix = np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

T_lidar_robot = T_lidar_applanix @ get_inverse_tf(T_robot_applanix)
adT_lidar_robot = Transformation(T_ba=T_lidar_robot).adjoint()

# f = open("/workspace/nas/ASRL/2021-Boreas/boreas-objects-v1/applanix/lidar_poses.csv", 'w')
f = open("/home/krb/ASRL/boreas-objects-poses/lidar_poses.csv", 'w')
f.write(header)

for idx, line in tqdm(enumerate(oldlines)):
    t = int(line.split(',')[0])
    if t - t_prev > 5e5:
        seq_indices.append(idx)
        seq_times.append(t)
        gt = [float(x) for x in line.split(',')]
        T_enu_lidar0 = get_transform(gt)

    new_data = [float(x) for x in newlines[idx].split()]
    T_ak_r0 = np.eye(4)
    T_ak_r0[:3, :] = np.array(new_data[1:]).reshape(3, 4)  # world (robot at t=0) to applanix at t=k
    T_lk_l0 = T_lidar_applanix @ T_ak_r0 @ T_robot_applanix @ T_applanix_lidar
    T_l0_lk = get_inverse_tf(T_lk_l0)
    T_enu_lk = T_enu_lidar0 @ T_l0_lk

    w_m_r_in_r = np.array([float(x) for x in vellines[idx].split()]).reshape(6, 1)
    w_l_m_in_l = -1 * adT_lidar_robot @ w_m_r_in_r
    # transform into lidar frame
    # note: v_s_e_in_e, omega_s_e_in_s
    # save stuff to pose file
    line = [None] * 13
    line[0] = t
    line[1] = T_enu_lk[0, 3]
    line[2] = T_enu_lk[1, 3]
    line[3] = T_enu_lk[2, 3]
    y, p, r = rotToYawPitchRoll(T_enu_lk[:3, :3])
    line[9] = y
    line[8] = p
    line[7] = r
    v_s_e_in_e = T_enu_lk[:3, :3] @ w_l_m_in_l[:3].reshape(3, 1)
    omega_s_e_in_s = w_l_m_in_l[3:]
    line[4] = v_s_e_in_e[0, 0]
    line[5] = v_s_e_in_e[1, 0]
    line[6] = v_s_e_in_e[2, 0]
    line[10] = omega_s_e_in_s[2, 0]
    line[11] = omega_s_e_in_s[1, 0]
    line[12] = omega_s_e_in_s[0, 0]

    outstr = ''
    for item in line:
        outstr += '{},'.format(item)
    outstr = outstr[:-1]    # Get rid of last comma
    outstr += '\n'
    f.write(outstr)

    t_prev = t
f.close()

print('seq indices:')
print(seq_indices)
print('seq times:')
print(seq_times)

