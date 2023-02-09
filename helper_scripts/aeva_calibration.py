import os
import os.path as osp
import numpy as np
from pyboreas.utils.odometry import read_traj_file, read_traj_file_gt

from pylgmath import Transformation, se3op
from pysteam.problem import OptimizationProblem, StaticNoiseModel, L2LossFunc, WeightedLeastSquareCostTerm
from pysteam.solver import GaussNewtonSolver
from pysteam.evaluable import se3 as se3ev
from pysteam.evaluable import Evaluable, Node, Jacobians
from pysteam.evaluable.se3 import SE3StateVar, compose_velocity, compose_rinv, compose, tran2vec
from pysteam.evaluable.vspace import VSpaceStateVar

np.set_printoptions(precision=6, suppress=True)


def get_inverse_tf(T):
  """Returns the inverse of a given 4x4 homogeneous transform.
    Args:
        T (np.ndarray): 4x4 transformation matrix
    Returns:
        np.ndarray: inv(T)
    """
  T2 = T.copy()
  T2[:3, :3] = T2[:3, :3].transpose()
  T2[:3, 3:] = -1 * T2[:3, :3] @ T2[:3, 3:]
  return T2


seq = 'boreas-2022-05-13-10-30'
gt_dir = '/home/yuchen/ASRL/data/boreas/sequences'
pred_dir = '/media/yuchen/T7/ASRL/temp/lidar/aeva/boreas-2022-05-13-10-30.icp.rv/odometry_result/'

# load predictions
pred_T_vi, _ = read_traj_file(osp.join(pred_dir, seq + ".txt"))  # T_applanix_world
print(len(pred_T_vi))

# T_gt_pred = np.array([[ 0.995621,  0.002137,  0.09346 ,  0.002811],
#                       [-0.003235,  0.999928,  0.011597, -0.04655 ],
#                       [-0.093429, -0.011848,  0.995555,  0.128853],
#                       [ 0.      ,  0.      ,  0.      ,  1.      ]])
# for i in range(len(pred_T_vi)):
#   pred_T_vi[i] = T_gt_pred @ pred_T_vi[i]

pred_T_v_vp1 = []
for i in range(len(pred_T_vi) - 1):
  tmp = pred_T_vi[i] @ get_inverse_tf(pred_T_vi[i + 1])
  tmp = SE3StateVar(Transformation(T_ba=tmp), locked=True)
  pred_T_v_vp1.append(tmp)

# load ground truth poses
filepath = os.path.join(gt_dir, seq, 'applanix/aeva_poses.csv')  # use 'lidar_poses.csv' for groundtruth
T_calib = np.loadtxt(os.path.join(gt_dir, seq, 'calib/T_applanix_aeva.txt'))
gt_T_vi, _ = read_traj_file_gt(filepath, T_calib, 3)
print(len(gt_T_vi))

gt_T_v_vp1 = []
for i in range(len(gt_T_vi) - 1):
  tmp = gt_T_vi[i] @ get_inverse_tf(gt_T_vi[i + 1])
  tmp = SE3StateVar(Transformation(T_ba=tmp), locked=True)
  gt_T_v_vp1.append(tmp)

###############################################################################################

T_ab = SE3StateVar(Transformation(T_ba=np.eye(4)))  #   T_ab = T gt pred

noise_model = StaticNoiseModel(np.eye(6))
loss_func = L2LossFunc()
cost_terms = []
for i in range(len(pred_T_v_vp1) - 1):
  est_gt_T_v_vp1 = compose_rinv(compose(T_ab, pred_T_v_vp1[i]), T_ab)
  error_func = tran2vec(compose_rinv(est_gt_T_v_vp1, gt_T_v_vp1[i]))
  # print(error_func.evaluate())
  cost_terms.append(WeightedLeastSquareCostTerm(error_func, noise_model, loss_func))

opt_prob = OptimizationProblem()
opt_prob.add_state_var(T_ab)
opt_prob.add_cost_term(*cost_terms)

gauss_newton = GaussNewtonSolver(opt_prob, verbose=True, max_iterations=100)
gauss_newton.optimize()

print(T_ab.value)