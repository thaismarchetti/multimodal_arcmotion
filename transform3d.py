import numpy as np
import matplotlib.pyplot as plt
from pytransform3d import rotations as pr
from pytransform3d import transformations as pt
from pytransform3d.transform_manager import TransformManager

import transformations as tf


#random_state = np.random.RandomState(0)

# ee2robot = pt.transform_from_pq(
#     np.hstack((np.array([0.4, -0.3, 0.5]),
#                pr.random_quaternion(random_state))))
# cam2robot = pt.transform_from_pq(
#     np.hstack((np.array([0.0, 0.0, 0.8]), pr.q_id)))
# object2cam = pt.transform_from(
#     pr.active_matrix_from_intrinsic_euler_xyz(np.array([0.0, 0.0, -0.5])),
#     np.array([0.5, 0.1, 0.1]))

robot2trk = [[ 3.26006013e-01,  9.45273796e-01 , 1.33240445e-02, -2.02855718e+02],
             [ 9.70479041e-02, -1.94436575e-02, -9.95089769e-01,  2.69030767e+02],
             [-9.40373215e-01,  3.25698319e-01, -9.80755899e-02,  1.53869455e+03],
             [ 0.00000000e+00, 0.00000000e+00,  0.00000000e+00, 1.00000000e+00]]

probe2trk = pt.transform_from(
    pr.active_matrix_from_intrinsic_euler_zyx(np.array([88.97403348 ,  22.08162782, -23.24631636])),
    np.array([-112.78859538,  67.17476245,  909.26519042]))

ee2robot = pt.transform_from(
    pr.active_matrix_from_intrinsic_euler_zyx(np.array([-167.323  , 26.239 ,-163.036])),
    np.array([602.067 ,-115.418 , 263.847]))

R_ee2robot = np.vstack([ee2robot[0][:3],ee2robot[1][:3],ee2robot[2][:3]])


tm = TransformManager()
tm.add_transform("probe", "tracker", probe2trk)
tm.add_transform("robot", "tracker", robot2trk)
tm.add_transform("ee", "robot", ee2robot)
#tm.add_transform("object", "camera", object2cam)

probe2robot = tm.get_transform("probe", "robot")
print(probe2robot)
R_probe2robot = np.vstack([probe2robot[0][:3],probe2robot[1][:3],probe2robot[2][:3]])
print(R_probe2robot)

R_probe2ee = tf.inverse_matrix(R_probe2robot) @ R_ee2robot
# print(np.vstack([probe2robot[0][:3],probe2robot[1][:3],probe2robot[2][:3]]))
print(np.degrees(pr.euler_zyx_from_matrix(R_probe2robot)))
#tm.add_transform("prober", "robot", probe2robot)

quat_probe2robot = pr.quaternion_from_matrix(R_probe2robot)
quat_ee2robot = pr.quaternion_from_matrix(R_ee2robot)
quat_probe2ee = pr.quaternion_from_axis_angle(pr.quaternion_diff(quat_ee2robot, quat_probe2robot))
#print(quat_probe2ee)


# probe2ee = pt.transform_from_pq(
#     np.hstack((np.array([-1.64011846e+02, -6.48021741e+02, 1.57745991e+03]),
#                quat_probe2ee)))

# probe2ee = pt.transform_from_pq(
#     np.hstack((np.array([802.067 ,-115.418 , 263.847]),
#                pr.concatenate_quaternions(quat_ee2robot,quat_probe2robot))))

probe2ee = pt.transform_from(
    R_probe2ee,
    np.array([902.067 ,-115.418 , 263.847]))



tm.add_transform("prober", "robot", probe2ee)

ax = tm.plot_frames_in("robot", s=100)
#ax = tm.plot_connections_in("robot")
ax.set_xlim((-1200, 1200))
ax.set_ylim((-500, 500))
ax.set_zlim((-600.0, 600.0))
plt.show()