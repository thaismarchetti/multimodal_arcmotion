import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pytransform3d.rotations import *
import transformations as tf

robot_target = [698.037, -114.432, 313.449, -41.9, 60.249, -44.523]
trans = tf.translation_matrix(robot_target[:3])
a, b, g = np.radians(robot_target[3:6])
rot_robot = tf.euler_matrix(a, b, g, 'rzyx')
#print(rot_robot)
M_robot = tf.concatenate_matrices(trans, rot_robot)

mref = np.array([[-6.37813904e-01 ,6.63839676e-02 ,-7.67324307e-01 , 7.45532674e+02],
         [ 7.65938050e-01 , 1.59218887e-01, -6.22887028e-01 ,-1.49483572e+02],
         [ 8.08228096e-02 ,-9.85008891e-01, -1.52398026e-01,  2.36833721e+02],
         [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00 , 1.00000000e+00]])

mref_current = np.array([[-5.37813904e-01 ,6.63839676e-02 ,-7.67324307e-01 , 7.45532674e+02],
         [ 7.65938050e-01 , 0.59218887e-01, -6.22887028e-01 ,-1.49483572e+02],
         [ 8.08228096e-02 ,-9.85008891e-01, -0.52398026e-01,  2.36833721e+02],
         [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00 , 1.00000000e+00]])

s0_raw = np.array([[-4.89259429e-01 , 4.78629129e-02 , 8.70823951e-01,  7.24077000e+02],
        [-7.84192141e-02, 9.92034120e-01, - 9.85836260e-02, - 8.63810000e+01],
        [-8.68605571e-01, - 1.16522298e-01, - 4.81608675e-01,  2.79802000e+02],
        [0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00]])

angles_ref_current = [-114.55958108  , -3.07411591  ,129.24016154]
# tem que inverter x e z no angles_ref_target
angles_ref_target = [ -92.76449782, -3.38738992,129.34653241 ]
angles_robot_target = [-49.671, 56.771, -51.508]
#angles_robot_current = [-69.671, 46.771, -61.508]

psi_target, theta_target, phi_target = np.radians(angles_robot_target)
rot_target = tf.euler_matrix(psi_target, theta_target, phi_target, 'rzyx')

# psi_robot_current, theta_robot_current, phi_robot_current = np.radians(angles_robot_current)
# rot_robot_current = tf.euler_matrix(psi_robot_current, theta_robot_current, phi_robot_current, 'rzyx')

diff_angles = ((np.array(angles_ref_current)) - (np.array(angles_ref_target)))
print(diff_angles)
a, b, g = np.radians(diff_angles)
rot_ref_dif = tf.euler_matrix(a, b, g, 'rzyx')
#rot_correct = s0_raw @ rot_ref_dif @ np.linalg.inv(s0_raw) @ rot_target
rot_correct = rot_ref_dif  @ rot_target
_, _, angles, _, _ = tf.decompose_matrix(rot_correct)
psi, theta, phi = np.degrees(angles)
print(psi, theta, phi)

print(phi - angles_robot_target[0], theta - angles_robot_target[1], psi - angles_robot_target[2])


# Calculate the relative rotation matrix from t0 to t1
rot_mat_rel = np.matmul(np.transpose(M_robot), mref)
#print(rot_mat_rel)
rot_mat_rel = np.linalg.inv(M_robot) @ mref
#print(rot_mat_rel)

scale, shear, angles_ref, translate, perspective = tf.decompose_matrix(mref)
print(translate, np.degrees(angles_ref))
scale, shear, angles_ref_current, translate, perspective = tf.decompose_matrix(mref_current)
print(translate, np.degrees(angles_ref_current))

mat = ((np.array(np.degrees(angles_ref_current))) - np.array(np.degrees(angles_ref)))
print('mat', mat)
trans = tf.translation_matrix(robot_target[:3])
a, b, g = np.radians(mat)
#a, b, g = np.radians(mat)
rot_ref_dif = tf.euler_matrix(a, b, g, 'rzyx')
print(rot_ref_dif)
M_ref_dif = tf.concatenate_matrices(trans, rot_ref_dif)

mat = rot_ref_dif @ rot_robot
print(mat)
scale, shear, angles, translate, perspective = tf.decompose_matrix(mat)
psi, theta, phi = np.degrees(angles)
print(psi, theta, phi)
print(translate, np.degrees(angles))
#y'_orientation = mat(x'_orientation - x_orientation)*mat(y_orientation)

ax = plot_basis(R=np.eye(3), ax_s=0.01)

p = np.array([0, 0, 0])
#rot_mat_rel[:3,3]
#R = matrix_from_euler_xyz(r.as_euler('xyz'))
plot_basis(ax, rot_mat_rel[:3,:3], rot_mat_rel[:3,3], s=100)
plot_basis(ax, M_robot[:3,:3],M_robot[:3,3], s=100)
plot_basis(ax, mat[:3,:3],M_robot[:3,3], s=100)
plot_basis(ax, mref[:3,:3], mref[:3,3], s=100)
ax.set_xlim([-100, 600])
ax.set_ylim([-300, 500])
ax.set_zlim([-300, 200])

plt.show()