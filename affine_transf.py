import numpy as np
import transformations as tf
def change_of_basis(initial, final):
    '''
    rotate points/vectors in a 3D coordinate system to a new coordinate system

        input: m x 3 array of points or vectors that have to be transformed from the initial to the final csys
        initial: sequence of sequences of floats representing the normalized axis of the csys that has to be transformed
        final: sequence of sequences of floats representing the normalized axis of the csys to which has to be transformed

        return: the points/vectors in the new coordinate system
    '''
    x1, y1, z1 = initial
    x1, y1, z1 = x1[:3], y1[:3], z1[:3]
    x2, y2, z2 = final
    x2, y2, z2 = x2[:3], y2[:3], z2[:3]

    M11, M12, M13 = np.dot(x1, x2), np.dot(x1, y2), np.dot(x1, z2)
    M21, M22, M23 = np.dot(y1, x2), np.dot(y1, y2), np.dot(y1, z2)
    M31, M32, M33 = np.dot(z1, x2), np.dot(z1, y2), np.dot(z1, z2)

    # set up rotation matrix
    R = np.array([[M11, M12, M13],
                  [M21, M22, M23],
                  [M31, M32, M33]])

    return R

mtc1 = np.array([   8.9370519  ,  53.41131023 , 812.70708433,   85.05061047 ,  19.84840919, -36.08782648])
elfin1 = np.array([ 731.294,  -33.206,  288.815, -168.287 ,  24.711, -154.658])

mtc2 = np.array([-112.78859538 ,  67.17476245,  909.26519042 ,  88.97403348 ,  22.08162782, -23.24631636])
elfin2 = np.array([ 602.067 ,-115.418 , 263.847 ,-167.323  , 26.239 ,-163.036])

mtc3 = np.array([  78.66768097,   76.03699824,  938.38222501,   77.77018979,   25.26409748,-43.57919163])
elfin3 = np.array([ 636.696,   74.973 , 254.392, -164.19 ,   30.844 ,-144.025])

mtc = np.array([mtc1[:3],mtc2[:3],mtc3[:3]])
elfin = np.array([elfin1[:3],elfin2[:3],elfin3[:3]])

#print(mtc.T, elfin.T)
m_change = tf.affine_matrix_from_points(elfin[:].T, mtc[:].T,
                                        shear=False, scale=False)
fids_mtc = np.vstack([mtc1,mtc2,mtc3])
fids_elfin = np.vstack([elfin1,elfin2,elfin3])


base_mtc_raw, q_mtc_raw = tf.base_creation(fids_mtc[:3, :3])
r_mtc_raw = np.identity(4)
r_mtc_raw[:3, :3] = base_mtc_raw[:3, :3]
t_mtc_raw = tf.translation_matrix(q_mtc_raw)
m_mtc_raw = tf.concatenate_matrices(t_mtc_raw, r_mtc_raw)

base_elfin_raw, q_elfin_raw = tf.base_creation(fids_elfin[:3, :3])
r_elfin_raw = np.identity(4)
r_elfin_raw[:3, :3] = base_elfin_raw[:3, :3]
t_elfin_raw = tf.translation_matrix(q_elfin_raw)
m_elfin_raw = tf.concatenate_matrices(t_elfin_raw, r_elfin_raw)


print(m_change)
trans = tf.translation_matrix(elfin1[:3])
a, b, g = np.radians(elfin1[3:6])
rot_elfin1 = tf.euler_matrix(a, b, g, 'rzyx')
m_elfin1 = tf.concatenate_matrices(trans, rot_elfin1)

trans = tf.translation_matrix(mtc1[:3])
a, b, g = np.radians(mtc1[3:6])
rot = tf.euler_matrix(a, b, g, 'rzyx')
m_mtc1 = tf.concatenate_matrices(trans, rot)


m_change2robot = np.linalg.inv(m_change) @ m_mtc1
scale, shear, angles, translate, perspective = tf.decompose_matrix(m_change2robot)
print(translate, np.degrees(angles))

rot_change_basis = change_of_basis(m_change2robot[:3,:3],rot_elfin1[:3,:3])
print(rot_change_basis)
m_changed = np.linalg.inv(rot_change_basis) @ m_change2robot[:3,:3]


trans = tf.translation_matrix(elfin2[:3])
a, b, g = np.radians(elfin2[3:6])
rot_elfin2 = tf.euler_matrix(a, b, g, 'rzyx')
m_elfin2 = tf.concatenate_matrices(trans, rot_elfin2)

trans = tf.translation_matrix(mtc2[:3])
a, b, g = np.radians(mtc2[3:6])
rot = tf.euler_matrix(a, b, g, 'rzyx')
m_mtc2 = tf.concatenate_matrices(trans, rot)


m_change2robot2 = np.linalg.inv(m_change) @ m_mtc2
scale, shear, angles, translate, perspective = tf.decompose_matrix(m_change2robot2)
print(translate, np.degrees(angles))

rot_change_basis2 = change_of_basis(m_change2robot2[:3,:3],rot_elfin2[:3,:3])
print(rot_change_basis2)
m_changed2 = np.linalg.inv(rot_change_basis2) @ m_change2robot2[:3,:3]


trans = tf.translation_matrix(elfin3[:3])
a, b, g = np.radians(elfin3[3:6])
rot_elfin3 = tf.euler_matrix(a, b, g, 'rzyx')
m_elfin3 = tf.concatenate_matrices(trans, rot_elfin3)

trans = tf.translation_matrix(mtc3[:3])
a, b, g = np.radians(mtc3[3:6])
rot = tf.euler_matrix(a, b, g, 'rzyx')
m_mtc3 = tf.concatenate_matrices(trans, rot)

m_change2robot3 = np.linalg.inv(m_change) @ m_mtc3
scale, shear, angles, translate, perspective = tf.decompose_matrix(m_change2robot3)
print(translate, np.degrees(angles))

rot_change_basis3 = change_of_basis(m_change2robot3[:3,:3],rot_elfin3[:3,:3])
print(rot_change_basis3)
m_changed3 = np.linalg.inv(rot_change_basis3) @ m_change2robot3[:3,:3]
m_change2robot3[:3, :3] = m_changed3[:3, :3]
scale, shear, angles, translate, perspective = tf.decompose_matrix(m_change2robot3)
print(translate, np.degrees(angles))


#
# print(m_mtc_raw)
#
#
# print(m_elfin_raw)

#plot
import matplotlib.pyplot as plt
import matplotlib
from mpl_toolkits.mplot3d import Axes3D
from pytransform3d.rotations import *
matplotlib.use('TkAgg')

ax = plot_basis(R=np.eye(3), ax_s=0.01)

p = np.array([0, 0, 0])
#rot_mat_rel[:3,3]
#R = matrix_from_euler_xyz(r.as_euler('xyz'))
plot_basis(ax, m_mtc1[:3,:3], m_mtc1[:3,3], s=100)
plot_basis(ax, m_elfin1[:3,:3],m_elfin1[:3,3]-10, s=100)
#plot_basis(ax, m_change2robot[:3,:3],m_change2robot[:3,3], s=100)
#plot_basis(ax, np.linalg.inv(m_changed[:3,:3]),m_change2robot[:3,3], s=100)
plot_basis(ax, m_changed[:3,:3],m_change2robot[:3,3], s=100)
plot_basis(ax, m_changed2[:3,:3],m_change2robot2[:3,3]+10, s=100)
plot_basis(ax, m_changed3[:3,:3],m_change2robot3[:3,3]+10, s=100)


ax.set_xlim([-300, 600])
ax.set_ylim([-300, 900])
ax.set_zlim([-300, 900])

plt.show()