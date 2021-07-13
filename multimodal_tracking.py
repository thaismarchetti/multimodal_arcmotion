import transformations as tf
import numpy as np


#robot

#probe_elfin = np.array([ 666.532,   35.820,   313.686,   -179.837,  21.813, -174.970])
probe_elfin = np.array([ 728.737 , -20.169,  62.335, -174.517 ,  27.822, -171.305])
#probe_plh = np.array([ 413.4,   300.1,   512.8,   -51.66,  -67.32, -176.11])
#probe_plh[3:] = np.deg2rad(probe_plh[3:])
trans = tf.translation_matrix(probe_elfin[:3])
a, b, g = np.radians(probe_elfin[3:6])
rot = tf.euler_matrix(a, b, g, 'rzyx')
M_probe_plh = tf.concatenate_matrices(-trans, rot)
invM_probe_plh = tf.inverse_matrix(M_probe_plh)

#robot
#probe_mtc = [48.06056186,  32.26243672, 725.13052259, 79.13542604, 12.26369149, -18.33794281]
#probe_mtc = np.array([ -0.91355045 , 73.71262114 ,754.01792716 , 74.943451 ,    5.34604425, -74.30276042])
probe_mtc = np.array([-112.78859538 ,  67.17476245,  909.26519042 ,  88.97403348 ,  22.08162782, -23.24631636])
#probe_mtc = np.array([133.50128728, 115.09843656, 900.91982109,  97.84440177, -15.34498326, 47.85826012])
probe_mtc[3:] = np.deg2rad(probe_mtc[3:])
trans = tf.translation_matrix(probe_mtc[:3])
a, b, g = probe_mtc[3:6]
#a, b, g = np.radians(probe_mtc[3:6])
rot = tf.euler_matrix(a, b, g, 'rzyx')
M_probe_mtc = tf.concatenate_matrices(trans, rot)
invM_probe_mtc = tf.inverse_matrix(M_probe_mtc)


#m_change = tf.affine_matrix_from_points(elfin[:], mtc[:],
m_change = [[ 3.26006013e-01,  9.45273796e-01 , 1.33240445e-02, -2.02855718e+02],
             [ 9.70479041e-02, -1.94436575e-02, -9.95089769e-01,  2.69030767e+02],
             [-9.40373215e-01,  3.25698319e-01, -9.80755899e-02,  1.53869455e+03],
             [ 0.00000000e+00 , 0.00000000e+00,  0.00000000e+00 , 1.00000000e+00]]
#m_change = tf.affine_matrix_from_points(mtc[:], elfin[:],
# m_change =   [[ 4.52164779e-01 , 5.22983860e-01, -7.22519823e-01 , 1.23390471e+03],
#              [ 8.85246700e-01 ,-1.64125713e-01 , 4.35202288e-01, -3.42694277e+02],
#              [ 1.09019691e-01, -8.36391435e-01, -5.37181603e-01 , 5.29001175e+02],
#              [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00 , 1.00000000e+00]]



#invM_probe_plh = tf.inverse_matrix(tf.compose_matrix(angles=probe_plh[3:], translate=probe_plh[:3]))
#invM_probe_mtc = tf.inverse_matrix(tf.compose_matrix(angles=probe_mtc[3:], translate=probe_mtc[:3]))

#M_probe_plh = tf.compose_matrix(angles=probe_plh[3:], translate=probe_plh[:3])
#M_probe_mtc = tf.compose_matrix(angles=probe_mtc[3:], translate=probe_mtc[:3])

probe_plh_test = np.array([ 666.532,   35.820,   313.686,   -179.837,  21.813, -174.970])
#probe_plh_test = np.array([ 716.916,   18.842,   61.245, -175.362 ,  23.223, -168.643])
#probe_plh[3:] = np.deg2rad(probe_plh[3:])
trans = tf.translation_matrix(probe_plh_test[:3])
a, b, g = np.radians(probe_plh_test[3:6])
rot = tf.euler_matrix(a, b, g, 'rzyx')
M_probe_plh_test = tf.concatenate_matrices(trans, rot)
invM_probe_plh_test = tf.inverse_matrix(M_probe_plh_test)

probe_mtc_test = np.array([   8.9370519  ,  53.41131023 , 812.70708433,   85.05061047 ,  19.84840919, -36.08782648])
#probe_plh[3:] = np.deg2rad(probe_plh[3:])
trans = tf.translation_matrix(probe_mtc_test[:3])
a, b, g = np.radians(probe_mtc_test[3:6])
rot = tf.euler_matrix(a, b, g, 'rzyx')
M_probe_mtc_test = tf.concatenate_matrices(trans, rot)
invM_probe_mtc_test = tf.inverse_matrix(M_probe_mtc_test)

print(invM_probe_plh)
print(invM_probe_mtc)
print(M_probe_plh)

# M_plh_in_mtc = M_probe_mtc @ invM_probe_plh
# print('calib matrix:')
# print(M_plh_in_mtc)
# M_plh_in_mtc = M_plh_in_mtc @ M_probe_plh
# #probe_plh_in_mtc = M_plh_in_mtc @ M_probe_plh_test
# scale, shear, angles, translate, perspective = tf.decompose_matrix(M_plh_in_mtc)
# print(translate, np.degrees(angles))

probe_plh_in_mtc = tf.inverse_matrix(m_change) @ M_probe_mtc
scale, shear, angles, translate, perspective = tf.decompose_matrix(probe_plh_in_mtc)
print(translate, np.degrees(angles))

# probe_mtc_in_plh = tf.inverse_matrix(m_change) @ M_probe_mtc_test
# scale, shear, angles, translate, perspective = tf.decompose_matrix(probe_mtc_in_plh)
# print(translate, np.degrees(angles))
