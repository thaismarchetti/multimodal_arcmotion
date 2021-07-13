import transformations as tf
import numpy as np


m_change = [[ 6.60777026e-01  ,1.27368851e-01 , 7.39696490e-01 , 1.02545622e+02],
             [-6.90178998e-01, -2.84225955e-01,  6.65483702e-01 , 1.91842882e+02],
             [ 2.95002836e-01, -9.50259324e-01, -9.99026749e-02 , 1.11287987e+02],
             [ 0.00000000e+00 , 0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]

mref = [[-6.37813904e-01 ,6.63839676e-02 ,-7.67324307e-01 , 7.45532674e+02],
         [ 7.65938050e-01 , 1.59218887e-01, -6.22887028e-01 ,-1.49483572e+02],
         [ 8.08228096e-02 ,-9.85008891e-01, -1.52398026e-01,  2.36833721e+02],
         [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00 , 1.00000000e+00]]

img_target = [111.35555768062464, 129.923394358876, 180.32488531460717,
              3.1732159651192897, 1.1034764747211379, -143.53082939948717]


robot_target = [698.037, -114.432, 313.449, -41.9, 60.249, -44.523]


r_obj = np.array([[-8.03106269e-01 , 5.94440030e-01 ,-4.07599311e-02 , 0],
         [-5.95479359e-01, -8.03111512e-01,  2.04017683e-02 ,0],
         [-2.06071421e-02,  4.06564856e-02 , 9.98960658e-01 ,0],
         [ 0.00000000e+00,  0.00000000e+00 , 0.00000000e+00 , 1.00000000e+00]])

r_obj_img = [[-0.91449147 , 0.36415976, -0.17633213 , 0.        ],
             [-0.12936968 ,-0.67610748, -0.72535657 , 0.        ],
             [-0.38336515, -0.64052037 , 0.66540576 , 0.        ],
             [ 0.     ,     0.     ,     0.         , 1.        ]]

m_obj_raw = [[3.31250503e-01, - 1.38156820e-01 ,- 9.33373343e-01 , 2.03967288e+01],
            [-8.73880195e-01 ,3.28122245e-01 ,- 3.58704887e-01 ,- 4.16668577e+00],
            [3.55818084e-01 , 9.34477654e-01, - 1.20418497e-02, - 4.19185678e+01],
            [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]

s0_dyn =    [[0.18293028 ,  0.71972262 ,- 0.6697282  , 64.81036668],
            [0.80953179, 0.27626952,0.5180091, - 33.73787388],
            [0.55784835, - 0.63692582 ,- 0.53210028 , 29.50654894],
            [0.,0.,0.,1.]]

m_probe_ref = np.array([[-5.56350055e-01  ,6.78057253e-01 ,- 4.80325908e-01 , 6.83750149e+01],
                [8.28410205e-01, 4.97740381e-01, - 2.56887223e-01, - 4.77494999e+01],
                [6.48933557e-02, - 5.40826104e-01, - 8.38627436e-01, - 4.08349210e+01],
                [0.00000000e+00,0.00000000e+00 ,0.00000000e+00 ,1.00000000e+00]])

s0_raw = np.array([[-4.89259429e-01 , 4.78629129e-02 , 8.70823951e-01,  7.24077000e+02],
        [-7.84192141e-02, 9.92034120e-01, - 9.85836260e-02, - 8.63810000e+01],
        [-8.68605571e-01, - 1.16522298e-01, - 4.81608675e-01,  2.79802000e+02],
        [0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00]])

t_offset = [[1.     ,      0.     ,      0.       ,   43.61119537],
            [0.,1.,0., - 12.97811589],
            [0.   ,       0.    ,       1., - 10.96295089],
            [0.,0.,0.,1.]]

t_probe_raw = [[1.,     0.,      0.,     698.037],
                [0., 1., 0., - 114.431],
                [0.,   0.,      1.,    313.446],
                [0., 0., 0., 1.]]



trans = tf.translation_matrix(img_target[:3])
a, b, g = np.radians(img_target[3:6])
rot = tf.euler_matrix(a, b, g, 'rzyx')
M_image_target = tf.concatenate_matrices(trans, rot)
invM_image_target = tf.inverse_matrix(M_image_target)

print(robot_target)
trans = tf.translation_matrix(robot_target[:3])
a, b, g = np.radians(robot_target[3:6])
rot_robot = tf.euler_matrix(a, b, g, 'rzyx')
M_robot = tf.concatenate_matrices(trans, rot_robot)
#print(tf.inverse_matrix(mimg) @ M_image_target)
# m_robot = mref @ np.linalg.inv(m_change) @ M_image_target
m_img2robotref = np.linalg.inv(m_change) @ M_image_target
m_img2robotref[2, -1] = -m_img2robotref[2, -1]

m_robotref2robotraw = mref @ m_img2robotref


#Offset to coil
#m_robot = np.linalg.inv(s0_raw @ t_offset @ np.linalg.inv(s0_raw)) @ m_robot ## igual linha de baixo
m_robot = s0_raw @ np.linalg.inv(t_offset) @ np.linalg.inv(s0_raw) @ m_robotref2robotraw
scale, shear, angles, translate, perspective = tf.decompose_matrix(m_robot)
print(translate, np.degrees(angles))

scale, shear, angles, translate_ref, perspective = tf.decompose_matrix(mref)
print(translate_ref, np.degrees(angles))

teste = np.linalg.inv(m_obj_raw) @ np.linalg.inv(s0_dyn) @ m_robotref2robotraw @ m_obj_raw
scale, shear, angles, translate, perspective = tf.decompose_matrix(teste)
print(translate, np.degrees(angles))


scale, shear, angles, translate, perspective = tf.decompose_matrix(m_robot)
trans = tf.translation_matrix(translate)
M_target_robot = tf.concatenate_matrices(trans, rot_robot)

M_robot_ref = np.linalg.inv(mref) @ M_robot
scale, shear, angles, translate, perspective = tf.decompose_matrix(M_robot_ref)
print(translate, np.degrees(angles))
import pytransform3d.rotations as pr
euler_xyz_intrinsic_active_radians = pr.euler_xyz_from_matrix(M_robot_ref[:3,:3].T)
print(np.rad2deg(euler_xyz_intrinsic_active_radians))


teste = np.linalg.inv(M_robot) @ mref
scale, shear, angles, translate, perspective = tf.decompose_matrix(teste)
print(translate, np.degrees(angles))

# teste = mref @ m_robot
# scale, shear, angles, translate, perspective = tf.decompose_matrix(teste)
# print(translate, np.degrees(angles))

'''   
r_obj = r_obj_img @ np.linalg.inv(m_obj_raw) @ np.linalg.inv(s0_dyn) @ m_probe_ref @ m_obj_raw
m_img[:3, :3] = r_obj[:3, :3]
'''
#print(m_change @ M_image_target)
#print(tf.inverse_matrix(m_change) @ tf.inverse_matrix(mimg) @ M_image_target)
#r_obj = r_obj_img @ np.linalg.inv(m_obj_raw) @ np.linalg.inv(s0_dyn) @ m_probe_ref @ m_obj_raw
#r_obj =np.linalg.inv(r_obj_img) @ m_obj_raw @ s0_dyn @ r_obj @ np.linalg.inv(m_obj_raw)

# inv = np.linalg.inv(m_obj_raw) @ np.linalg.inv(s0_dyn) @ np.linalg.inv(rot_robot) @ m_obj_raw
# scale, shear, angles, translate, perspective = tf.decompose_matrix(inv)
# print('oi',translate, np.degrees(angles))
#
# print(m_probe_ref)
# scale, shear, angles, translate, perspective = tf.decompose_matrix(m_probe_ref)
# print(translate, np.degrees(angles))
# print('\n')
# rot1 = np.linalg.inv(m_obj_raw) @ np.linalg.inv(s0_dyn) @ m_probe_ref @ m_obj_raw
# M1 = r_obj_img @ rot1
# print(M1)
# scale, shear, angles, translate, perspective = tf.decompose_matrix(M1)
# print(translate, np.degrees(angles))
# M2 = np.linalg.inv(r_obj_img) @ M1
# print(M2)
# scale, shear, angles, translate, perspective = tf.decompose_matrix(M2)
# print(translate, np.degrees(angles))
#
# print(rot1)
#print(np.linalg.inv(r_obj_img) @ np.linalg.inv(m_obj_raw) @ r_obj @ m_obj_raw)

'''
r_obj [[-8.03106269e-01  5.94440030e-01 -4.07599311e-02  3.98047050e+01]
 [-5.95479359e-01 -8.03111512e-01  2.04017683e-02 -5.34275311e+01]
 [-2.06071421e-02  4.06564856e-02  9.98960658e-01 -2.95169825e+01]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
r_obj_img [[-0.91449147  0.36415976 -0.17633213  0.        ]
 [-0.12936968 -0.67610748 -0.72535657  0.        ]
 [-0.38336515 -0.64052037  0.66540576  0.        ]
 [ 0.          0.          0.          1.        ]]
 m_obj_raw [[ 3.31250503e-01 -1.38156820e-01 -9.33373343e-01  2.03967288e+01]
 [-8.73880195e-01  3.28122245e-01 -3.58704887e-01 -4.16668577e+00]
 [ 3.55818084e-01  9.34477654e-01 -1.20418497e-02 -4.19185678e+01]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
 s0_dyn [[  0.18293028   0.71972262  -0.6697282   64.81036668]
 [  0.80953179   0.27626952   0.5180091  -33.73787388]
 [  0.55784835  -0.63692582  -0.53210028  29.50654894]
 [  0.           0.           0.           1.        ]]
 m_probe_ref [[-5.56350055e-01  6.78057253e-01 -4.80325908e-01  6.83750149e+01]
 [ 8.28410205e-01  4.97740381e-01 -2.56887223e-01 -4.77494999e+01]
 [ 6.48933557e-02 -5.40826104e-01 -8.38627436e-01 -4.08349210e+01]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
 
 s0_raw [[-4.89259429e-01  4.78629129e-02  8.70823951e-01  7.24077000e+02]
 [-7.84192141e-02  9.92034120e-01 -9.85836260e-02 -8.63810000e+01]
 [-8.68605571e-01 -1.16522298e-01 -4.81608675e-01  2.79802000e+02]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
 t_offset [[  1.           0.           0.          43.61119537]
 [  0.           1.           0.         -12.97811589]
 [  0.           0.           1.         -10.96295089]
 [  0.           0.           0.           1.        ]]
 t_probe_raw [[   1.       0.       0.     698.037]
 [   0.       1.       0.    -114.431]
 [   0.       0.       1.     313.446]
 [   0.       0.       0.       1.   ]]
 
 mchange [[ 6.60777026e-01  1.27368851e-01  7.39696490e-01  1.02545622e+02]
 [-6.90178998e-01 -2.84225955e-01  6.65483702e-01  1.91842882e+02]
 [ 2.95002836e-01 -9.50259324e-01 -9.99026749e-02  1.11287987e+02]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
 
 m_ref  [[-6.37813904e-01  6.63839676e-02 -7.67324307e-01  7.45532674e+02]
 [ 7.65938050e-01  1.59218887e-01 -6.22887028e-01 -1.49483572e+02]
 [ 8.08228096e-02 -9.85008891e-01 -1.52398026e-01  2.36833721e+02]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]

coord [111.35555768062464, 129.923394358876, 180.32488531460717, 3.1732159651192897, 1.1034764747211379, -143.53082939948717, 1.0, 1.0, 0.0, 2, 'x', 0, 0, 0, -41.901, 60.249, -44.524] 

M_coord_target[[ 9.98281580e-01  3.30857339e-02 -4.83655016e-02  1.11355558e+02]
 [ 5.53444924e-02 -8.03577436e-01  5.92621542e-01  1.29923394e+02]
 [-1.92581071e-02 -5.94279933e-01 -8.04027665e-01  1.80324885e+02]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]] 

M_probe_raw [[-9.80181363e-01  1.44696583e-01  1.35304817e-01  7.30376030e+02]
 [ 9.14947879e-03  7.15342206e-01 -6.98714402e-01 -7.83041583e+01]
 [-1.97890833e-01 -6.83628867e-01 -7.02488997e-01  2.94935255e+02]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]

[698.037, -114.432, 313.449, -41.9, 60.249, -44.523]
(730.376030178341, -78.30415832468836, 294.93525538206455, 179.46518949027828, 11.413647702666305, -135.77954350690476)
Error code:  1044
'''