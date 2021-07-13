
import numpy as np
import transformations as tf
from time import sleep, time

import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
#from pytransform3d.rotations import *
#matplotlib.use('TkAgg')
import stabilizer

def ClaronTracker():
    import sys
    FS_ENCODE = sys.getfilesystemencoding()
    calib  = "D:\\Thais\movv\\rmatsuda1-multimodal_tracking-e386f01e1071\\rmatsuda1-multimodal_tracking-e386f01e1071\\mtc_files\\CalibrationFiles"
    markers = "D:\\Thais\movv\\rmatsuda1-multimodal_tracking-e386f01e1071\\rmatsuda1-multimodal_tracking-e386f01e1071\\mtc_files\\Markers"
    #MTC_PROBE_NAME = "1Probe"
    MTC_REF_NAME = "2Ref"
    MTC_OBJ_NAME = "3bigcoil"

    import pyclaron

    lib_mode = 'wrapper'
    trck_init = pyclaron.pyclaron()
    trck_init.CalibrationDir = calib.encode(FS_ENCODE)
    trck_init.MarkerDir = markers.encode(FS_ENCODE)
    trck_init.NumberFramesProcessed = 10
    trck_init.FramesExtrapolated = 0
    # trck_init.PROBE_NAME = MTC_PROBE_NAME.encode(FS_ENCODE)
    trck_init.REF_NAME = MTC_REF_NAME.encode(FS_ENCODE)
    trck_init.OBJ_NAME = MTC_OBJ_NAME.encode(FS_ENCODE)
    trck_init.Initialize()


    if trck_init.GetIdentifyingCamera():
        trck_init.Run()
        print("MicronTracker camera identified.")
    else:
        trck_init = None


    return trck_init

def ElfinRobot():
    try:
        import elfin_robot as elfin
        #trck_init = elfin.elfin_server('169.254.229.243', 10003)
        trck_init = elfin.elfin_server('169.254.153.251', 10003)
        #trck_init = elfin.elfin_server('127.0.0.1', 10003)Initialize
        trck_init.Initialize()
        lib_mode = 'wrapper'
        print('Connect to elfin robot tracking device.')

    except:
        lib_mode = 'disconnect'
        trck_init = None
        print('Could not connect to default tracker.')

    # return tracker initialization variable and type of connection
    return trck_init

def ElfinCoord(trck_init):
    trck = trck_init
    trck.Run()
    probe = trck.Run()
    probe[3], probe[5] = probe[5], probe[3]
    ref = [0,0,0,0,0,0]
    coord = np.vstack([probe, ref])

    return coord

def ClaronCoord(trck_init):
    trck = trck_init
    trck.Run()
    scale = np.array([1.0, 1.0, 1.0])

    coord1 = np.array([float(trck.PositionTooltipX1)*scale[0], float(trck.PositionTooltipY1)*scale[1],
                      float(trck.PositionTooltipZ1)*scale[2],
                      float(trck.AngleZ1), float(trck.AngleY1), float(trck.AngleX1)])

    coord2 = np.array([float(trck.PositionTooltipX2)*scale[0], float(trck.PositionTooltipY2)*scale[1],
                       float(trck.PositionTooltipZ2)*scale[2],
                       float(trck.AngleZ2), float(trck.AngleY2), float(trck.AngleX2)])

    coord3 = np.array([float(trck.PositionTooltipX3) * scale[0], float(trck.PositionTooltipY3) * scale[1],
                       float(trck.PositionTooltipZ3) * scale[2],
                       float(trck.AngleZ3), float(trck.AngleY3), float(trck.AngleX3)])

    coord = np.vstack([coord1, coord2, coord3])
    flag = [trck.refID,trck.coilID]
    print('flag_markers', flag)

    return coord, flag

def transform_tracker_2_robot(tracker_coord, M_tracker_2_robot):
    trans = tf.translation_matrix(tracker_coord[:3])
    a, b, g = np.radians(tracker_coord[3:6])
    rot = tf.euler_matrix(a, b, g, 'rzyx')
    M_tracker = tf.concatenate_matrices(trans, rot)
    M_tracker_in_robot = M_tracker_2_robot @ M_tracker
    #print('tracker_coord', tracker_coord)

    _, _, angles, translate, _ = tf.decompose_matrix(M_tracker_in_robot)
    tracker_in_robot = [translate[0], translate[1], translate[2],\
            np.degrees(angles[2]), np.degrees(angles[1]), np.degrees(angles[0])]
    #print('tracker_in_robot', tracker_in_robot)

    return tracker_in_robot, M_tracker_in_robot

def affine_correg(tracker, robot):
    m_change = tf.affine_matrix_from_points(robot[:].T, tracker[:].T,
                                            shear=False, scale=False, usesvd=False)
    return m_change

def run_coords(trck_init_mtc, trck_init_robot):
    coord_tracker, flag_markers = ClaronCoord(trck_init_mtc)
    coord_robot = ElfinCoord(trck_init_robot)
    return coord_tracker, flag_markers, coord_robot

def transform_pos_2_matrix(coord):
    trans = tf.translation_matrix(coord[:3])
    a, b, g = np.radians(coord[3:6])
    rot = tf.euler_matrix(a, b, g, 'rzyx')
    return tf.concatenate_matrices(trans, rot)

def yes_or_no(question):
    while "the answer is invalid":
        reply = str(input(question+' (y/n): ')).lower().strip()
        if reply[:1] == 'y':
            return True
        if reply[:1] == 'n':
            return False

def estimate_head_velocity(coord_vel, timestamp):
    distance = []
    velocity = []
    coord_vel = np.vstack(np.array(coord_vel))
    print('time',(timestamp[-1]-timestamp[0]))
    coord_init = coord_vel[:int(len(coord_vel)/2)].mean(axis=0)
    coord_final = coord_vel[int(len(coord_vel)/2):].mean(axis=0)
    velocity = (coord_final - coord_init)/(timestamp[-1]-timestamp[0])
    distance = (coord_final - coord_init)

    # for i in range(len(coord_vel)-2):
    #     distance.append(coord_vel[i + 2] - coord_vel[i])
    #     velocity.append((coord_vel[i+2] - coord_vel[i])/(timestamp[i+2]-timestamp[i]))
    return velocity, distance

def MoveC(self, target):
        """
        function: Arc motion
        :param: Through position[X,Y,Z],GoalCoord[X,Y,Z,RX,RY,RZ],Type[0 or 1],;
        :return:
        """
        target = [str(s) for s in target]
        target = (",".join(target))
        message = "MoveC," + self.rbtID + ',' + target + self.end_msg
        return self.send(message)

def Versores(init_point, final_point):
    vector = (final_point[0]-init_point[0],final_point[1]-init_point[1],final_point[2]-init_point[2])
    somaq= vector[0]**2+vector[1]**2+vector[2]**2
    norma = pow(somaq,0.5)
    versor=(vector[0]/norma,vector[1]/norma,vector[2]/norma)
    versorfator=(versor[0]*100,versor[1]*50,versor[2]*100)
    return versorfator

flag_affine = False
flag_settarget = True
flag_tracker = True
flag_plot = False
flag_velo = True
flag_kalman = True

trck_init_mtc = ClaronTracker()
trck_init_robot = ElfinRobot()

coord_tracker, flag_markers, coord_robot = run_coords(trck_init_mtc, trck_init_robot)

if flag_affine:
    mtc_coord = []
    mtc_angles = []
    robot_coord = []
    robot_angles = []
    while len(mtc_coord)<300:
        #input_text = input('input something to collect the next point: ')
        coord_tracker, flag_markers, coord_robot = run_coords(trck_init_mtc, trck_init_robot)

        if flag_markers[1]:
            mtc_coord.append(coord_tracker[2][:3])
            mtc_angles.append(coord_tracker[2][3:])
            robot_coord.append(coord_robot[0][:3])
            robot_angles.append(coord_robot[0][3:])
            #print(mtc, robot)
        else:
            print('Cannot detect the coil markers, pls try again')

        # if input_text == "exit":
        #     trck_init_mtc.Close()
        #     break
    input_text = input('input something to continue: ')
    mtc_coord = np.array(mtc_coord)
    robot_coord = np.array(robot_coord)

    M_robot_2_tracker = affine_correg(mtc_coord, robot_coord)
    print("M_robot_2_tracker", M_robot_2_tracker)

else:
    # M_robot_2_tracker =[[7.08856322e-02 , 9.97182726e-01, - 2.45323877e-02 ,- 1.66223249e+02],
    #                      [-1.48584842e-02, - 2.35359372e-02 ,- 9.99612568e-01,  3.62543320e+02],
    #                     [-9.97373778e-01, 7.12226829e-02, 1.31482640e-02, 1.66368389e+03],
    #                     [0.00000000e+00  ,0.00000000e+00 , 0.00000000e+00 , 1.00000000e+00]]
    M_robot_2_tracker = [[ 3.10604415e-01, 9.50346610e-01,  1.91368150e-02, -1.30087200e+02],
                        [ 3.05885228e-02,  1.01288811e-02, -9.99480739e-01,  2.92404288e+02],
                        [-9.50046967e-01,  3.11028497e-01, -2.59236238e-02,  1.96741466e+03],
                        [ 0.00000000e+00 , 0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]
    # M_robot_2_tracker = [[ 3.41541979e-01 , 9.39589953e-01,  2.27990560e-02, -3.39484527e+02],
    #                      [ 8.40244849e-02 ,-6.36439415e-03, -9.96443365e-01,  2.28264256e+02],
    #                      [-9.36103073e-01 , 3.42242918e-01, -8.11222696e-02,  1.65836850e+03],
    #                      [ 0.00000000e+00 , 0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]

M_tracker_2_robot = tf.inverse_matrix(M_robot_2_tracker)

#setting Targets 1 and 2 (two different points of stimulation)
if flag_settarget:
    m_change_robot2ref_target1 = None
    while m_change_robot2ref_target1 is None:
        coord_tracker, flag_markers, coord_robot = run_coords(trck_init_mtc, trck_init_robot)
        print('detecting ref:', flag_markers)
        ref_tracker_in_robot, M_ref_tracker_in_robot = transform_tracker_2_robot(coord_tracker[1], M_tracker_2_robot)
        m_robot = transform_pos_2_matrix(coord_robot[0])
        if flag_markers[0]:
            if yes_or_no('Would you like to save the current position as target1?'):
                change = np.linalg.inv(M_ref_tracker_in_robot) @ m_robot
                scale, shear, angles, translate, perspective = tf.decompose_matrix(change)
                print(change)
                m_change_robot2ref_target1 = change

    m_change_robot2ref_target2 = None
    while m_change_robot2ref_target2 is None:
        coord_tracker, flag_markers, coord_robot = run_coords(trck_init_mtc, trck_init_robot)
        print('detecting ref:', flag_markers)
        ref_tracker_in_robot, M_ref_tracker_in_robot = transform_tracker_2_robot(coord_tracker[1], M_tracker_2_robot)
        m_robot = transform_pos_2_matrix(coord_robot[0])
        if flag_markers[0]:
            if yes_or_no('Would you like to save the current position as target2?'):
                change = np.linalg.inv(M_ref_tracker_in_robot) @ m_robot
                scale, shear, angles, translate, perspective = tf.decompose_matrix(change)
                print(change)
                m_change_robot2ref_target2 = change
        # MANUAL TARGET INPUT'''


else:
    # #Target feeler
    # m_change_robot2ref_target =[[ 2.27302892e-02, -1.02499556e-01, -9.94473315e-01 , 1.26389970e+01],
    #                      [-9.61355850e-01, -2.75234510e-01,  6.39486100e-03 , 1.85740619e+02],
    #                      [-2.74368847e-01,  9.55897382e-01, -1.04794708e-01,  8.13326377e+01],
    #                      [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]
    #Target COIL
    m_change_robot2ref_target1 =[[-3.09714152e-01,  6.48715643e-01, -6.95158369e-01,  3.77473420e+01],
                                [ 2.40546864e-01, -6.53866483e-01, -7.17353349e-01,  1.92178610e+02],
                                [-9.19899098e-01, -3.89392650e-01,  4.64651896e-02,  4.12281123e+01],
                                [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]

    m_change_robot2ref_target2 =[[-7.64107191e-01,  4.09701124e-01, -4.98282239e-01,  1.87398706e+02],
                                [ 3.70613597e-01, -3.53417432e-01, -8.58918902e-01,  1.66064139e+02],
                                [-5.28001669e-01, -8.40976282e-01,  1.18207994e-01, -3.29833828e+01],
                                [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]

if flag_kalman:
    tracker_stabilizers = [stabilizer.KalmanTracker(
        state_num=2,
        measure_num=1,
        cov_process=0.001, #Q is the covariance of the process noise. Simply put, Q specifies how much the actual motion of the object deviates from your assumed motion model.
        cov_measure=0.1) for _ in range(6)] # R is the covariance matrix of the measurement noise, assumed to be Gaussian.The R matrix must describe how uncertain you are about the location of the centroid.  If you are tracking people's faces, they are not likely to move with a constant velocity, so you need to crank up Q.
    import csv
    fieldnames = ['time','x','y','z','rx','ry', 'rz','xf','yf','zf','rxf','ryf', 'rzf', 'statusx','statusy','statusz']
    with open('data.csv', 'w') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        csv_writer.writeheader()

# if flag_velo:
#     lag = 10
#     threshold = 10
#     influence = 1
#
#     x = np.empty(lag)
#     x.fill(coord_tracker[1][0])
#     y = np.empty(lag)
#     y.fill(coord_tracker[1][1])
#     z = np.empty(lag)
#     z.fill(coord_tracker[1][2])
#     # Run algo with settings from above
#     tracker_pick_threshold_x = stabilizer.real_time_peak_detection([coord_tracker[1][0]], lag=lag, threshold=threshold, influence=influence)
#     tracker_pick_threshold_y = stabilizer.real_time_peak_detection([coord_tracker[1][1]], lag=lag, threshold=threshold, influence=influence)
#     tracker_pick_threshold_z = stabilizer.real_time_peak_detection([coord_tracker[1][2]], lag=lag, threshold=threshold, influence=influence)

if flag_tracker is False:
    trck_init_mtc.Close()

coord_vel = []
timestamp = []
coord_vec = []
vel_vec = []
time_start = time()
std = 0
move_ok = True
i = 0
while flag_tracker:
    i += 1
    start = time()
    coord_tracker, flag_markers, coord_robot = run_coords(trck_init_mtc, trck_init_robot)
    if not flag_markers[0]:
        print("Head markers is not visible")
        #input("The head marker is not visible. Please, try to repositioning the head. Input something to continue...")
        continue

    # Kalman filter
    if flag_kalman:
        kalman_array = []
        pose_np = np.array((coord_tracker[1][:3], coord_tracker[1][3:])).flatten()
        for value, ps_stb in zip(pose_np, tracker_stabilizers):
            ps_stb.update([value])
            kalman_array.append(ps_stb.state[0])
        # print(kalman_array)
        # kalman_array = np.reshape(kalman_array, (-1, 3))
        # coord, angles = kalman_array
        coord_kalman = np.hstack(kalman_array)

        #coord_kalman = np.hstack([coord, angles])
        #print(coord_kalman)
        #print(coord_tracker[1])

        coord_vec.append(coord_kalman[:3])
        if len(coord_vec) < 50: #avoid initial fluctuations
            coord_kalman = coord_tracker[1]
            print('init filter')
        else:
            del coord_vec[0]
    else:
        coord_kalman = coord_tracker[1]


    #Condições para movimentação
    actual_point = trck_init_robot.Run()
    print(actual_point)

    #m_ref = transform_pos_2_matrix(coord_kalman)
    #ref_tracker_in_robot, M_ref_tracker_in_robot = transform_tracker_2_robot(coord_kalman, M_tracker_2_robot)
    #m_robot_new = M_ref_tracker_in_robot @ m_change_robot2ref_target1
    #scale, shear, angles, translate, perspective = tf.decompose_matrix(m_robot_new)
    #angles = np.degrees(angles)

    #correc_point =  m_robot_new[0, -1], m_robot_new[1, -1], m_robot_new[2, -1], angles[0], angles[1], angles[2]

    #sum = (correc_point[0]-actual_point[0])**2+(correc_point[1]-actual_point[1])**2+(correc_point[2]-actual_point[2])**2
    #head_distance_compensation = pow(sum,0.5)
    #print('Distância de correção:',head_distance_compensation)

    if flag_velo:
        # status_x = tracker_pick_threshold_x.thresholding_algo(coord_kalman[0])
        # status_y = tracker_pick_threshold_y.thresholding_algo(coord_kalman[1])
        # status_z = tracker_pick_threshold_z.thresholding_algo(coord_kalman[2])
        # print(status_x,status_y,status_z)


        # if status_x == 1 or status_y == 1 or status_z == 1 or status_x == -1 or status_y == -1 or status_z == -1:
        #     print('peak detected')
        #     sleep(5)
        #     continue

        coord_vel.append(coord_kalman)
        timestamp.append(time())
        if len(coord_vel) >= 10:
            head_velocity, head_distance = estimate_head_velocity(coord_vel, timestamp)
            vel_vec.append(head_velocity)
            #Acumulhar head_velocity e calcular std, Qdo std estiver estavel, voltar a posicionar robot
            #pensar::
            # coord = np.abs(head_velocity[:3])
            coord = head_velocity[:3]
            # angles = np.abs(head_velocity[3:])
            angles = head_velocity[3:]
            # coord = np.abs(np.vstack([head_velocity[i][:len(head_velocity[0]) // 2] for i in range(len(head_velocity))]))
            # angles = np.abs(np.vstack([head_velocity[i][len(head_velocity[0]) // 2:] for i in range(len(head_velocity))]))
            # coord_distance = np.abs(
            #     np.vstack([head_distance[i][:len(head_distance[0]) // 2] for i in range(len(head_distance))]))
            # angles_distance = np.abs(
            #     np.vstack([head_distance[i][len(head_distance[0]) // 2:] for i in range(len(head_distance))]))
            #
            # status_velo_x = tracker_pick_threshold_x.thresholding_algo(coord[0])
            # status_velo_y = tracker_pick_threshold_y.thresholding_algo(coord[1])
            # status_velo_z = tracker_pick_threshold_z.thresholding_algo(coord[2])
            # print(status_velo_x, status_velo_y, status_velo_z)

            del coord_vel[0]
            del timestamp[0]
            print('std', np.std(vel_vec))

            if len(vel_vec) >= 20:
                std = np.std(vel_vec)
                del vel_vec[0]

            # if status_velo_x == 1 or status_velo_y == 1 or status_velo_z == 1 or\
            #         status_velo_x == -1 or status_velo_y == -1 or status_velo_z == -1
            #     print('peak detected')
            #     #sleep(1)
            #     continue
            if std > 5:
                print('std')
                move_ok = False

            else:
                move_ok = True


#     print('coord',coord)
        #     if  coord[coord > 10].any() or angles[angles > 10].any():
        #         # print("(coord_distance < 1).all()",(coord_distance < 1).all())
        #         # print("(angles_distance < 1).all()",(angles_distance < 1).all())
        #         print(" coord[coord > 30].any()", coord[coord > 30].any())
        #         print("angles[angles > 10].any()",angles[angles > 10].any())
        #         print('Velocity threshold activated')
        #         #sleep(1)
        #         continue


            with open('data.csv', 'a') as csv_file:
                csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

                info = {
                    "time": time()-time_start,
                    "x": coord_tracker[1][0],
                    "y": coord[1],
                    "z": coord[2],
                    "rx": coord_tracker[1][3],
                    "ry": coord_tracker[1][4],
                    "rz": coord_tracker[1][5],
                    "xf": coord_kalman[0],
                    "yf": coord_kalman[1],
                    "zf": coord_kalman[2],
                    "rxf": angles[0],
                    "ryf": angles[1],
                    "rzf": angles[2],
                    "statusx": std,
                    "statusy": std,
                    "statusz": std,
                }

                csv_writer.writerow(info)

    actual_point = trck_init_robot.Run()
    print(actual_point)

    #Cálculo da distância de correção:

    m_ref = transform_pos_2_matrix(coord_kalman)
    ref_tracker_in_robot, M_ref_tracker_in_robot = transform_tracker_2_robot(coord_kalman, M_tracker_2_robot)

    m_robot_new = M_ref_tracker_in_robot @ m_change_robot2ref_target1
    scale, shear, angles, translate, perspective = tf.decompose_matrix(m_robot_new)
    angles = np.degrees(angles)

    correc_point =  m_robot_new[0, -1], m_robot_new[1, -1], m_robot_new[2, -1], angles[0], angles[1], angles[2]

    sum = (correc_point[0]-actual_point[0])**2+(correc_point[1]-actual_point[1])**2+(correc_point[2]-actual_point[2])**2
    head_distance_compensation = pow(sum,0.5)
    print('Distância de correção:',head_distance_compensation)

    if head_distance_compensation < 100:
        m_ref = transform_pos_2_matrix(coord_kalman)
        ref_tracker_in_robot, M_ref_tracker_in_robot = transform_tracker_2_robot(coord_kalman, M_tracker_2_robot)

        m_robot_new = M_ref_tracker_in_robot @ m_change_robot2ref_target1
        scale, shear, angles_init, translate, perspective = tf.decompose_matrix(m_robot_new)
        angles_init = np.degrees(angles_init)

        target = m_robot_new[0, -1], m_robot_new[1, -1], m_robot_new[2, -1], angles_init[0], angles_init[1], angles_init[2]

        if move_ok:
            trck_init_robot.SendCoordinates(target)


    #elif i >=100:
    elif head_distance_compensation >= 100:
        p1 = correc_point

        pc = 756,-90,188, angles[0], angles[1], angles[2] #ponto central da cabeça

        versorfator1_calculado = Versores(pc,actual_point)
        init_ext_point = actual_point[0]+versorfator1_calculado[0],actual_point[1]+versorfator1_calculado[1],actual_point[2]+versorfator1_calculado[2], actual_point[3], actual_point[4], actual_point[5]
        print('init_ext_point:', init_ext_point)

        #m_robot_new2 = M_ref_tracker_in_robot @ m_change_robot2ref_target2
        #scale, shear, angles2, translate, perspective = tf.decompose_matrix(m_robot_new2)
        #angles2 = np.degrees(angles2)
        #p2 =  m_robot_new2[0, -1], m_robot_new2[1, -1], m_robot_new2[2, -1], angles2[0], angles2[1], angles2[2]

        pm = ((p1[0]+actual_point[0])/2, (p1[1]+actual_point[1])/2,(p1[2]+actual_point[2])/2 ) #ponto médio da trajetória

        versorfator2 = Versores(pc,pm)
        versorfator2 = np.array(versorfator2)
        newarr = versorfator2*2
        pontointermediario = pm[0]+newarr[0],pm[1]+newarr[1],pm[2]+newarr[2]

        versorfator3 = Versores(pc,p1)
        final_ext_point = p1[0]+versorfator3[0],p1[1]+versorfator3[1],p1[2]+versorfator3[2], angles[0], angles[1], angles[2]
        print('final_ext_point:', final_ext_point)
        final_ext_point_arc = final_ext_point = p1[0]+versorfator3[0],p1[1]+versorfator3[1],p1[2]+versorfator3[2], angles[0], angles[1], angles[2],0

        type_arc = 0
        #p11 = target

        target_arc = pontointermediario+final_ext_point_arc
        print(target_arc)

        if move_ok:
            trck_init_robot.StopMove()
            trck_init_robot.SendCoordinatesArc(init_ext_point,target_arc)

        #trck_init_robot.SendCoordinates(p11)

        #movimento arco
        #movimento linear até o target 1

    #else:
        #m_ref = transform_pos_2_matrix(coord_kalman)
        #ref_tracker_in_robot, M_ref_tracker_in_robot = transform_tracker_2_robot(coord_kalman, M_tracker_2_robot)

        #m_robot_new = M_ref_tracker_in_robot @ m_change_robot2ref_target1
        #scale, shear, angles, translate, perspective = tf.decompose_matrix(m_robot_new)
        #angles = np.degrees(angles)

        #target = m_robot_new[0, -1], m_robot_new[1, -1], m_robot_new[2, -1], angles[0], angles[1], angles[2]
        #print('target movimentaion:', target)

        #if move_ok:
             #trck_init_robot.SendCoordinates(target)


    end = time()
    print("                   ",end - start)
trck_init_mtc.Close()
