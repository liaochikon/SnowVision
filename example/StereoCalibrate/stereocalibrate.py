import numpy as np
import cv2

def Load_Video(video_name, usable_cam_num):
    cap_array = np.array([])
    img_size_array = np.array([])
    length_array = np.array([], dtype=int)
    for num in range(usable_cam_num):
        cap_temp = cv2.VideoCapture(video_name + str(num) + '.avi')
        length_temp = int(cv2.VideoCapture.get(cap_temp, cv2.CAP_PROP_FRAME_COUNT))
        v_w = int(cv2.VideoCapture.get(cap_temp, cv2.CAP_PROP_FRAME_WIDTH))
        v_h = int(cv2.VideoCapture.get(cap_temp, cv2.CAP_PROP_FRAME_HEIGHT))
        img_size_temp = np.array([v_w, v_h])
        cap_array = np.append(cap_array, cap_temp)
        length_array = np.append(length_array, length_temp)
        img_size_array = np.append(img_size_array, img_size_temp)
        print('Get video : ' + video_name + str(num) + '.avi')
        print('Video resolution : ' + str(img_size_array))
        print('Video total frame number : ' + str(length_array))
    img_size_array = img_size_array.reshape((usable_cam_num, 2))
    return cap_array, length_array, img_size_array


def Single_Corner_Collector(image, current_cam, chessboard):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (chessboard.height, chessboard.width), None)
    if ret:
        corners_sub = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), chessboard.criteria)
        globals()['Single_objectPoints' + str(current_cam)].append(chessboard.objp)
        globals()['Single_imagePoints' + str(current_cam)].append(corners_sub)
    return ret

def Single_Calibrate_Limit(current_cam, limit_num, img_size):
    newcameramtx = []
    gap = len(globals()['Single_objectPoints' + str(current_cam)]) // limit_num
    Single_objectPoints = globals()['Single_objectPoints' + str(current_cam)]
    Single_imagePoints = globals()['Single_imagePoints' + str(current_cam)]
    Single_objectPoints_temp = []
    Single_imagePoints_temp = []
    l = len(Single_objectPoints)
    pad = 1
    for i in range(limit_num):
        index = i * gap
        if index >= l:
            if l % gap == 1:
                index = i * gap - l
            else:
                index = i * gap - l + pad
        Single_objectPoints_temp.append(Single_objectPoints[index])
        Single_imagePoints_temp.append(Single_imagePoints[index])

    ret, mtx, dist, _, _ = cv2.calibrateCamera(Single_objectPoints_temp, Single_imagePoints_temp, img_size, None, None)
    if ret:
        newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, img_size, 1, img_size)
    return ret, newcameramtx, dist

def Calibrate_Cam_K(video_name, usable_cam_num, limit_num, chessboard):
    for num in range(usable_cam_num):
        globals()['Single_objectPoints' + str(num)] = []
        globals()['Single_imagePoints' + str(num)] = []
        globals()['K' + str(num)] = []
        globals()['D' + str(num)] = []

    cap, length, img_size = Load_Video(video_name, usable_cam_num)

    for num in range(usable_cam_num):
        for current_frame in range(length[0]):
            print('current_frame : ' + str(current_frame))
            _, frame = cap[num].read()
            img = cv2.resize(frame, (800, 450))
            cv2.imshow(str(num), img)
            cv2.waitKey(1)
            ret = Single_Corner_Collector(frame, num, chessboard)
            if ret:
                print('Get mono corners at cam ' + str(num) + ' : ' + str(len(globals()['Single_objectPoints' + str(num)])))
        print('Calibrating Mono corners at cam ' + str(num))
        ret, globals()['K' + str(num)], globals()['D' + str(num)] = Single_Calibrate_Limit(num, limit_num, (int(img_size[num][0]), int(img_size[num][1])))
        if ret:
            print('Camera calibration at cam ' + str(num) + ' successed!')
        else:
            print('Camera calibration at cam ' + str(num) + ' failed...')

def Stereo_Calibrate(current_sub_cam, img_size):
    ret1, mtx1, dist1, rvecs1, tvecs1 = cv2.calibrateCamera(globals()['Stereo_objectPoints' + str(current_sub_cam)],
                                                            globals()['Stereo_imagePoints' + str(current_sub_cam) + str(0)],
                                                            img_size, None, None)
    ret2, mtx2, dist2, rvecs2, tvecs2 = cv2.calibrateCamera(globals()['Stereo_objectPoints' + str(current_sub_cam)],
                                                            globals()['Stereo_imagePoints' + str(current_sub_cam) + str(1)],
                                                            img_size, None, None)

    ret_s, mtx_s1, dist_s1, mtx_s2, dist_s2, rvecs_s, tvecs_s, E, F = cv2.stereoCalibrate(globals()['Stereo_objectPoints' + str(current_sub_cam)],
                                                                                          globals()['Stereo_imagePoints' + str(current_sub_cam) + str(0)],
                                                                                          globals()['Stereo_imagePoints' + str(current_sub_cam) + str(1)],
                                                                                          mtx1, dist1, mtx2, dist2, img_size)

    newcameramtx1, _ = cv2.getOptimalNewCameraMatrix(mtx_s1, dist_s1, img_size, 1, img_size)
    newcameramtx2, _ = cv2.getOptimalNewCameraMatrix(mtx_s2, dist_s2, img_size, 1, img_size)

    return newcameramtx1, newcameramtx2, dist_s1, dist_s2, rvecs_s, tvecs_s, E, F

def Calibrate_Cam_Rt(video_name, usable_cam_num, limit_num, chessboard):
    for num in range(usable_cam_num):
        globals()['time_code' + str(num)] = []
        globals()['Single_objectPoints' + str(num)] = []
        globals()['Single_imagePoints' + str(num)] = []
        globals()['Stereo_objectPoints' + str(num)] = []
        globals()['Stereo_imagePoints' + str(num) + str(0)] = []
        globals()['Stereo_imagePoints' + str(num) + str(1)] = []
        globals()['T' + str(num)] = []

    cap, length, img_size = Load_Video(video_name, usable_cam_num)

    for num in range(usable_cam_num):
        for current_frame in range(length[0]):
            print('current_frame : ' + str(current_frame))
            _, frame = cap[num].read()
            img = cv2.resize(frame, (800, 450))
            cv2.imshow(str(num), img)
            cv2.waitKey(1)
            ret = Single_Corner_Collector(frame, num, chessboard)
            if ret:
                globals()['time_code' + str(num)].append(current_frame)
                print('Get mono corners at cam ' + str(num) + ' : ' + str(len(globals()['Single_objectPoints' + str(num)])))

    R0 = np.eye(3)
    t0 = np.array([[0], [0], [0]])
    globals()['T' + str(0)] = np.hstack((R0, t0))

    for num in range(usable_cam_num - 1):
        main_cam = 0
        sub_cam = num + 1
        t_c1 = globals()['time_code' + str(main_cam)]
        t_c2 = globals()['time_code' + str(sub_cam)]
        intersection = set(t_c1) & set(t_c2)
        print(len(intersection))
        for a, i in enumerate(intersection):
            if a < limit_num:
                index1 = t_c1.index(i)
                index2 = t_c2.index(i)
                globals()['Stereo_objectPoints' + str(sub_cam)].append(chessboard.objp)
                globals()['Stereo_imagePoints' + str(sub_cam) + str(0)].append(
                    globals()['Single_imagePoints' + str(main_cam)][index1])
                globals()['Stereo_imagePoints' + str(sub_cam) + str(1)].append(
                    globals()['Single_imagePoints' + str(sub_cam)][index2])
            else:
                break
        _, _, _, _, R, t, _, _ = Stereo_Calibrate(sub_cam, (int(img_size[num][0]), int(img_size[num][1])))
        globals()['T' + str(sub_cam)] = np.hstack((R, t))

def Output_Data(filename, usable_cam_num):
    with open(filename + '.txt', 'w') as f:
        f.write('Usable_Cam_Num=' + str(usable_cam_num) + '\n')
        for num in range(usable_cam_num):
            f.write('Cam_' + str(num) + ':\n')
            K_temp = globals()['K' + str(num)].reshape((9))
            f.write('K=')
            for k in K_temp:
                f.write(str(float(k)) + ',')
            f.write('\n')
            Rt_temp = globals()['T' + str(num)].reshape((12))
            f.write('Rt=')
            for Rt in Rt_temp:
                f.write(str(float(Rt)) + ',')
            f.write('\n')
            D_temp = globals()['D' + str(num)][0]
            f.write('D=')
            for D in D_temp:
                f.write(str(float(D)) + ',')
            print(K_temp)
            print(Rt_temp)
            print(D_temp)
            f.write('\n')

class ChessBoard:
    height = 9
    width = 6
    square_size = 0.095
    objp = np.zeros((width * height, 3), np.float32)
    objp[:, :2] = square_size * np.mgrid[0:height, 0:width].T.reshape(-1, 2)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

CB = ChessBoard

Calibrate_Cam_K("calk_long", 4, 4, CB)
Calibrate_Cam_Rt("calk_long", 4, 4, CB)
Output_Data("output_long_test", 4)

