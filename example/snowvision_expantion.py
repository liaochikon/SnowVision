import cv2
import numpy as np
import math
import pytransform3d.transformations as pt
import pytransform3d.camera as pc
from pytransform3d.plot_utils import plot_vector


def Read_Array(line, shape):
    K = np.array([])
    K_str = line.split('=')[1].split(',')
    for i in range(int(shape[0] * shape[1])):
        k_temp = float(K_str[i])
        K = np.append(K, k_temp)
    K = K.reshape(shape)
    return K

def Get_Camera_Rotation_Matrix(Rt):
    return Rt[:,:3]

def Get_Camera_Position(Rt):
    return Rt.T[3]

def Load_Camera_Info(filepath):
    with open(filepath) as f:
        K_array = np.array([])
        Rt_array = np.array([])
        D_array = np.array([])
        lines = f.readlines()
        usable_cam_num = int(lines[0].split('=')[1])
        print("Usable_Cam_Num：",usable_cam_num)
        for i in range(usable_cam_num):
            index = int(1 + i * 4)
            K = Read_Array(lines[index + 1], (3, 3))
            Rt = Read_Array(lines[index + 2], (3, 4))
            D = Read_Array(lines[index + 3], (1, 5))
            print(lines[index])
            print("K：",K)
            print("Rt：", Rt)
            print("D：", D)
            K_array = np.append(K_array, K)
            Rt_array = np.append(Rt_array, Rt)
            D_array = np.append(D_array, D)
        K_array = K_array.reshape((usable_cam_num , 3, 3))
        Rt_array = Rt_array.reshape((usable_cam_num, 3, 4))
        D_array = D_array.reshape((usable_cam_num, 5))
        return usable_cam_num, K_array, Rt_array, D_array

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

def Load_Image(image_name, usable_cam_num):
    img_array = np.array([])
    img_size_array = np.array([])
    for num in range(usable_cam_num):
        img_temp = cv2.imread(image_name + str(num) + '.png')
        v_w = img_temp.shape[1]
        v_h = img_temp.shape[0]
        img_size_temp = np.array([v_w, v_h])
        img_array = np.append(img_array, img_temp)
        img_size_array = np.append(img_size_array, img_size_temp)
        print('Get image : ' + image_name + str(num) + '.png')
        print('image resolution : ' + str(img_size_array))
    img_size_array = img_size_array.reshape((usable_cam_num, 2))
    return img_array, img_size_array

def Rt_to_T(Rt):
    Rt = np.append(Rt, np.array([0, 0, 0, 1]))
    T = Rt.reshape((4, 4))
    return T

def Draw_Camera(ax, K, Rt, img_size, scale):
    T = Rt_to_T(Rt)
    pt.plot_transform(ax=ax, A2B=T, s=scale)
    #pc.plot_camera(
    #    ax, cam2world=T, M=K, sensor_size=img_size)
    return ax

def DrawCameraGroup(ax, CameraInfo, img_size_array, scale):
    for i, p in enumerate(zip(CameraInfo, img_size_array)):
        K = p[0].K
        Rt = p[0].Rt
        img_size = p[1]
        Draw_Camera(ax, K, Rt, img_size, scale)
        t = Get_Camera_Position(Rt)
        ax.text(t[0], t[1] - 0.1, t[2], str(i + 1), size=20, zorder=1, color='k')

def Find_Usable_Cam(cam_search_limit):
    usable_cam = np.array([])
    for num in range(cam_search_limit):
        video_temp = cv2.VideoCapture(num)
        ret, _ = video_temp.read()
        if ret:
            usable_cam = np.append(usable_cam, num)
            globals()['cap' + str(num)] = cv2.VideoCapture(num)
        else:
            break
    for num in range(len(usable_cam)):
        video_temp = globals()['cap' + str(num)]
        ret, frame_temp = video_temp.read()
        if np.any(frame_temp) == None:
            usable_cam = np.delete(usable_cam, num)
    return usable_cam

def Camera_Matrix_From_Fundamental_Matrix(K1,K2,R,T):
    A = K1 * R.T * T
    C = np.array([0,-A[2],A[1]],[A[2],0,-A[0]],[-A[1],A[0],0])
    F = np.linalg.inv(K2).T * R * K1 * C
    return F

def Fundamental_Matrix_From_Projections_Matrix(P1,P2):
    X = np.zeros((3,2,4))
    X[0] = np.array([P1[1],P1[2]])
    X[1] = np.array([P1[2],P1[0]])
    X[2] = np.array([P1[0],P1[1]])

    Y = np.zeros((3, 2, 4))
    Y[0] = np.array([P2[1],P2[2]])
    Y[1] = np.array([P2[2],P2[0]])
    Y[2] = np.array([P2[0],P2[1]])

    F = np.zeros((3,3))
    for i in range(3):
        for j in range(3):
            XY = np.array([X[j][0],X[j][1],Y[i][0],Y[i][1]])
            F[i][j] = np.linalg.det(XY)
    return F

def Gray_GaussianBlur(image):
    kernel_size = 5
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)
    return blur_gray

def Processed_image(image, hsv_low, hsv_high):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_low, hsv_high)
    return mask

def Find_Points(processed_image, min_area = 0, max_area = 70):
    points = np.array([])
    contours, _ = cv2.findContours(processed_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for num, cnt in enumerate(contours):
        M = cv2.moments(cnt)
        a = cv2.contourArea(cnt)
        if a > min_area and a < max_area:
            xdot = M["m10"] / M["m00"]
            ydot = M["m01"] / M["m00"]
            points_temp = np.array([xdot, ydot])
            points = np.append(points, points_temp)
    points = np.reshape(points,(len(points)//2,2))
    return points

def Point_Label(image,point,istext,text,offset):
    x = int(point[0])
    y = int(point[1])
    image = cv2.circle(image, (x, y), 5, (255, 0, 0), -1)
    if (istext):
        image = cv2.putText(image, str(text), (x, y - offset), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 2)
        return image
    return image

def Point_Group_Label(image,points,istext,text,offset):
    for p in points:
        image = Point_Label(image, p, istext, text, offset)
    return image

def Inv_Group(M):
    shape = M.shape
    Inv_array = np.array([])
    for m in M:
        m_inv = np.linalg.inv(m)
        Inv_array = np.append(Inv_array, m_inv)
    Inv_array = Inv_array.reshape(shape)
    return Inv_array

def Get_F(K_inv, w):
    return np.dot(K_inv, w)

def Get_F_Group(Points, K_array):
    K_inv_array = Inv_Group(K_array)
    F_array = np.array([])
    for p in Points:
        cam_id = int(p[2])
        w = np.array([p[0], p[1], 1]).reshape((-1, 1))
        F = Get_F(K_inv_array[cam_id], w)
        F_array = np.append(F_array, F)
    return F_array.reshape((len(Points), 3, 1))

def Get_H(F_main, F_sub, R_sub):
    G = np.dot(R_sub, F_sub)
    return np.hstack((F_main, -G))

def Get_S_Star(F_main, F_sub, Rt):
    R_sub = Get_Camera_Rotation_Matrix(Rt)
    t_sub = Get_Camera_Position(Rt)
    H = Get_H(F_main, F_sub, R_sub)
    A = np.dot(H.T, H)
    A_inv = np.linalg.inv(A)
    B = np.dot(H.T, t_sub).reshape((-1, 1))
    S_star_temp = np.dot(A_inv, B)
    S_star = np.zeros((2,2))
    S_star[0][0] = S_star_temp[0][0]
    S_star[1][1] = S_star_temp[1][0]
    return S_star

def Get_W_star(S_star, F_main, F_sub):
    F_star_upper = np.append(F_main, np.zeros(3))
    F_star_lower = np.append(np.zeros(3), F_sub)
    F_star = np.vstack((F_star_upper, F_star_lower))
    return np.dot(S_star, F_star)

def Get_W_sub_W_main(W_star):
    W_main = np.delete(W_star[0], np.array([3, 4, 5]))
    W_sub = np.delete(W_star[1], np.array([0, 1, 2]))
    return W_sub, W_main

def W_TO_World_Coordination(W, Rt):
    R = Get_Camera_Rotation_Matrix(Rt)
    t = Get_Camera_Position(Rt)
    return np.dot(R, W) + t

def W_Distance_Checker(W_sub, W_main, Distance_tol):
    Distance = np.linalg.norm(W_sub[:3] - W_main[:3])
    if Distance < Distance_tol:
        return True
    return False

def Get_W_center(W_sub, W_main):
    return (W_sub[:3] + W_main[:3]) / 2

def Flat_Points_To_Tri_Points(Points, K_array, Rt_array, Distance_tol):
    tri_points = np.array([])
    F_array = Get_F_Group(Points, K_array)
    current_main_point_index = 0
    total_ln = len(Points) - 1
    while total_ln != 0:
        for current_padding in range(total_ln):
            current_sub_point_index = current_padding + current_main_point_index + 1
            current_main_cam_index = int(Points[current_main_point_index][2])
            current_sub_cam_index = int(Points[current_sub_point_index][2])
            F_main = F_array[current_main_point_index]
            F_sub = F_array[current_sub_point_index]
            Rt_main = Rt_array[current_main_cam_index]
            Rt_sub = Rt_array[current_sub_cam_index]
            S_star = Get_S_Star(F_main, F_sub, Rt_sub)
            W_star = Get_W_star(S_star, F_main, F_sub)
            W_sub, W_main = Get_W_sub_W_main(W_star)
            W_sub = W_TO_World_Coordination(W_sub, Rt_sub)
            W_main = W_TO_World_Coordination(W_main, Rt_main)
            if W_Distance_Checker(W_sub, W_main, Distance_tol):
                tri_points = np.append(tri_points, Get_W_center(W_sub, W_main))
                tri_points = np.append(tri_points, current_main_cam_index)
                #tri_points = np.append(tri_points, W_sub)
                #tri_points = np.append(tri_points, current_sub_cam_index)
                #tri_points = np.append(tri_points, W_main)
                #tri_points = np.append(tri_points, current_main_cam_index)
        current_main_point_index += 1
        total_ln -= 1
    if tri_points != np.array([]):
        tri_points = tri_points.reshape((len(tri_points) // 4, 4))
    return tri_points

def Plot_Tri_Point_Ray(tri_points, Rt_array):
    for t in tri_points:
        cam_index = int(t[3])
        c_t = Get_Camera_Position(Rt_array[cam_index])
        start = c_t
        direction = t
        plot_vector(start=start, direction=direction, color="orange")