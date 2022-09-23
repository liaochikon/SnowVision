import snowvision as sv
import numpy as np
import cv2
import snowvision_expantion as so
import matplotlib.pyplot as plt
import pytransform3d.transformations as pt
import time

Video_Name = "light"
Cam_Info_Path = 'output.txt'
CameraGroup = sv.CameraGroup(Cam_Info_Path)
Distance_tol = 2
kernel_size = 5
thres_low = [245, 245, 245, 245]
thres_high = [255, 255, 255, 255]

work_space = [-2, 2, -3, 1, 4.8, 8.8]

cap, length, img_size = so.Load_Video(Video_Name, CameraGroup.CameraCount)
ax = pt.plot_transform()

#a = np.array([])
so.DrawCameraGroup(ax, CameraGroup.CameraInfo, img_size, 1)
ss = np.array([])
for l in range(200):
    b = time.time()
    for num in range(CameraGroup.CameraCount):
        _, img = cap[num].read()
        if l < 35:
            continue
        img = cv2.undistort(img, CameraGroup.CameraInfo[num].K, CameraGroup.CameraInfo[num].D)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur_gray = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)
        _, thre = cv2.threshold(blur_gray, thres_low[num], thres_high[num], cv2.THRESH_BINARY)
        points_temp = so.Find_Points(thre)
        CameraGroup.AddCameraPoints(points_temp)
        img = so.Point_Group_Label(img, points_temp, True, str(0), 5)
        cv2.imshow(str(num), cv2.resize(img, (800, 450)))
    CameraGroup.Triangulation(Distance_tol)
    CameraGroup.TriangulationWorkspaceFilter(work_space)
    CameraGroup.TriangulationCondense(0.7)

    if CameraGroup.TriangulatedPoints != []:
        for t in CameraGroup.TriangulatedPoints:
            ss = np.append(ss, np.array([t[0], t[1], t[2]]))
    CameraGroup.ClearTriangulatedPoints()
    CameraGroup.ClearCameraPoints()
    cv2.waitKey(1)
#    d = time.time() - b
#    a = np.append(a, d)
#    print(l)
#    print(d)
#print(a.mean())

ss = ss.reshape((len(ss) // 3, 3)).T
ax.scatter(ss[0], ss[1], ss[2])
ax.set_xlim3d(work_space[0], work_space[1])
ax.set_ylim3d(work_space[2], work_space[3])
ax.set_zlim3d(work_space[4], work_space[5])
plt.show()