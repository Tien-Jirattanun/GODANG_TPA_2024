#https://docs.opencv.org/3.3.0/dc/dbb/tutorial_py_calibration.html

import numpy as np
import cv2
import glob
import camera_realworldxyz

cameraXYZ=camera_realworldxyz.camera_realtimeXYZ()

calculatefromCam=True

imgdir="/home/pi/Desktop/Captures/"

writeValues=True

#test camera calibration against all points, calculating XYZ

#load camera calibration
savedir="camera_data/"
cam_mtx=np.load(savedir+'cam_mtx.npy')
dist=np.load(savedir+'dist.npy')
newcam_mtx=np.load(savedir+'newcam_mtx.npy')
roi=np.load(savedir+'roi.npy')


#load center points from New Camera matrix
cx=newcam_mtx[0,2]
cy=newcam_mtx[1,2]
fx=newcam_mtx[0,0]
print("cx: "+str(cx)+",cy "+str(cy)+",fx "+str(fx))


#MANUALLY INPUT YOUR MEASURED POINTS HERE
#ENTER (X,Y,d*)26
#d* is the distance from your point to the camera lens. (d* = Z for the camera center)
#we will calculate Z in the next steps after extracting the new_cam matrix

508
#world center + 9 world points

total_points_used=9

#x,y,z

#image cap_2
# worldPoints=np.array([[-9.5,-13.5,270],
#                        [0,-13.5,270],
#                        [9.5,-13.5,270],
#                        [-9.5,21,270],
#                        [0,21,270],
#                        [9.5,21,270],
#                        [-9.5,28.5,270],
#                        [0,28.5,270],
#                        [9.5,28.5,270]], dtype=np.float32)

# #image cap_1 x=4
# worldPoints=np.array([[-13.5,-27,173],
#                        [-4,-27,173],
#                        [5.5,-27,173],
#                        [-13.5,-34.5,173],
#                        [-4,-34.5,173],
#                        [5.5,-34.5,173],
#                        [-13.5,-42,173],
#                        [-4,-42,173],
#                        [5.5,-42,173]], dtype=np.float32)


# #image cap_3 x=3
# worldPoints=np.array([[38,13.5,270],
#                        [4.75,13.5,270],
#                        [57,13.5,270],
#                        [38,21,270],
#                        [4.75,21,270],
#                        [57,21,270],
#                        [38,28.5,270],
#                        [4.75,28.5,270],
#                        [57,28.5,270]], dtype=np.float32)

# #image cap_2_1 x=5
worldPoints=np.array([[-118.5,13.5,270],
                       [-109,13.5,270],
                       [-99.5,13.5,270],
                       [-118.5,21,270],
                       [-109,21,270],
                       [-99.5,21,270],
                       [-118.5,28.5,270],
                       [-109,28.5,270],
                       [-99.5,28.5,270]], dtype=np.float32)

#image cap_2_2 
# vertical
# worldPoints=np.array([[99.5,-20.7,270],
#                        [107,-20.7,270],
#                        [114.5,-20.7,270],
#                        [99.5,-30.2,270],
#                        [107,-30.2,270],
#                        [114.5,-30.2,270],
#                        [99.5,-39.7,270],
#                        [107,-39.7,270],
#                        [114.5,-39.7,270]], dtype=np.float32)

#MANUALLY INPUT THE DETECTED IMAGE COORDINATES HERE

#[u,v] center + 9 Image points

# image 2_2
# imagePoints=np.array([[1676,656],
#                        [1726,656],
#                        [1777,657],
#                        [1676,717],
#                        [1726,717],
#                        [1777,718],
#                        [1675,776],
#                        [1726,778],
#                        [1776,779]], dtype=np.float32)


# # image 3
# imagePoints=np.array([[1193,576],
#                        [1230,575],
#                        [1268,575],
#                        [1193,604],
#                        [1231,604],
#                        [1268,604],
#                        [1193,633],
#                        [1231,633],
#                        [1268,633]], dtype=np.float32)

# # image 2_1
imagePoints=np.array([[317,690],
                       [376,575],
                       [1083,575],
                       [1011,604],
                       [1047,603],
                       [1084,603],
                       [1011,632],
                       [1047,632],
                       [1084,632]], dtype=np.float32)


# image 2
# imagePoints=np.array([[1010,576],
#                        [1047,575],
#                        [1083,575],
#                        [1011,604],
#                        [1047,603],
#                        [1084,603],
#                        [1011,632],
#                        [1047,632],
#                        [1084,632]], dtype=np.float32)

# image 4 cap_1
# imagePoints=np.array([[968,687],
#                        [1026, 687],
#                        [1082,687],
#                        [968,732],
#                        [1026,732],
#                        [1083,732],
#                        [969,777],
#                        [1026,777],
#                        [1083,777]], dtype=np.float32)

#FOR REAL WORLD POINTS, CALCULATE Z from d*

for i in range(1,total_points_used):
    #start from 1, given for center Z=d*
    #to center of camera
    wX=worldPoints[i,0]
    wY=worldPoints[i,1]
    wd=worldPoints[i,2]

    d1=np.sqrt(np.square(wX)+np.square(wY))
    wZ=np.sqrt(np.square(wd)-np.square(d1))
    worldPoints[i,2]=wZ

print(worldPoints)


#print(ret)
print("Camera Matrix")
print(cam_mtx)
print("Distortion Coeff")
print(dist)

print("Region of Interest")
print(roi)
print("New Camera Matrix")
print(newcam_mtx)
inverse_newcam_mtx = np.linalg.inv(newcam_mtx)
print("Inverse New Camera Matrix")
print(inverse_newcam_mtx)
if writeValues==True: np.save(savedir+'inverse_newcam_mtx.npy', inverse_newcam_mtx)

print(">==> Calibration Loaded")


print("solvePNP")
ret, rvec1, tvec1=cv2.solvePnP(worldPoints,imagePoints,newcam_mtx,dist)

print("pnp rvec1 - Rotation")
print(rvec1)
if writeValues==True: np.save(savedir+'rvec1.npy', rvec1)

print("pnp tvec1 - Translation")
print(tvec1)
if writeValues==True: np.save(savedir+'tvec1.npy', tvec1)

print("R - rodrigues vecs")
R_mtx, jac=cv2.Rodrigues(rvec1)
print(R_mtx)
if writeValues==True: np.save(savedir+'R_mtx.npy', R_mtx)

print("R|t - Extrinsic Matrix")
Rt=np.column_stack((R_mtx,tvec1))
print(Rt)
if writeValues==True: np.save(savedir+'Rt.npy', Rt)

print("newCamMtx*R|t - Projection Matrix")
P_mtx=newcam_mtx.dot(Rt)
print(P_mtx)
if writeValues==True: np.save(savedir+'P_mtx.npy', P_mtx)

#[XYZ1]



#LETS CHECK THE ACCURACY HERE


s_arr=np.array([0], dtype=np.float32)
s_describe=np.array([0,0,0,0,0,0,0,0,0,0],dtype=np.float32)

for i in range(0,total_points_used):
    print("=======POINT # " + str(i) +" =========================")
    
    print("Forward: From World Points, Find Image Pixel")
    XYZ1=np.array([[worldPoints[i,0],worldPoints[i,1],worldPoints[i,2],1]], dtype=np.float32)
    XYZ1=XYZ1.T
    print("{{-- XYZ1")
    print(XYZ1)
    suv1=P_mtx.dot(XYZ1)
    print("//-- suv1")
    print(suv1)
    s=suv1[2,0]    
    uv1=suv1/s
    print(">==> uv1 - Image Points")
    print(uv1)
    print(">==> s - Scaling Factor")
    print(s)
    s_arr=np.array([s/total_points_used+s_arr[0]], dtype=np.float32)
    s_describe[i]=s
    if writeValues==True: np.save(savedir+'s_arr.npy', s_arr)

    print("Solve: From Image Pixels, find World Points")

    uv_1=np.array([[imagePoints[i,0],imagePoints[i,1],1]], dtype=np.float32)
    uv_1=uv_1.T
    print(">==> uv1")
    print(uv_1)
    suv_1=s*uv_1
    print("//-- suv1")
    print(suv_1)

    print("get camera coordinates, multiply by inverse Camera Matrix, subtract tvec1")
    xyz_c=inverse_newcam_mtx.dot(suv_1)
    xyz_c=xyz_c-tvec1
    print("      xyz_c")
    inverse_R_mtx = np.linalg.inv(R_mtx)
    XYZ=inverse_R_mtx.dot(xyz_c)
    print("{{-- XYZ")
    print(XYZ)

    if calculatefromCam==True:
        cXYZ=cameraXYZ.calculate_XYZ(imagePoints[i,0],imagePoints[i,1])
        print("camXYZ")
        print(cXYZ)


s_mean, s_std = np.mean(s_describe), np.std(s_describe)

print(">>>>>>>>>>>>>>>>>>>>> S RESULTS")
print("Mean: "+ str(s_mean))
#print("Average: " + str(s_arr[0]))
print("Std: " + str(s_std))


print("total_points_used: "+str(total_points_used))
print(">>>>>> S Error by Point")

for i in range(0,total_points_used):
    print("Point "+str(i))
    print("S: " +str(s_describe[i])+" Mean: " +str(s_mean) + " Error: " + str(s_describe[i]-s_mean))
    print("-----------------")
    # print("real world Z: "+str(worldPoints[i,2])+" calculated Z: "+str(XYZ[2,0]))
    # print("Error: "+str(worldPoints[i,2]-XYZ[2,0]))
    # print("real world X: "+str(worldPoints[i,0])+" calculated X: "+str(XYZ[0,0]))
    # print("Error: "+str(worldPoints[i,0]-XYZ[0,0]))
    # print("real world Y: "+str(worldPoints[i,1])+" calculated Y: "+str(XYZ[1,0]))
    # print("Error: "+str(worldPoints[i,1]-XYZ[1,0]))
