import numpy as np
import cv2
import glob

workingdir = "/home/pi/Desktop/Captures/"
savedir = "camera_data/"

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((7*7,3), np.float32)

# add 2.5 to account for 2.5 cm per square in grid
objp[:, :2] = np.mgrid[0:7, 0:7].T.reshape(-1, 2) * 2.5

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.
images = glob.glob('./Abuday1/*.jpg')
win_name = "Verify"
cv2.namedWindow(win_name, cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty(win_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

# print("getting images")
for fname in images:
    img = cv2.imread(fname)
    print(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7, 7), None)
    # If found, add object points, image points (after refining them)
    if ret:
        # print("found corners")
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv2.drawChessboardCorners(img, (7, 7), corners2, ret)
        cv2.imshow(win_name, img)
        cv2.waitKey(500)

    img1 = img

cv2.destroyAllWindows()

print(">==> Starting calibration")
ret, cam_mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("Camera Matrix")
print(cam_mtx)
np.save(savedir+'cam_mtx.npy', cam_mtx)
focal_length_x = cam_mtx[0, 0]
focal_length_y = cam_mtx[1, 1]

print(f"Focal Length (x-axis): {focal_length_x} pixels")
print(f"Focal Length (y-axis): {focal_length_y} pixels")

cx = cam_mtx[0, 2]
print(f"image coordinate u: {cx}")
cy = cam_mtx[1, 2]
print(f"image coordinate v: {cy}")
print("Distortion Coeff")
print(dist)
np.save(savedir+'dist.npy', dist)

print("r vecs")
print(rvecs[2])
np.save(savedir+'rvec1.npy', rvecs)

print("t Vecs")
print(tvecs[2])
np.save(savedir+'tvec1.npy', tvecs)

h, w = img1.shape[:2]
print("Image Width, Height")
print(w, h)

newcam_mtx, roi = cv2.getOptimalNewCameraMatrix(cam_mtx, dist, (w, h), 0, (w, h))

print("Region of Interest")
print(roi)
np.save(savedir+'roi.npy', roi)

print("New Camera Matrix")
np.save(savedir+'newcam_mtx.npy', newcam_mtx)
print(np.load(savedir+'newcam_mtx.npy'))

inverse = np.linalg.inv(newcam_mtx)
print("Inverse New Camera Matrix")
print(inverse)

# undistort
undst = cv2.undistort(img1, cam_mtx, dist, None, newcam_mtx)

cv2.imshow('img1', img1)
cv2.waitKey(5000)      
cv2.destroyAllWindows()
cv2.imshow('img1', undst)
cv2.waitKey(5000)      
cv2.destroyAllWindows()
