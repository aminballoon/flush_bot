import numpy as np
import cv2
import glob
import argparse

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

def calibrate(width=9, height=15):
    """ Apply camera calibration operation for images in the given directory path. """
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
    objp = np.zeros((height*width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)

    objp = objp * 1.5
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.
    images = glob.glob(r'img_caribate/*.jpg')

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (width, height), None)
        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (width, height), corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(1)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    cv2.destroyAllWindows()
    return [ret, mtx, dist, rvecs, tvecs]

def save_coefficients(mtx, dist):
    """ Save the camera matrix and the distortion coefficients to given path/file. """
    cv_file = cv2.FileStorage("config_camera.yml", cv2.FILE_STORAGE_WRITE)
    cv_file.write("K", mtx)
    cv_file.write("D", dist)
    # note you *release* you don't close() a FileStorage object
    cv_file.release()

def load_coefficients(path):
    """ Loads camera matrix and distortion coefficients. """
    # FILE_STORAGE_READ
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode("K").mat()
    dist_matrix = cv_file.getNode("D").mat()

    cv_file.release()
    return [camera_matrix, dist_matrix]

def undistort_image(img,newcameramtx):
    cv_file = cv2.FileStorage("config_camera.yml", cv2.FILE_STORAGE_READ)

    mtx = cv_file.getNode("K").mat()
    dist = cv_file.getNode("D").mat()

    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

    x,y,w,h = roi
    dst = dst[y:y+h, x:x+w]
    cv2.imwrite('calibresult.png',dst)
    cv2.imshow(" ballon",dst)
    cv2.imshow("ror",img)
    cv2.waitKey(0)

if __name__ == "__main__":
    # ret, mtx, dist, rvecs, tvecs = calibrate(9,15)
    # save_coefficients(mtx, dist)
    # print("Calibration is finished. RMS: ", ret)

    img = cv2.imread('img_caribate\imageedit_1_9353646497.jpg')
    h,  w = img.shape[:2]
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(load_coefficients("config_camera.yml")[0],load_coefficients("config_camera.yml")[1],(w,h),1,(w,h))
    undistort_image(img,newcameramtx)