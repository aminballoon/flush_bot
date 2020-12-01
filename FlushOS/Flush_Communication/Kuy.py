import cv2
import numpy as np
import matplotlib as mpl
import glob
from cv2 import aruco

def line_intersection(line1, line2):
    # reference https://stackoverflow.com/questions/20677795/how-do-i-compute-the-intersection-point-of-two-lines
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       raise Exception('lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return [int(x), int(y)]

def perspectrive_with_aruco(img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        line_checker = np.zeros(gray.shape, np.uint8)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        # aruco ids
        top_row =    [16,15,14,13,12,11,10,9,8]
        left_row =   [16,17,18,19,20,21,22,23,24]
        right_row =  [8,7,6,5,4,3,2,1,32]
        bottom_row = [24,25,26,27,28,29,30,31,32]
        all_rows = []
        lines = []
        for row in [top_row,left_row,right_row,bottom_row]:
            original_row = row.copy()
            set_of_id = set([num[0] for num in ids.tolist()])
            row = list(set(row).intersection(set_of_id))
            row = [ele for ele in original_row if ele in row]
            if (len(row) < 2):
                return None
            all_rows.append([row[0],row[-1]])
        for end_points_ids in all_rows:
            start_point_corners = corners[[num[0] for num in ids.tolist()].index(end_points_ids[0])][0]
            end_points_corners  = corners[[num[0] for num in ids.tolist()].index(end_points_ids[1])][0]
            start_point =   (int((start_point_corners[0][0] + start_point_corners[1][0] + start_point_corners[2][0] + start_point_corners[3][0]) / 4),
                            int((start_point_corners[0][1] + start_point_corners[1][1] + start_point_corners[2][1] + start_point_corners[3][1]) / 4))
            end_point =     (int((end_points_corners[0][0] + end_points_corners[1][0] + end_points_corners[2][0] + end_points_corners[3][0]) / 4),
                            int((end_points_corners[0][1] + end_points_corners[1][1] + end_points_corners[2][1] + end_points_corners[3][1]) / 4))
            lines.append([start_point,end_point])
        top_left     = line_intersection(lines[0], (lines[1]))
        top_right    = line_intersection(lines[0],(lines[2]))
        bottom_left  = line_intersection(lines[3],(lines[1]))
        bottom_right = line_intersection(lines[3],(lines[2]))
        pts1 = np.float32([top_left,top_right,bottom_left,bottom_right])
        pts2 = np.float32([[0, 0], [0, 400], [400, 0], [400, 400]])
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        result = cv2.warpPerspective(img, matrix, (400, 400))
        return result


def median_multiple_images():
    b_list = []
    g_list = []
    r_list = []
    for name in glob.glob(r'C:\Users\aminb\Documents\GitHub\flush_bot\FlushOS\Flush_Communication\Rail_Photo\*.jpg'):
        img = cv2.imread(str(name))
        img = perspectrive_with_aruco(img)
        print(img[0].shape)
        b_list.append(img[:,:,0])
        g_list.append(img[:,:,1])
        r_list.append(img[:,:,2])
    b_list = np.asarray(b_list)
    g_list = np.asarray(g_list)
    r_list = np.asarray(r_list)
    b_med = np.median(b_list, axis=0)
    g_med = np.median(g_list, axis=0)
    r_med = np.median(r_list, axis=0)
    bgr = np.dstack((b_med,g_med,r_med))
    print(bgr.shape)
    # bgr = crop_img(bgr,0.9)
    cv2.imwrite('test.png',bgr)
    cv2.waitKey(0)

median_multiple_images()
# o = cv2.imread(r'C:\Users\aminb\Documents\GitHub\flush_bot\FlushOS\Flush_Communication\Rail_Photo\Photo5.jpg')
# cv2.imshow("222",perspectrive_with_aruco(o))
# print(perspectrive_with_aruco(o))