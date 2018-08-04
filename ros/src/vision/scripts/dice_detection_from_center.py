import numpy as np
import cv2
from math import pi
import math

cap1 = cv2.VideoCapture(0)
cap2 = cv2.VideoCapture('foot.mp4')

indicator = 0
## Processing Webcam or Video
text = input("Do you want use webcam(Yes/No) : ")
if (text == "Yes" or text == "yes" or text == 'y'):
    print("Processing Webcam....\n")
else:
    print("Processing Video....\n")
    indicator = 1

cv2.namedWindow('frame', cv2.WINDOW_NORMAL)

while (True):
    # Capture frame-by-frame
    if indicator == 1:
        ret, frame = cap2.read()
    else:
        ret, frame = cap1.read()

    image_size = frame.shape
    y,x,_ = image_size
    mid_point = (int(x/2),int(y/2))

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # orignal = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    orignal = frame.copy()
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    gray = cv2.medianBlur(gray, 5)

    gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 3.5)

    kernel = np.ones((3, 3), np.uint8)
    gray = cv2.erode(gray, kernel, iterations=1)
    gray = cv2.dilate(gray, kernel, iterations=1)

    _, contours, hierarchy = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    contours_list = []
    area_list = []
    center_list = []

    for contour in contours:
        approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
        area = cv2.contourArea(contour)
        (x, y), radius = cv2.minEnclosingCircle(contour)
        cal_area = radius * radius * pi
        area2 = 1.6 * area
        if ((len(approx) > 8) & (len(approx) < 23) & (200 > area > 50) & (cal_area < area2)):
            contours_list.append(contour)
            center = (int(x), int(y))
            center_list.append(center)
            area_list.append(area)

    distance_list = []
    min_list1 = []
    min_list2 = []

    for center_1 in center_list:
        x1,y1 = center_1
        for center_2 in center_list:
            x2,y2 = center_2
            if ((x1 != x2) & (y1 != y2)):
                distance = math.hypot(x2-x1, y2-y1)
                distance_list.append(distance)

    for center_1 in center_list:
        x1,y1 = center_1
        for center_2 in center_list:
            x2,y2 = center_2
            distance = math.hypot(x2-x1, y2-y1)
            if ((x1 != x2) & (y1 != y2)):
                if (distance < (3*min(distance_list))):
                    if center_1 not in min_list1 and center_2 not in min_list2:
                        min_list1.append(center_1)
                        min_list2.append(center_2)


    clusterA_x = []
    clusterA_y = []
    clusterB_x = []
    clusterB_y = []

    for x1,y1 in min_list1:
        for x2,y2 in min_list2:
            if ((x1 != x2) & (y1 != y2)):
                x = abs(x1-x2)
                y = abs(y1-y2)
                if x > 100 or y >70:
                    if x1 not in clusterA_x and x1 not in clusterB_x and y1 not in clusterA_y and y1 not in clusterB_y:
                        if x2 not in clusterA_x and x2 not in clusterB_x and y2 not in clusterA_y and y2 not in clusterB_y:
                            if x1 > np.mean(clusterA_x)+30:
                                clusterB_x.append(x1)
                            elif y1 > np.mean(clusterA_y)+20:
                                clusterB_y.append(y1)
                            else:
                                clusterA_x.append(x1)
                                clusterA_y.append(y1)
                                clusterB_x.append(x2)
                                clusterB_y.append(y2)

    if clusterA_x and clusterB_y and clusterA_y and clusterB_x is not 0:
        mid_ax = min(clusterA_x)
        mid_ay = min(clusterA_y)
        mid_bx = max(clusterB_x)
        mid_by = max(clusterB_y)

        a_center = (int(mid_ax),int(mid_ay))
        b_center = (int(mid_bx),int(mid_by))

        cv2.line(orignal,mid_point,a_center,(255,0,0),5)
        cv2.line(orignal,mid_point,b_center,(0,0,255),5)

    cv2.drawContours(orignal, contours_list, -1, (255, 0, 0), 1)

    min_list1.clear()
    min_list2.clear()
    clusterA_x.clear()
    clusterB_x.clear()
    clusterA_y.clear()
    clusterB_y.clear()

    # Display the resulting frame
    cv2.imshow('frame', orignal)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap1.release()
cap2.release()
cv2.destroyAllWindows()