import math
import time
import astar
import rrtstar
import cv2
import numpy as np
import matplotlib.pyplot as plt

img = cv2.imread('some_circles.png',0)
# cv2.imshow('image',img)
mag = 4
scaled_min_radius = int(18 * mag / 4)
scaled_max_radius = int(40 * mag / 4)
disk_locs = []
circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 40,
                                   param1=50, param2=30, minRadius=scaled_min_radius, maxRadius=scaled_max_radius)
if circles is not None:
    circles = np.uint32(np.around(circles))
    for i in circles[0, :]:
        # find center
        # if 50 < i[2] < 80:
        disk_locs.append((i[0], i[1]))

for max_loc in disk_locs:
    img = cv2.drawMarker(img, max_loc, (255, 255, 255), markerType=cv2.MARKER_CROSS,
                              markerSize=20, thickness=1, line_type=cv2.LINE_AA)

    #cv2.putText(img, str(max_loc), max_loc, cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 255, 255), 2,
                #cv2.LINE_AA)

height = np.size(img, 0)
width = np.size(img, 1)
area_x = [0, width-1]
area_y = [0, height-1]
start = disk_locs[1]
disk_locs.remove(start)
start = list(start)
goal = [150,450]
robot_radius = 27.0


img = cv2.drawMarker(img, tuple(goal), (255, 255, 255), markerType=cv2.MARKER_STAR,
                              markerSize=20, thickness=1, line_type=cv2.LINE_AA)

K = 0
if K:
    resolution = 5.0
    a_star = astar.AStarPlanner(area_x, area_y, disk_locs, resolution, robot_radius)
    path = a_star.planning(start, goal)
else:
    expand_dis = 20
    resolution = 5
    goal_sample_rate = 5
    max_iter = 2500
    connect_circle_dist =100.0
    search_until_max_iter = True

    rrt_star = rrtstar.RRTStar(
        start,
        goal,
        disk_locs,
        robot_radius,
        area_x,
        area_y,
        expand_dis,
        resolution,
        goal_sample_rate,
        max_iter,
        connect_circle_dist,
        search_until_max_iter)
    path = rrt_star.planning(animation=False)

path = np.array(path)

path = path.astype('int32')

#a = np.array([[0,0], [20,20], [20,150]])
#print(a)
img = cv2.polylines(img,[path],False, (0, 0, 0),2)
cv2.imshow("Disk locations", img)

cv2.waitKey(0)
