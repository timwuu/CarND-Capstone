#!/usr/bin/env python

# 
# 2017.09.27 timijk:
#       1. Add 2 more points to SPLINE calculation
#       2. Remove the coordinate transformation used in the SPLINE calculation
#       3. Add distance_sq function to increase performance

import math
import numpy as np
import csv
from scipy import interpolate

import logging

from waypoint_updater.map_zone import MapZone

LANE = -1   # left lane: -1, right lane: +1

class PathPlanner(object):
    def __init__(self, map_zone = None):
        self.waypoints = None
        self.current_pose = None
        self.current_speed = None

        self.frenet_coordinate = None
        self.final_waypoints = None

        self.maps_delta_s = None

        self.map_zone = map_zone

    def path_planning(self, waypoints_size, car_x, car_y, theta, car_speed, maps_x, maps_y):
        # find the next N points

        # Main car's localization Data
        car_s, car_d = self.getFrenet(car_x, car_y, theta, maps_x, maps_y)
        maps_s, maps_d = self.getMapsS(maps_x, maps_y)

        #logging.info('s:{} :{:.2f}'.format(len(maps_s),maps_s[len(maps_s)-1]))

        map_waypoints_s = maps_s
        map_waypoints_x = maps_x
        map_waypoints_y = maps_y

        # Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
        # Later we will interpolate these waypoints with spline and fill it with
        # more points that control spline
        ptsx = []
        ptsy = []

        # reference x,y,yaw states
        # either we will reference the starting point as where the car is or at the previous path's end point
        ref_x = car_x
        ref_y = car_y
        ref_yaw = theta

        # if previous size is almost empty, use the car as starting reference

        # Use two points that make the path tangent to the car
        prev_car_x = car_x - math.cos(ref_yaw)          # This is the same as math.cos(theta) * 1
        prev_car_y = car_y - math.sin(ref_yaw)

        ptsx.append(prev_car_x)
        ptsx.append(car_x)

        ptsy.append(prev_car_y)
        ptsy.append(car_y)

        global LANE
        # In Frenet add evenly 30m spaced points ahead of the starting reference
        lane = LANE   # left lane: +1, right lane: -1
        road_width = 4
        middle_of_lane = 0
        
        #divide waypoints( 90 meters max) by 5 segments, 
        length_end_s = min(maps_s[len(maps_s)-1], 90.0) 

        next_wp0 = self.getXY(car_s + length_end_s*0.2, middle_of_lane + road_width * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y)
        next_wp1 = self.getXY(car_s + length_end_s*0.4, middle_of_lane + road_width * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y)
        next_wp2 = self.getXY(car_s + length_end_s*0.6, middle_of_lane + road_width * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y)
        next_wp3 = self.getXY(car_s + length_end_s*0.8, middle_of_lane + road_width * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y)
        next_wp4 = self.getXY(car_s + length_end_s, middle_of_lane + road_width * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y)

        ptsx.append(next_wp0[0])
        ptsx.append(next_wp1[0])
        ptsx.append(next_wp2[0])
        ptsx.append(next_wp3[0])
        ptsx.append(next_wp4[0])

        ptsy.append(next_wp0[1])
        ptsy.append(next_wp1[1])
        ptsy.append(next_wp2[1])
        ptsy.append(next_wp3[1])
        ptsy.append(next_wp4[1])

        # logging.info('')
        # for i in range(0,7):
        #     logging.info('ptsx.append({:.2f})'.format(ptsx[i]))
        #     logging.info('ptsy.append({:.2f})'.format(ptsy[i]))

        # create a spline
        # set(x,y) points to the spline
        #s = interpolate.splrep(ptsx, ptsy)

        # Define the actual (x,y) points we will use for the planner
        next_x_vals = []
        next_y_vals = []

        # Calculate how to breakup spline points so that we travel at our desired reference velocity

        spline_s = [-1.0, 0.0, length_end_s*0.2, length_end_s*0.4, length_end_s*0.6, length_end_s*0.8, length_end_s]

        pts = []
        for i in range(0, len(ptsx)):
            pts.append([ptsx[i],ptsy[i]])
        cs = interpolate.CubicSpline(spline_s, pts)
        xs = np.linspace(0, length_end_s, 30)
        #plt.figure(figsize = (6.5, 4))
        #plt.plot(cs(xs)[:, 0], cs(xs)[:, 1], label='spline')
        #plt.plot(ptsx, ptsy)

        spoints = cs(xs)

        for i in range(1, len(xs)):
            next_x_vals.append(spoints[i][0])
            next_y_vals.append(spoints[i][1])
  
        return next_x_vals, next_y_vals


    def getMapsS(self, maps_x, maps_y):
        # origin (s,d)
        maps_s = [0.0]
        maps_d = [0.0]
        map_s_accu = 0.0
        for i in range(1, len(maps_x)):
            map_s_accu = map_s_accu + self.maps_delta_s[i]
            maps_s.append(map_s_accu)
            maps_d.append(0.0)
        return maps_s, maps_d

    def findMapDeltaS(self, maps_x, maps_y):
        self.maps_delta_s = [0]
        for i in range(1, len(maps_x)):
             delta = self.distance(maps_x[i-1],maps_y[i-1],maps_x[i],maps_y[i])
             self.maps_delta_s.append(delta)
        return 

    def distance(self, x1, y1, x2, y2):
        d = math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1))
        return d

    def distance_sq(self, x1, y1, x2, y2):
        d = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)
        return d

    # int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
    def ClosestWaypoint(self, x, y, maps_x, maps_y, map_zone=None):

        elems = map_zone.getZoneNeighborElements( x, y) if map_zone is not None else []

        closestLen = 9999999.9  # large number
        closestWaypoint = -1 # -1 if point not found

        if elems ==[]:
            elems = range(len(maps_x)-1, 0, -1)

        #print('elems:{}'.format(elems))

        for i in elems:
            dist = self.distance_sq(x, y, maps_x[i], maps_y[i])
            if (dist < closestLen):
                closestLen = dist
                closestWaypoint = i

        return closestWaypoint

    def NextWaypoint(self, x, y, theta, maps_x, maps_y, map_zone=None):
        closestWaypoint = self.ClosestWaypoint(x, y, maps_x, maps_y, map_zone)
        map_x = maps_x[closestWaypoint]
        map_y = maps_y[closestWaypoint]

        heading = math.atan2((map_y - y), (map_x - x))
        angle = abs(theta - heading)

        if (angle > math.pi / 4):
            closestWaypoint = closestWaypoint + 1

        return closestWaypoint

    def getNextWaypoints(self, x, y, theta, wp_size, maps_x, maps_y):
        nextwaypoint = self.NextWaypoint(x, y, theta, maps_x, maps_y, self.map_zone)
        #### TODO: Assuming this is a loop without overlapping waypoints!
        nextWaypoints_x = []
        nextWaypoints_y = []

        endpoint = min(nextwaypoint+wp_size, len(maps_x))
        nextWaypoints_x = maps_x[nextwaypoint:endpoint]
        nextWaypoints_y = maps_y[nextwaypoint:endpoint]
        if endpoint == len(maps_x):
            nextWaypoints_x = nextWaypoints_x + maps_x[0:wp_size - len(nextWaypoints_x)]
            nextWaypoints_y = nextWaypoints_y + maps_y[0:wp_size - len(nextWaypoints_y)]

        return nextWaypoints_x, nextWaypoints_y


    # If the course is a loop and there are overlapping waypoints, remove the duplicate points so the car may drive continuously
    def cleanseWaypoints(self, is_loop, maps_x, maps_y, start_duplicate = -1):
        if is_loop is not True:
            return

        if start_duplicate == -1:
            #find start duplicate
            search_size = 200  #number of points to search
            map_size = len(maps_x)
            begin_search = map_size - search_size
            start_duplicate = self.ClosestWaypoint(maps_x[0], maps_y[0], maps_x[begin_search:map_size], maps_y[begin_search:map_size], self.map_zone)
            start_duplicate = begin_search + start_duplicate
            print("origin point: ", maps_x[0], maps_y[0])
            print("duplicate point:", maps_x[start_duplicate], maps_y[start_duplicate])
            print("start_duplicate found at index: ", start_duplicate)
        return maps_x[0:start_duplicate - 1], maps_y[0:start_duplicate - 1]

    # Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    # vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
    def getFrenet(self, x, y, theta, maps_x, maps_y):

        next_wp = max( self.NextWaypoint(x, y, theta, maps_x, maps_y), 1)

        prev_wp = next_wp - 1

        try:
            n_x = maps_x[next_wp] - maps_x[prev_wp]
            n_y = maps_y[next_wp] - maps_y[prev_wp]
            x_x = x - maps_x[prev_wp]
            x_y = y - maps_y[prev_wp]
        except:
            print("next_wp",next_wp)
            pass


        # find the projection of x onto n
        proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y)
        proj_x = proj_norm * n_x
        proj_y = proj_norm * n_y

        frenet_d = self.distance(x_x, x_y, proj_x, proj_y)

        # see if d value is positive or negative by comparing it to the two points
        # that locate on each side of the track

# '''
#         point_right_x = n_y
#         point_right_y = - n_x

#         point_left_x = -n_y
#         point_left_y = n_x

#         dist_right_sq = (x_x - point_right_x) ** 2.0 + (x_y - point_right_y) ** 2.0
#         dist_left_sq = (x_x - point_left_x) ** 2.0 + (x_y - point_left_y) ** 2.0

#         if dist_right_sq < dist_left_sq:  # positive means d is on the left side
#             frenet_d = -frenet_d
# '''

        if (x_y*n_x < x_x*n_y): # use the inner product to decide
            frenet_d = -frenet_d

        # calculate s value
        frenet_s = 0
        for i in range(0, prev_wp):
            frenet_s += self.maps_delta_s[i]

        frenet_s += self.distance(0, 0, proj_x, proj_y)

        return frenet_s, frenet_d


    # Transform from Frenet s,d coordinates to Cartesian x,y
    # vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
    def getXY(self, s, d, maps_s, maps_x, maps_y):


        prev_wp = -1

        # TODO: Currently, we assume there exists at least one point on maps_s
        for i in range(0, len(maps_s)-1):
            if s >= maps_s[i]:
                prev_wp = i
            else:
                break

        if prev_wp == -1:
            return maps_x[0], maps_y[0]

        wp2 = (prev_wp + 1) % len(maps_x)

        heading = math.atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]))

        # the x,y,s along the segment
        seg_s = (s - maps_s[prev_wp])

        seg_x = maps_x[prev_wp] + seg_s * math.cos(heading)
        seg_y = maps_y[prev_wp] + seg_s * math.sin(heading)

        perp_heading = heading - math.pi / 2

        x = seg_x + d * math.cos(perp_heading)
        y = seg_y + d * math.sin(perp_heading)

        return x, y
