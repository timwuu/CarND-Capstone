#!/usr/bin/env python

# 2017.09.23 timijk: 
#       1. branch from tt_threading with backlog > 100.
#       2. clean up codes and add the yaw controller.
#       3. convert to radians
#
# 2017.09.24 timijk:
#       1. add MapZone feature
#
# 2017.09.29 timijk:
#       1. convert speed/velocity to meter per second
#
# 2017.09.30 timijk:
#       1. slow down the speed if the lateral acceleration is too large
#       2. bug fix. map_zone.clear()
#
# 2017.10.17 timijk: bug fix
#       1. YawController.get_steering() returns steer angle and no need to normalize
#
# 2017.10.17 timijk: bug fix
#       1. update YawController's parameters to allow larger steering angles
#
# 2017.10.19 timijk: test with churchlot_with_cars.csv waypoints (working in progress)
#       1. for the track waypoints requiring sharp turns, an accurate next waypoint is required.

import csv
import json
import sys
import os
import socketio
import eventlet
import eventlet.wsgi
import time
import logging
import math
import random
from flask import Flask, render_template

import threading
import copy

__path__ = [""]

from waypoint_updater.path_planner import PathPlanner
from waypoint_updater.map_zone import MapZone
from twist_controller.pid import PID
from twist_controller.yaw_controller import YawController
from twist_controller.lowpass import LowPassFilter

import logging

y = None
z = None
yaw = None
velocity = None
x = None
dbw_enable = False
LOOKAHEAD_WPS = 25
count = 0

cw_x = None
cw_y = None

steering = None
throttle = None
brake = None

maps_x = []
maps_y = []

time_path_planning = 0.0
time_find_path = 0.0


sio = socketio.Server( async_handlers=True, async_mode='eventlet')
app = Flask(__name__)

#dbw_enable = False

# ----- threading related -----
data_rdy = False
control_data_rdy = False

pose_lock = threading.Lock()
final_waypoint_lock = threading.Lock()

final_waypoint_updated = None

# ----- control related -----
MPH2MPS = 0.44704 # miles/hr to m/s
TARGET_SPEED = 7.0 * MPH2MPS  # meter per second

SAMPLE_TIME = 0.1  #defalut 0.1
ACCEL_SENSITIVITY = 0.06
# mx=0.2: means acc.max = 1m/s^2
speed_controller = PID( ACCEL_SENSITIVITY*1.25, 0.003, 0.0, mn=-1.0, mx=0.4)

WHEEL_BASE = 2.8498      # 2.8498 meters Lincoln MKZ
STEER_RATIO = 14.8 # default:14.8       # steer angle : wheel angle
MAX_LAT_ACCEL = 3.0       # default 3.
MAX_STEER_ANGLE = 8.0    # include both positive and negative sides (-4/+4) radians

STEER_SENSITIVITY = STEER_RATIO / MAX_STEER_ANGLE

MIN_SPEED = 0.5  # TODO: adjust

yaw_controller = YawController(WHEEL_BASE, STEER_RATIO, MIN_SPEED, MAX_LAT_ACCEL, MAX_STEER_ANGLE)

steering_LPF = LowPassFilter(8.0,2.0)

# ------ MapZone ------
map_zone = MapZone()

#---- logging -----

logging.basicConfig( filename='Z:\TEMP\output.txt', level=logging.INFO)

logging.info('Hello Logging!{}'.format(time.strftime('%X')))

def display_parameters():

    while True:

        if dbw_enable and throttle is not None:
            delta_time = time.time() - start_time
            if steering > 0.0:
                fmt_string = '{: .1f} {:.2f} {:.2f}\t[{: .2f} {: .2f} ]\t<{: .2f}  \t[{: .3f}\t{: .3f}\t{: .2f} ]'
            else:
                fmt_string = '{: .1f} {:.2f} {:.2f}\t[{: .2f} {: .2f} ]\t {: .2f} >\t[{: .3f}\t{: .3f}\t{: .2f} ]'

            print(fmt_string.format(delta_time, time_find_path*1000.0, time_path_planning*1000.0, x, y, yaw, steering, throttle, brake))

        time.sleep(0.5)
    pass

def find_final_waypoint():  # 0.5Hz
    global dbw_enable

    slptime = 0.25 #default 2: every 2 seconds

    global time_find_path, final_waypoint_updated

    while True:
        if dbw_enable:
            time_find_path_ = time.time()
            final_waypoint_lock.acquire()
            find_path()
            final_waypoint_lock.release()
            time_find_path = time.time() - time_find_path_
            final_waypoint_updated = True

        time.sleep(slptime)
    pass

def dbw_control():
    slptime = SAMPLE_TIME

    global dbw_enable
    global steering, throttle, brake

    global data_rdy, control_data_rdy

    global x, y, yaw, velocity, pp
    global cw_x, cw_y
    global sp_x, sp_y

    global time_path_planning
    global final_waypoint_updated

    while True:
        if dbw_enable:
            if data_rdy and final_waypoint_updated is not None:
                time_path_planning_ = time.time()
                control_data_rdy = False

                #get the current car position and the yaw angle, the speed
                car_x = x
                car_y = y
                theta = yaw

                car_speed = velocity

                if final_waypoint_updated:
                    final_waypoint_lock.acquire()
                    cw_x_copy = copy.deepcopy(cw_x)
                    cw_y_copy = copy.deepcopy(cw_y)
                    final_waypoint_lock.release()
                    final_waypoint_updated = False            
                    pp.findMapDeltaS( cw_x_copy, cw_y_copy)

                idx = ClosestWaypoint2(car_x, car_y, theta, cw_x_copy, cw_y_copy)
                
                if idx > 1:
                    cw_x_copy = cw_x_copy[idx-1:len(cw_x_copy)]
                    cw_y_copy = cw_y_copy[idx-1:len(cw_y_copy)]
                    pp.maps_delta_s = pp.maps_delta_s[idx-1:len(pp.maps_delta_s)]
                    pp.maps_delta_s[0] = 0.0

                sp_x, sp_y = pp.path_planning(LOOKAHEAD_WPS, car_x, car_y, theta, car_speed, cw_x_copy, cw_y_copy)

                sp_x = cw_x_copy
                sp_y = cw_y_copy

                #print('{: .2f},{: .2f}\n{: .2f},{: .2f}'.format(car_x,car_y,sp_x[0],sp_y[0]))    

                # logging.info('-------- car pose ---------')
                # logging.info('{:.2f}, {:.2f}, {:.2f}'.format(car_x, car_y,theta))

                # logging.info('-------- cw waypoints ---------')
                # for i in range(0,len(cw_x_copy)):
                #     logging.info('{:.2f}, {:.2f}'.format(cw_x_copy[i],cw_y_copy[i]))

#                logging.info('')
#                 for i in range(0,len(sp_x)):
#                     logging.info('{:.2f}, {:.2f}'.format(sp_x[i], sp_y[i]))

#                 logging.info('')
#                 logging.info('')
# '''

                find_actuation2()

                if math.fabs(steering) > 0.5:
                    throttle = min(0.1, throttle)

                control_data_rdy = True

                data_rdy = False

                time_path_planning = time.time() - time_path_planning_
        time.sleep(slptime)

    pass

def ClosestWaypoint( x, y, maps_x, maps_y):

    closestLen_sq = 10000000.0;  # large number
    closestWaypoint = -1; # -1 if point not found

    for i in range(len(maps_x)-1, 0, -1):
        dist = distance_sq(x, y, maps_x[i], maps_y[i])
        if (dist < closestLen_sq):
            closestLen_sq = dist
            closestWaypoint = i

    return closestWaypoint + 1   


def ClosestWaypoint2( x, y, theta, maps_x, maps_y):

    closestLen_sq = 10000000.0;  # large number
    closestWaypoint = -1; # -1 if point not found

    for i in range(len(maps_x)-1, 0, -1):
        dist = distance_sq(x, y, maps_x[i], maps_y[i])
        if (dist < closestLen_sq):
            closestLen_sq = dist
            closestWaypoint = i

    map_x = maps_x[closestWaypoint]
    map_y = maps_y[closestWaypoint]

    heading = math.atan2((map_y - y), (map_x - x))
    angle = abs(theta - heading)

    if (angle > math.pi / 4.0):
        closestWaypoint = closestWaypoint + 1

    return closestWaypoint

def distance_sq( x1, y1, x2, y2):
    return (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)

@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid, ':', environ)
    global maps_x, maps_y, pp
#    with open('./../../data/sim_waypoints.csv', 'rt') as csvfile:
#    with open('./../../data/wp_yaw_const.txt', 'rt') as csvfile:

    base_x = 1118.72  #1131.22
    base_y = 1181.38  #1183.27

    with open('./../../data/churchlot_with_cars.csv', 'rt') as csvfile:
        maps_x = []
        maps_y = []
        maps_reader = csv.reader(csvfile, delimiter=',')
        for row in maps_reader:
            maps_x.append(float(row[0]) + base_x)
            maps_y.append(float(row[1]) + base_y)

    map_zone.clear()    #bug fix: 2017.09.30    
    pp = PathPlanner( map_zone)
    # maps_x, maps_y = pp.cleanseWaypoints(True, maps_x, maps_y)

    #create map zones
    map_zone.zoning( maps_x, maps_y)

@sio.on('telemetry') #25~35Hz
def telemetry(sid, data):
    global dbw_enable, control_data_rdy, start_time
    if data["dbw_enable"] != dbw_enable:
        dbw_enable = data["dbw_enable"]

        if dbw_enable:
            start_time = time.time()

    global data_rdy
    global pose_lock, count

    if not data_rdy:
        # get new data
        pose_lock.acquire()  #blocking: waiting for acquire the lock
        extract_data(data)
        pose_lock.release()

        data_rdy = True

    if dbw_enable:
#        count = count + 1
#        if count == 2:
        if control_data_rdy:
            send_control()
            control_data_rdy = False

    pass

@sio.on('control') #25Hz
def control(sid, data):
    pass

@sio.on('obstacle')
def obstacle(sid, data):
    #print("obstacle",data)
    pass

@sio.on('lidar')
def lidar(sid, data):
    #print("lidar",data)
    pass

@sio.on('trafficlights')
def trafficlights(sid, data):
    #print("trafficlights",data)
    pass

@sio.on('image')
def image(sid, data):
    #print("image",data)
    #print('>')
    pass

def send_control():
    global steering, throttle, brake

    if throttle is None:
        return

    sio.emit('steer', data={'steering_angle': str(steering)})

    if brake > 0.5:
        sio.emit('throttle', data={'throttle': str(0.0)})
        sio.emit('brake', data={'brake': str(brake)})
    else:
        sio.emit('throttle', data={'throttle': str(throttle)})
        #sio.emit('brake', data={'brake': str(0.1)})
    pass

def extract_data(data):
    global y, z, yaw, velocity, x, dbw_enable
    #data = json.loads(json_str)
    y = float(data['y'])
    z = float(data['z'])
    yaw = float(data['yaw'])
    velocity = float(data['velocity']) * MPH2MPS  # convert to m/s
    x = float(data['x'])

    #yaw data cleansing
    if yaw > 180.0:
        yaw = yaw - 360.0
    if yaw < -180.0:
        yaw = yaw + 360.0

    # convert to -pi/+pi
    yaw = math.radians(yaw)   

    pass

def find_path():
    global maps_x, maps_y, x, y, yaw, velocity, pp
    global cw_x, cw_y

    global pose_lock
    pose_lock.acquire()
    car_x = x
    car_y = y
    theta = yaw
    car_speed = velocity
    pose_lock.release()

    cw_x, cw_y = pp.getNextWaypoints(car_x, car_y, theta, LOOKAHEAD_WPS, maps_x, maps_y)

    pass

def find_actuation():
    global x, y, yaw, velocity, throttle, brake, steering
    global sp_x, sp_y
    global TARGET_SPEED

    desiredSpeed = TARGET_SPEED
    currentSpeed = velocity
    target_x = sp_x[5]
    target_y = sp_y[5]
    current_x = x
    current_y = y
    current_yaw = yaw

    dx = sp_x[1]-current_x
    dy = sp_y[1]-current_y
    dist = math.sqrt( dx*dx+dy*dy)
  
    brake = 0.0

    # calculate angular velocity (implies radius/curvature) first
    angular_velocity = calcAngularVelocity(sp_x, sp_y, current_yaw)  #radians

    # check the max tangent speed if angular_velocity > 0.025
    if angular_velocity > 0.025:
        v = yaw_controller.get_max_linear_velocity( velocity, angular_velocity)

        if desiredSpeed > v:
            #print('[{: .2f}{: .2f}] max:{: .2f} target:{: .2f}'.format(x, y, v,desiredSpeed))
            desiredSpeed = v

        if velocity > v:
            brake = 250.0
            throttle = 0.0
            speed_controller.reset()
            print('SHARPTURN - SLOWDOWN! Brake{: .2f} Applied!'.format(brake))


    global SAMPLE_TIME

    if brake == 0.0:
        throttle = speed_controller.step( desiredSpeed-currentSpeed, SAMPLE_TIME)

        if throttle < 0.0:
            print('OVERSPEED - SLOWDOWN! throttle:{: .2f}'.format(throttle))
            brake = 250.0
            throttle = 0.0

    # give a small randomized offset to the throttle value
    if throttle > 0.01:
        throttle = throttle + (random.random()-0.5)/100.

    # 2017.10.17 timijk, bug fix: get_steering returns steer_angle and no need to normalize
    steering = yaw_controller.get_steering(desiredSpeed, angular_velocity, currentSpeed)

    #steering = steering_LPF.filt(steering)

    pass

    # not using pathplanner version
def find_actuation2():
    global x, y, yaw, velocity, throttle, brake, steering
    global sp_x, sp_y
    global TARGET_SPEED

    desiredSpeed = TARGET_SPEED
    currentSpeed = velocity
    current_x = x
    current_y = y
    current_yaw = yaw
  
    brake = 0.0

    dx = sp_x[1] - sp_x[0]
    dy = sp_y[1] - sp_y[0]
    phi = math.atan2(dy, dx)

    # calculate angular velocity (implies radius/curvature) first
    dx = sp_x[0] - current_x
    dy = sp_y[0] - current_y

    dist = math.sqrt( dx*dx+dy*dy)

    phi = normalize_angle( phi - math.atan2(dy, dx))

    targetAngle = normalize_angle(math.atan2(dy, dx) - current_yaw) + 0.7 * phi

    angular_velocity = targetAngle * currentSpeed / dist

    if angular_velocity > 0.025 and desiredSpeed > 0.1:
        desiredSpeed = max(desiredSpeed / 3.86, 0.1)
    
    global SAMPLE_TIME

    if brake == 0.0:
        throttle = speed_controller.step( desiredSpeed-currentSpeed, SAMPLE_TIME)

        if throttle < 0.0:
            # 2017.10.18 print('OVERSPEED - SLOWDOWN! throttle:{: .2f}'.format(throttle))
            # 2017.10.18 brake = 250.0
            throttle = 0.0

    # give a small randomized offset to the throttle value
    if throttle > 0.01:
        throttle = throttle + (random.random()-0.5)/100.

    # 2017.10.17 timijk, bug fix: get_steering returns steer_angle and no need to normalize
    steering = yaw_controller.get_steering(desiredSpeed, angular_velocity, currentSpeed)
    # steering = 8.0

    #steering = steering_LPF.filt(steering)

    pass

def normalize_angle(theta):

    if theta > math.pi:
        theta = theta - 2.0*math.pi
    else:
        if theta < -math.pi:
            theta = theta + 2.0*math.pi

    return theta

    #radians
def calcAngularVelocity(cw_x, cw_y, current_yaw):
    global velocity

    current_speed_mps = velocity  # mps

    # target_angle * current_speed / distance between 5th point and the current point (~ 15m,
    # we use 90m spline and divides it into 30 segments)

    # theta: the tagency of the 5th point[6] from the current car position[1]
    x = cw_x[7] - cw_x[5]
    y = cw_y[7] - cw_y[5]

    #theta = normalize_angle(math.degrees(math.atan2(y, x)) - current_yaw)
    theta = normalize_angle(math.atan2(y, x) - current_yaw)

    # phi: lookahead at the 5th point[6] from the current car position[1] and its angle
    x = cw_x[6] - cw_x[1]
    y = cw_y[6] - cw_y[1]

    phi = normalize_angle(math.atan2(y, x) - current_yaw)

    targetAngle =  theta * 0.7 + phi * 0.3

    dist = math.sqrt( math.pow(cw_x[6]-cw_x[1],2.0)+math.pow(cw_y[6]-cw_y[1],2.0))

    #print("target angle is: {: f} lookahead distance: {: f}".format( targetAngle, dist))

    #angular_velocity = math.radians(targetAngle) * current_speed_mps / dist
    angular_velocity = targetAngle * current_speed_mps / dist

    return angular_velocity


if __name__ == '__main__':

    # start find_final_waypoint thread
    th = threading.Thread(target=find_final_waypoint)
    th.start()

    th2 = threading.Thread(target=dbw_control)
    th2.start()

    th4 = threading.Thread(target=display_parameters)
    th4.start()

    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    print("start WSGI server, backlog>=200!")

    # deploy as an eventlet WSGI server
    #    backlog >= 100 (performance issues)
    eventlet.wsgi.server(eventlet.listen(('', 4567),backlog=500), app)
