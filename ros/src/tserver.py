#!/usr/bin/env python

# 2017.09.23 timijk: 
#       1. branch from tt_threading with backlog > 100.
#       2. clean up codes and add the yaw controller.
#       3. convert to radians
#
# 2017.09.24 timijk:
#       1. add MapZone feature

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

__path__ = [""]

from waypoint_updater.path_planner import PathPlanner
from waypoint_updater.map_zone import MapZone
from twist_controller.pid import PID
from twist_controller.yaw_controller import YawController

y = None
z = None
yaw = None
velocity = None
x = None
dbw_enable = False
LOOKAHEAD_WPS = 100
count = 0

cw_x = None
cw_y = None

steering = None
throttle = None
brake = None

maps_x = []
maps_y = []


sio = socketio.Server( async_handlers=True, max_http_buffer_size=64000, async_mode='eventlet')
app = Flask(__name__)

#dbw_enable = False

# ----- threading related -----
data_rdy = False
control_data_rdy = False

pose_lock = threading.Lock()
final_waypoint_lock = threading.Lock()


# ----- control related -----
TARGET_SPEED = 20  # MPH

SAMPLE_TIME = 0.1
ACCEL_SENSITIVITY = 0.06
speed_controller = PID( ACCEL_SENSITIVITY*1.25, 0.003, 0.0, mn=-0.5, mx=0.5)

WHEEL_BASE = 2.8498      # 2.8498 meters Lincoln MKZ
STEER_RATIO = 14.8       # steer angle : wheel angle
MAX_LAT_ACCEL = 3.       # default 3.
MAX_STEER_ANGLE = 4.0    # include both positive and negative sides (-4/+4) radians

STEER_SENSITIVITY = STEER_RATIO / MAX_STEER_ANGLE

MIN_SPEED = 5.  # TODO: adjust

yaw_controller = YawController(WHEEL_BASE, STEER_RATIO, MIN_SPEED, MAX_LAT_ACCEL, MAX_STEER_ANGLE)

# ------ MapZone ------
map_zone = MapZone()

def display_parameters():

    while True:

        if dbw_enable and throttle is not None:
            global start_time
            delta_time = time.time() - start_time
            if steering > 0.0:
                print('{: .1f}\t[{: .2f} {: .2f} ]\t<{: .2f}  \t[{: .3f}\t{: .3f}\t{: .2f} ]'.format(delta_time, x, y, yaw, steering, throttle, brake))
            else:
                print('{: .1f}\t[{: .2f} {: .2f} ]\t {: .2f} >\t[{: .3f}\t{: .3f}\t{: .2f} ]'.format(delta_time, x, y, yaw, steering, throttle, brake))

        time.sleep(0.5)
    pass

def find_final_waypoint():  # 0.5Hz
    global dbw_enable

    slptime = 2.0 #every 2 seconds

    while True:
        if dbw_enable:
            final_waypoint_lock.acquire()
            find_path()
            final_waypoint_lock.release()
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

    while True:
        if dbw_enable:
            if data_rdy and cw_x is not None:
                control_data_rdy = False

                #get the current car position and the yaw angle, the speed
                car_x = x
                car_y = y
                theta = yaw

                car_speed = velocity

                final_waypoint_lock.acquire()
                sp_x, sp_y = pp.path_planning(LOOKAHEAD_WPS, car_x, car_y, theta, car_speed, cw_x, cw_y)
                final_waypoint_lock.release()

                find_actuation()

                if math.fabs(steering) > 0.5:
                    throttle = min(0.1, throttle)

                control_data_rdy = True

                data_rdy = False

        time.sleep(slptime)

    pass

@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid, ':', environ)
    global maps_x, maps_y, pp
    with open('./../../data/sim_waypoints.csv', 'rt') as csvfile:
        maps_x = []
        maps_y = []
        maps_reader = csv.reader(csvfile, delimiter=',')
        for row in maps_reader:
            maps_x.append(float(row[0]))
            maps_y.append(float(row[1]))
    pp = PathPlanner( map_zone)
    maps_x, maps_y = pp.cleanseWaypoints(True, maps_x, maps_y)

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
        count = count + 1
        if count == 2:
            #if control_data_rdy:
            send_control()
            #    control_data_rdy = False
            count = 0

    pass

@sio.on('control') #25Hz
def control(sid, data):
    pass

@sio.on('obstacle')
def obstacle(sid, data):
    #print("obstacle",data)
    pass

@sio.on('lidar')
def obstacle(sid, data):
    #print("lidar",data)
    pass

@sio.on('trafficlights')
def trafficlights(sid, data):
    #print("trafficlights",data)
    pass

@sio.on('image')
def image(sid, data):
    #print("image",data)
    pass

def send_control():
    global steering, throttle, brake

    if throttle is None:
        return

    sio.emit('steer', data={'steering_angle': str(steering)})

    if brake > 0.5:
        sio.emit('brake', data={'brake': str(brake*20.0)})
        sio.emit('throttle', data={'throttle': str(0.0)})
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
    velocity = float(data['velocity'])
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

    global SAMPLE_TIME
    throttle = speed_controller.step( desiredSpeed-currentSpeed, SAMPLE_TIME)

    # give a small randomized offset to the throttle value
    if throttle > 0.01:
        throttle = throttle + (random.random()-0.5)/100.

    angular_velocity = calcAngularVelocity(sp_x, sp_y, current_yaw)  #radians
    # normalized steering : -1/+1
    #   normalized steering = steer_angle * 2 / MAX_STEER_ANGLE
    #   steer_angle = wheel_angle * STEER_RATIO
    #   normalized steering = wheel_angle * { STEER_RATIO * 2 / MAX_STEER_ANGLE }
    #   STEER_SENSITIVITY = STEER_RATIO * 2 / MAX_STEER_ANGLE
    #
    #   normalized steerting = wheel_angle * STEER_SENSITIVITY
    steering = yaw_controller.get_steering(desiredSpeed, angular_velocity, currentSpeed) * STEER_SENSITIVITY

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

    current_speed_mps = velocity * 0.44  # to mps

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

    print("start WSGI server, backlog=200!")

    # deploy as an eventlet WSGI server
    #    backlog >= 100 (performance issues)
    eventlet.wsgi.server(eventlet.listen(('', 4567),backlog=200), app)
