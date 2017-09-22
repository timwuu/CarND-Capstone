#!/usr/bin/env python

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
from flask import Flask, render_template
#from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport

__path__ = [""]

from waypoint_updater.path_planner import PathPlanner
from twist_controller.ai_controller import AIController
from twist_controller.pid import PID
from twist_controller.yaw_controller import YawController

y = None
z = None
yaw = None
velocity = None
x = None
dbw_enable = None
LOOKAHEAD_WPS = 200
count = 0

##from bridge import Bridge
##from conf import conf

sio = socketio.Server(async_handlers=True, async_mode='eventlet')
app = Flask(__name__)
##bridge = Bridge(conf)
msgs = {}
maps_x = []
maps_y = []

dbw_enable = False

SAMPLE_TIME = 0.5
ACCEL_SENSITIVITY = 0.06
#speed_controller = PID( ACCEL_SENSITIVITY*2.5, 0.006, 1.25, mn=-0.5, mx=0.5)
speed_controller = PID( ACCEL_SENSITIVITY*1.25, 0.003, 0.0, mn=-0.5, mx=0.5)

TARGET_SPEED = 15.0 #MPH default: 10

WHEEL_BASE = 2.8498      # 2.8498 meters Lincoln MKZ
STEER_RATIO = 14.8       # steer angle : wheel angle
MAX_LAT_ACCEL = 3.       # default 3.
MAX_STEER_ANGLE = 4.0    # include both positive and negative sides (-4/+4) radians

STEER_SENSITIVITY = STEER_RATIO / MAX_STEER_ANGLE

MIN_SPEED = 5.  # TODO: adjust

yaw_controller = YawController(WHEEL_BASE, STEER_RATIO, MIN_SPEED, MAX_LAT_ACCEL, MAX_STEER_ANGLE)


#logging.basicConfig(filename='example.log', level=logging.DEBUG)

@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)
    global maps_x, maps_y, pp
    with open('./../../data/sim_waypoints.csv', 'rt') as csvfile:
        maps_x = []
        maps_y = []
        maps_reader = csv.reader(csvfile, delimiter=',')
        for row in maps_reader:
            maps_x.append(float(row[0]))
            maps_y.append(float(row[1]))
    #print("maps_x[len(maps_x)-1]: ", maps_x[len(maps_x)-1])
    #print("maps_y[len(maps_y)-1]: ", maps_y[len(maps_y)-1])
    #print("len(maps):",len(maps))
    pp = PathPlanner()
    maps_x, maps_y = pp.cleanseWaypoints(True, maps_x, maps_y)

def send(topic, data):
    s = 1
    msgs[topic] = data
    #sio.emit(topic, data=json.dumps(data), skip_sid=True)

##bridge.register_server(send)

@sio.on('telemetry')
def telemetry(sid, data):
    global dbw_enable, count
    global steering, throttle, brake


    extract_data(data)



    if dbw_enable:
        count = count + 1
        if count == 25:
            find_path()
            find_actuation()
            #steering=0.0
            #throttle=0.5
            #brake=0.0
            #print( steering, throttle, brake)
            if math.fabs( steering ) > 0.5:
                throttle = min(0.1, throttle)
            send_control( steering, throttle, brake)
            count = 0
    pass

@sio.on('control')
def control(sid, data):
    #print("control",data)
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


def send_control(steering_angle, throttle, brake):
    sio.emit('steer', data={'steering_angle': str(steering_angle)}, skip_sid=True)

    if brake > 0.5:
        sio.emit('brake', data={'brake': str(brake*20.0)}, skip_sid=True)
    else:
        sio.emit('throttle', data={'throttle': str(throttle)}, skip_sid=True)
    pass

def extract_data(data):
    global y, z, yaw, velocity, x, dbw_enable, start_time
    #data = json.loads(json_str)
    y = data['y']
    z = data['z']
    yaw = data['yaw']
    velocity = data['velocity']
    x = data['x']
    if data["dbw_enable"] != dbw_enable:
        dbw_enable = data["dbw_enable"]
        #print("x:",x,"y:",y,"yaw:",yaw,"velocity:",velocity)

        if dbw_enable:
            start_time = time.time()

    pass

def find_path():
    global maps_x, maps_y, x, y, yaw, velocity, pp
    global cw_x, cw_y
    car_x = x
    car_y = y
    theta = yaw
    car_speed = velocity

    #cw = closest waypoints
    cw_x, cw_y = pp.getNextWaypoints(car_x, car_y, theta, LOOKAHEAD_WPS, maps_x, maps_y)


    #TODO: Debug path plan
    cw_x, cw_y = pp.path_planning(LOOKAHEAD_WPS, car_x, car_y, theta, car_speed, cw_x, cw_y)

    '''
    csv = open("writeout.csv","w")
    columnTitleRow = "x, y, x1, y1\n"
    csv.write(columnTitleRow)

    for i in range(0, len(cw_x1)):
        row = str(cw_x[i]) + "," + str(cw_y[i]) + "," + str(cw_x1[i]) + "," + str(cw_y1[i]) + "\n"
        csv.write(row)
    '''

    pass


#Plan A: Original PID controller
#Plan B: Use AI controller

def find_actuation():
    global x, y, yaw, velocity, throttle, brake, steering
    global cw_x, cw_y

    desiredSpeed = TARGET_SPEED
    currentSpeed = velocity
    target_x = cw_x[5]
    target_y = cw_y[5]
    current_x = x
    current_y = y
    current_yaw = yaw

#    if current_x > 2250.0: #or current_y > 2800.0:
#        desiredSpeed = 20.0

    dx = cw_x[1]-current_x
    dy = cw_y[1]-current_y
    dist = math.sqrt( dx*dx+dy*dy)

    if dist> 8.0:
        desiredSpeed = 5.0 



    controller = AIController()
  
    #throttle, brake, steering = controller.control(desiredSpeed, currentSpeed, target_x, target_y, current_x, current_y, current_yaw)

    brake = 0.0

    global SAMPLE_TIME
    throttle = speed_controller.step( desiredSpeed-currentSpeed, SAMPLE_TIME)

    angular_velocity = calcAngularVelocity(cw_x, cw_y, current_yaw)  #radians
    # normalized steering : -1/+1
    #   normalized steering = steer_angle * 2 / MAX_STEER_ANGLE
    #   steer_angle = wheel_angle * STEER_RATIO
    #   normalized steering = wheel_angle * { STEER_RATIO * 2 / MAX_STEER_ANGLE }
    #   STEER_SENSITIVITY = STEER_RATIO * 2 / MAX_STEER_ANGLE
    #
    #   normalized steerting = wheel_angle * STEER_SENSITIVITY
    steering = yaw_controller.get_steering(desiredSpeed, angular_velocity, currentSpeed) * STEER_SENSITIVITY
 
    delta_time = time.time() - start_time

    if steering > 0.:
        print('{: .1f}\t[{: .2f} {: .2f} ]\t{: .2f}\t({: .2f} )\t[<{: .3f}\t{: .3f}\t{: .2f} ]'.format(delta_time,
                                                                                                       current_x,
                                                                                                       current_y, dist,
                                                                                                       current_yaw,
                                                                                                       steering,
                                                                                                       throttle, brake))
    else:
        print('{: .1f}\t[{: .2f} {: .2f} ]\t{: .2f}\t({: .2f} )\t[>{: .3f}\t{: .3f}\t{: .2f} ]'.format(delta_time, current_x, current_y, dist,current_yaw,steering,throttle,brake))
      
    pass

def normalize_angle(theta):

    if theta > 180.0:
        theta = theta - 360.0
    else:
        if theta < -180.0:
            theta = theta + 360.0

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

    theta = normalize_angle(math.degrees(math.atan2(y, x)) - current_yaw)

    # phi: lookahead at the 5th point[6] from the current car position[1] and its angle
    x = cw_x[6] - cw_x[1]
    y = cw_y[6] - cw_y[1]

    phi = normalize_angle(math.degrees(math.atan2(y, x)) - current_yaw)

    targetAngle =  theta * 0.7 + phi * 0.3

    dist = math.sqrt( math.pow(cw_x[6]-cw_x[1],2.0)+math.pow(cw_y[6]-cw_y[1],2.0))

    #print("target angle is: {: f} lookahead distance: {: f}".format( targetAngle, dist))

    angular_velocity = math.radians(targetAngle) * current_speed_mps / dist

    return angular_velocity


if __name__ == '__main__':

    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    print("start WSGI server!")

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
    #eventlet.wsgi.server(eventlet.listen(('127.0.0.1', 4567)), app)
