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
speed_controller = PID( ACCEL_SENSITIVITY*1.25, 0.003, 0.0, mn=-0.5, mx=0.5)

WHEEL_BASE = 2.8498
STEER_RATIO = 14.8  # STEER/WHEEL
MAX_LAT_ACCEL = 3.0
MAX_STEER_ANGLE =  8.  # RAD

yaw_controller = YawController( WHEEL_BASE, STEER_RATIO, min_speed, MAX_LAT_ACCEL, MAX_STEER_ANGLE)

TARGET_SPEED = 35.0 #MPH

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

    steering = controller.steering(cw_x, cw_y, current_x, current_y, current_yaw) 

    #if dist> 3.0 and brake == 1:
    #    brake = 20000
 
    delta_time = time.time() - start_time

    print('{: .1f}\t<{: .2f} {: .2f} >\t{: .2f}\t({: .2f} )\t<{: .3f}\t{: .3f}\t{: .2f} >'.format(delta_time, current_x, current_y, dist,current_yaw,steering,throttle,brake))
    
    if dist > 10.0:
        print(current_x,"\t",current_y,"\t",target_x,"\t",target_y,"\t",dist,"\t",current_yaw,"\t",steering,"\t",throttle,"\t",brake)

    #logging.info(current_x,"\t",current_y,"\t",target_x,"\t",target_y,"\t",current_yaw,"\t",steering,"\t",throttle,"\t",brake)
    
    pass

if __name__ == '__main__':

    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    print("start WSGI server!")

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
    #eventlet.wsgi.server(eventlet.listen(('127.0.0.1', 4567)), app)
