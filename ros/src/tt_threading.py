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

import threading

__path__ = [""]

from waypoint_updater.path_planner import PathPlanner
from twist_controller.ai_controller import AIController

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

sio = socketio.Server(async_mode='eventlet')
app = Flask(__name__)
##bridge = Bridge(conf)
msgs = {}
maps_x = []
maps_y = []

dbw_enable = False
data_rdy = False
control_data_rdy = False

def worker():
    global x, y, yaw
    global count

    while True:
        print ('worker:', count)

        for i in range(0,10000000):
            pass

        count = count + 1

        time.sleep(2)
    return

def dbw_control():  # 5Hz
    global dbw_enable
    global steering, throttle, brake

    global data_rdy, control_data_rdy

    while True:
        if dbw_enable:
            if data_rdy:
                control_data_rdy = False

                find_path()
                find_actuation()
                #steering=0.0
                #throttle=0.5
                #brake=0.0
                #print( steering, throttle, brake)
                if math.fabs(steering) > 0.5:
                    throttle = min(0.1, throttle)

                control_data_rdy = True

                data_rdy = False

        time.sleep(0.2)

    pass

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

@sio.on('telemetry') #25~35Hz
def telemetry(sid, data):
    global dbw_enable, control_data_rdy
    if data["dbw_enable"] != dbw_enable:
        dbw_enable = data["dbw_enable"]

    global data_rdy
    if not data_rdy:
        # get new data
        extract_data(data)
        data_rdy = True

    if control_data_rdy:
        send_control()
        control_data_rdy = False

    pass

@sio.on('control') #25Hz
def control(sid, data):
    #global count
    #print(time.strftime('%X')," #",count," - ", data)
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
    sio.emit('steer', data={'steering_angle': str(steering)}, skip_sid=True)

    if brake > 0.5:
        sio.emit('brake', data={'brake': str(brake*20.0)}, skip_sid=True)
    else:
        sio.emit('throttle', data={'throttle': str(throttle)}, skip_sid=True)
    pass

def extract_data(data):
    global y, z, yaw, velocity, x, dbw_enable
    #data = json.loads(json_str)
    y = data['y']
    z = data['z']
    yaw = data['yaw']
    velocity = data['velocity']
    x = data['x']
    #print("x:",x,"y:",y,"yaw:",yaw,"velocity:",velocity)
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

    cw_x, cw_y = pp.path_planning(LOOKAHEAD_WPS, car_x, car_y, theta, car_speed, cw_x, cw_y)

    pass

def find_actuation():
    global x, y, yaw, velocity, throttle, brake, steering
    global cw_x, cw_y

    desiredSpeed = 10.0
    currentSpeed = velocity
    target_x = cw_x[5]
    target_y = cw_y[5]
    current_x = x
    current_y = y
    current_yaw = yaw

    if current_x > 2250.0 or current_y > 2800.0:
        desiredSpeed = 10.0

    dx = cw_x[1]-current_x
    dy = cw_y[1]-current_y
    dist = math.sqrt( dx*dx+dy*dy)

    if dist> 8.0:
        desiredSpeed = 5.0 

    controller = AIController()
  
    throttle, brake, steering = controller.control(desiredSpeed, currentSpeed, target_x, target_y, current_x, current_y, current_yaw)

    steering = controller.steering(cw_x, cw_y, current_x, current_y, current_yaw) 

    #if dist> 3.0 and brake == 1:
    #    brake = 20000

    #target_x = cw_x[1]
    #target_y = cw_y[1]

    #throttle1, brake1, steering1 = controller.control(desiredSpeed, currentSpeed, target_x, target_y, current_x, current_y, current_yaw)
 
    #steering = (steering + steering1)/2.0
 
    #print(current_x,"\t",current_y,"\t",target_x,"\t",target_y,"\t",dist,"\t",current_yaw,"\t",steering,"\t",throttle,"\t",brake)
    #print(current_x,"\t",current_y,"\t",target_x,"\t",target_y,"\t",dist,"\t",current_yaw,"\t",steering,"\t",throttle,"\t",brake)

    #print(time.strftime('%X'),"\t",dist,"\t",current_yaw,"\t",steering,"\t",throttle,"\t",brake)
    print(time.strftime('%X'),'\t{: f}\t{: f}\t{: f}\t{: f}\t{: f}'.format(dist,current_yaw,steering,throttle,brake))
    
    if dist > 10.0:
        print(current_x,"\t",current_y,"\t",target_x,"\t",target_y,"\t",dist,"\t",current_yaw,"\t",steering,"\t",throttle,"\t",brake)

    #logging.info(current_x,"\t",current_y,"\t",target_x,"\t",target_y,"\t",current_yaw,"\t",steering,"\t",throttle,"\t",brake)
    
    pass

if __name__ == '__main__':

    #th = threading.Thread(target=worker)
    #th.start()

    # start control thread
    th = threading.Thread(target=dbw_control)
    th.start()

    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    print("start WSGI server!")

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
