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
import random
from flask import Flask, render_template

import threading

__path__ = [""]

from waypoint_updater.path_planner_b import PathPlanner
from twist_controller.ai_controller import AIController
from twist_controller.pid import PID

y = None
z = None
yaw = None
velocity = None
x = None
dbw_enable = None
LOOKAHEAD_WPS = 100
count = 0

TARGET_SPEED = 30  # MPH

SAMPLE_TIME = 0.5
ACCEL_SENSITIVITY = 0.06
speed_controller = PID( ACCEL_SENSITIVITY*1.25, 0.003, 0.0, mn=-0.5, mx=0.5)


sio = socketio.Server( async_handlers=True, max_http_buffer_size=64000, async_mode='eventlet')
app = Flask(__name__)

maps_x = []
maps_y = []

cw_x = None
cw_y = None

dbw_enable = False
data_rdy = False
control_data_rdy = False

pose_lock = threading.Lock()
final_waypoint_lock = threading.Lock()

control_feedback ="-- None --"

pre_throttle_tag = 0
pre_throttle_tag_t = 0


client_sid = None

steering = None
throttle = None
brake = None

def netmon():
    global client_sid, sio

    while True:
        if client_sid is not None:
            print('disconnet client', client_sid)
            sio.disconnect(client_sid, namespace=None)
            client_sid = None
        time.sleep(0.25)    

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

def display_parameters():


    while True:

        if dbw_enable and throttle is not None:
            global start_time
            delta_time = time.time() - start_time
            print('{: .1f}\t<{: .2f} {: .2f} >\t({: .2f} )\t<{: .3f}\t{: .3f}\t{: .2f} >'.format(delta_time, x, y, yaw, steering, throttle, brake))

        time.sleep(0.5)
    pass

def dbw_control3():  # 5Hz
    global dbw_enable
    global steering, throttle, brake

    global data_rdy, control_data_rdy

    steering= 2

    while True:
        if dbw_enable:
            if data_rdy:
                control_data_rdy = False

                if steering > 0.19:
                    steering = -2.0
                else:
                    steering = 2.0

                throttle = 0.05
                brake = 0.0

                control_data_rdy = True

                data_rdy = False

        time.sleep(2.0)

    pass

def find_final_waypoint():  # 0.5Hz
    global dbw_enable
    global steering, throttle, brake

    global data_rdy, control_data_rdy

    fq = 1.0/0.5 #every 2 seconds

    while True:
        if dbw_enable:
            final_waypoint_lock.acquire()
            find_path()
            final_waypoint_lock.release()
        time.sleep(fq)
    pass

def dbw_control():  # 2Hz
    fq = 0.5

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

        time.sleep(fq)

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
    pp = PathPlanner()
    maps_x, maps_y = pp.cleanseWaypoints(True, maps_x, maps_y)

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
        if count == 25:
            #if control_data_rdy:
            send_control()
            #    control_data_rdy = False
            count = 0

    pass

@sio.on('control') #25Hz
def control(sid, data):
    global control_feedback
    control_feedback = data

    global pre_throttle_tag, pre_throttle_tag_t

    global client_sid

    throttle = int(data['throttle']*10000)  #.349603
    if throttle > 100 and throttle == pre_throttle_tag:

        if pre_throttle_tag_t == 0:
            pre_throttle_tag_t = time.time()   # start timer
        else:    
            if time.time() - pre_throttle_tag_t > 2.5:  #2.5 seconds
                print(time.strftime('%X'), '-- possibly bad TCP/IP connection --')
                pre_throttle_tag_t = 0
                #disconnect the client in the monitor
                client_sid = sid
    else:
        pre_throttle_tag_t = 0

    pre_throttle_tag = throttle
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
    global client_sid

    if throttle is None:
        return

    #if client_sid is not None:
    #    sio.emit('dummy','dummy')

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
    if yaw > 360.0:
        yaw = yaw - 360.0
    if yaw < 0.0:
        yaw = yaw + 360.0    
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

    #cw = closest waypoints
    cw_x, cw_y = pp.getNextWaypoints(car_x, car_y, theta, LOOKAHEAD_WPS, maps_x, maps_y)

    #cw_x, cw_y = pp.path_planning(LOOKAHEAD_WPS, car_x, car_y, theta, car_speed, cw_x, cw_y)

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

    if current_y > 2800.0:
       desiredSpeed = 30.0

    dx = sp_x[1]-current_x
    dy = sp_y[1]-current_y
    dist = math.sqrt( dx*dx+dy*dy)

    if dist> 8.0:
        desiredSpeed = 5.0 

    controller = AIController()
  
    #throttle, brake, steering = controller.control(desiredSpeed, currentSpeed, target_x, target_y, current_x, current_y, current_yaw)

    brake = 0.0

    global SAMPLE_TIME
    throttle = speed_controller.step( desiredSpeed-currentSpeed, SAMPLE_TIME)

    # give a small randomized offset to the throttle value
    if throttle > 0.01:
        throttle = throttle + (random.random()-0.5)/100.

    steering = controller.steering(sp_x, sp_y, current_x, current_y, current_yaw) 

    if math.fabs( steering) > 0.4:
        if currentSpeed > 20:
            brake = 5000.0
    elif math.fabs( steering) > 0.3:
        throttle = throttle * 0.2
    elif math.fabs( steering) > 0.3:
        throttle = throttle * 0.2
    elif math.fabs( steering) > 0.2:
        throttle = throttle * 0.3

    if dist > 10.0:
        print(current_x,"\t",current_y,"\t",target_x,"\t",target_y,"\t",dist,"\t",current_yaw,"\t",steering,"\t",throttle,"\t",brake)

    #logging.info(current_x,"\t",current_y,"\t",target_x,"\t",target_y,"\t",current_yaw,"\t",steering,"\t",throttle,"\t",brake)
    
    pass

if __name__ == '__main__':

    #th = threading.Thread(target=worker)
    #th.start()

    # start find_final_waypoint thread
    th = threading.Thread(target=find_final_waypoint)
    th.start()

    th2 = threading.Thread(target=dbw_control)
    th2.start()

    th3 = threading.Thread(target=netmon)
    th3.start()

    th4 = threading.Thread(target=display_parameters)
    th4.start()

    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    print("start WSGI server, backlog=200!")

    # deploy as an eventlet WSGI server
    #    backlog >= 100 (performance issues)
    eventlet.wsgi.server(eventlet.listen(('', 4567),backlog=200), app)
