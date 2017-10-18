#!/usr/bin/env python

import socketio
import eventlet
import eventlet.wsgi
import time
from flask import Flask, render_template

from PIL import Image as PIL_Image
from io import BytesIO
import base64

#from bridge import Bridge
#from conf import conf

sio = socketio.Server()
app = Flask(__name__)
#bridge = Bridge(conf)
msgs = []

dbw_enable = False

count = 0

@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)

def send(topic, data):
    s = 1
    msgs.append((topic, data))
    #sio.emit(topic, data=json.dumps(data), skip_sid=True)

#bridge.register_server(send)

@sio.on('telemetry')
def telemetry(sid, data):
    global dbw_enable, count
    if data["dbw_enable"] != dbw_enable:
        dbw_enable = data["dbw_enable"]
        #bridge.publish_dbw_status(dbw_enable)
    #bridge.publish_odometry(data)

    if dbw_enable:
        send_control( 8.0, 0.1, 0.0)

        if count >= 25:
            print( '{: .2f},{: .2f}'.format( data["x"], data["y"]))
            count = 0

    count += 1

    return

    for i in range(len(msgs)):
        topic, data = msgs.pop(0)
        sio.emit(topic, data=data, skip_sid=True)

@sio.on('control')
def control(sid, data):
    #bridge.publish_controls(data)
    pass

@sio.on('obstacle')
def obstacle(sid, data):
    pass

@sio.on('lidar')
def lidar(sid, data):
    pass

@sio.on('trafficlights')
def trafficlights(sid, data):
    pass

@sio.on('image')
def image(sid, data):
    pass

def send_control( steering, throttle, brake):

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


if __name__ == '__main__':

    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    #    increase the backlog size to prevent losing control of the car
    eventlet.wsgi.server(eventlet.listen(('', 4567),backlog=200), app)
