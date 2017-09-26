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
    global dbw_enable
    if data["dbw_enable"] != dbw_enable:
        dbw_enable = data["dbw_enable"]
        #bridge.publish_dbw_status(dbw_enable)
    #bridge.publish_odometry(data)
    for i in range(len(msgs)):
        topic, data = msgs.pop(0)
        sio.emit(topic, data=data, skip_sid=True)

@sio.on('control')
def control(sid, data):
    #bridge.publish_controls(data)
    pass

@sio.on('obstacle')
def obstacle(sid, data):
    #bridge.publish_obstacles(data)
    pass

@sio.on('lidar')
def lidar(sid, data):
    #bridge.publish_lidar(data)
    pass

@sio.on('trafficlights')
def trafficlights(sid, data):
    #bridge.publish_traffic(data)
    pass

@sio.on('image')
def image(sid, data):
    #bridge.publish_camera(data)
    print('Imagie Received:{}'.format(len(data)))
    imgString = data["image"]
    image = PIL_Image.open(BytesIO(base64.b64decode(imgString)))
    image.show()
    #image_array = np.asarray(image)

    #image_message = self.bridge.cv2_to_imgmsg(image_array, encoding="passthrough")
    #self.publishers['image'].publish(image_message)

    pass

if __name__ == '__main__':

    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    #    increase the backlog size to prevent losing control of the car
    eventlet.wsgi.server(eventlet.listen(('', 4567),backlog=200), app)
