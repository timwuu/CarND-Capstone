#!/usr/bin/env python

import csv
import socketio
import eventlet
import eventlet.wsgi
import time
from flask import Flask, render_template

from PIL import Image as PIL_Image
from io import BytesIO
import base64

from waypoint_updater.path_planner import PathPlanner
from waypoint_updater.map_zone import MapZone
from twist_controller.pid import PID
from twist_controller.yaw_controller import YawController

#from bridge import Bridge
#from conf import conf

MPH2mps = 0.44704

sio = socketio.Server()
app = Flask(__name__)
#bridge = Bridge(conf)
msgs = []

img_filenames = []

dbw_enable = False

img_count = 0
data_count = 0

x=0.0
y=0.0
z=0.0
yaw=0.0
velocity=0.0

maps_x = []
maps_y = []
map_index = {}
map_zone = MapZone()

lights_x = []
lights_y = []
traffic_zones = []
light_state = None

brake_test = False

static_lights_keys = [11701190, 15801160, 21301550, 21801820, 14702950, 8002910, 1602280, 3601550]
static_lights_check_zones = [11301180, 11401180, 11501180, 11501190, 11601190, 15301160, 15401160, 15501160, 15601160, 15701160, 21201510, 21201520, 21201530, 21201540, 21301540, 21701780, 21701790, 21701800, 21801800, 21801810, 15202950, 15102950, 15002950, 14902950, 14802950, 8502910, 8402910, 8302910, 8202910, 8102910, 1602330, 1602320, 1602310, 1602300, 1602290, 3501590, 3501580, 3501570, 3601570, 3601560]   

def send_control(steering_angle, throttle, brake):
    global velocity, brake_test

    sio.emit('steer', data={'steering_angle': str(steering_angle)}, skip_sid=True)

    if velocity > 20:
        brake_test = True
        sio.emit('throttle', data={'throttle': str(0)}, skip_sid=True)

    if brake_test:    
        sio.emit('brake', data={'brake': str(250)}, skip_sid=True)
    else:
        sio.emit('throttle', data={'throttle': str(throttle)}, skip_sid=True)

    '''
    if brake > 0.5:
        sio.emit('brake', data={'brake': str(brake*20.0)}, skip_sid=True)
    else:
        sio.emit('throttle', data={'throttle': str(throttle)}, skip_sid=True)
    '''

    pass   


def extract_data(data):
    global y, z, yaw, velocity, x, dbw_enable, start_time
    #data = json.loads(json_str)
    y = data['y']
    z = data['z']
    yaw = data['yaw']
    velocity = data['velocity']
    x = data['x']

@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)
    global maps_x, maps_y, pp, map_zone
    with open('./../../data/sim_waypoints.csv', 'rt') as csvfile:
        maps_x = []
        maps_y = []
        maps_reader = csv.reader(csvfile, delimiter=',')
        for row in maps_reader:
            maps_x.append(float(row[0]))
            maps_y.append(float(row[1]))
    print("maps_x[len(maps_x)-1]: ", maps_x[len(maps_x)-1])
    print("maps_y[len(maps_y)-1]: ", maps_y[len(maps_y)-1])
    #print("len(maps):",len(maps))
    map_zone.clear()
    pp = PathPlanner( map_zone)
    maps_x, maps_y = pp.cleanseWaypoints(True, maps_x, maps_y)

    #create map zones
    map_zone.zoning( maps_x, maps_y)

def send(topic, data):
    msgs.append((topic, data))
    #sio.emit(topic, data=json.dumps(data), skip_sid=True)

#bridge.register_server(send)

@sio.on('telemetry')
def telemetry(sid, data):
    global start_time, x, y
    global dbw_enable
    if data["dbw_enable"] != dbw_enable:
        dbw_enable = data["dbw_enable"]
        if dbw_enable:
            start_time = time.time()
            pass

    global velocity, data_count

    extract_data( data)

    if dbw_enable:

        if data_count==5:
            time_diff = time.time() - start_time
            print('{: .2f}, {: .2f}, {: .2f}, {: .2f}'.format(x, y, time_diff, velocity*MPH2mps))
            data_count = 0

            steering_angle = 0
            throttle = 0.2  
            brake = 0
            send_control(steering_angle, throttle, brake)

        data_count += 1

    #bridge.publish_dbw_status(dbw_enable)
    #bridge.publish_odometry(data)

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
    #bridge.publish_obstacles(data)
    pass

@sio.on('lidar')
def lidar(sid, data):
    #bridge.publish_lidar(data)
    pass

@sio.on('trafficlights')
def trafficlights(sid, data):

    return

    global lights_x, lights_y, map_zone
    global light_state

    light_state = data['light_state'][0]

    return 

    lights_x = data['light_pos_x']
    lights_y = data['light_pos_y']

    lights_keys = []
    for i in range(0, len(lights_x)):
        lights_keys.append(map_zone.getKey(lights_x[i], lights_y[i]))
    
    #print(lights_keys)    

    #bridge.publish_traffic(data)
    pass

@sio.on('image')
def image(sid, data):

    return 


    global x, y, light_state, start_time
    global img_filenames
    key = map_zone.getKey(x, y)
    if  key in static_lights_check_zones:
        return

    light_state=999
    #print('Imagie Received:{}'.format(len(data)))
    global img_count
    if img_count == 15:
        #print (light_state)
        imgString = data["image"]
        image = PIL_Image.open(BytesIO(base64.b64decode(imgString)))
        #image.show()
        current_time = int(time.time() - start_time)
        filename = "./data/{}/{}-{}-{}-{}.png".format(light_state, key, int(x), int(y), light_state )

        if not (filename in img_filenames):
            img_filenames.append (filename)
            print(filename)
            image.save(filename,"png")
        img_count = 0
    img_count += 1
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
