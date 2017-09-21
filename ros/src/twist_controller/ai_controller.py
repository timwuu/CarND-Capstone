# algo of this controller comes from https://github.com/udacity/self-driving-car-sim/blob/43c30490fd1230e387397139bc01432f020860ff/Assets/Standard%20Assets/Vehicles/Car/Scripts/CarAIControl.cs

import numpy
import math

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
BrakeSensitivity = 1.0
AccelSensitivity = 0.04
SteerSensitivity = 0.025  # default = 0.05
ThrottleMin = 0
ThrottleMax = 1
BrakeMin = 0
BrakeMax = 20000

class AIController(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        pass

    def normalize_angle(self, theta):

        if theta > 180.0:
            theta = theta - 360.0
        elif theta < -180.0:
            theta = theta + 360.0

        return theta

        pass       

    #cw_x,cw_y is the interpolated spline curve
    def steering(self, cw_x, cw_y, current_x, current_y, current_yaw):

        #lookahead
        x = cw_x[8] - cw_x[3]
        y = cw_y[8] - cw_y[3]
        
        theta = self.normalize_angle( math.degrees(math.atan2(y, x)) - current_yaw )

        #next
        x = cw_x[5] - current_x
        y = cw_y[5] - current_y
        
        phi = self.normalize_angle( math.degrees(math.atan2(y, x)) - current_yaw )

        dist = math.sqrt(x*x+y*y)

        targetAngle = theta * 0.7 + phi * 0.3

        steering = numpy.clip(targetAngle*SteerSensitivity, -2, 2)

        return steering

        pass

    def control(self, desiredSpeed, currentSpeed, target_x, target_y, current_x, current_y, current_yaw):

        # use different sensitivity depending on whether accelerating or braking:
        if (desiredSpeed < currentSpeed):
            accelBrakeSensitivity = BrakeSensitivity
        else:
            accelBrakeSensitivity = AccelSensitivity

        #decide the actual amount of accel / brake input to achieve desired speed.
        accel = numpy.clip((desiredSpeed - currentSpeed) * accelBrakeSensitivity, -0.5, 0.5)

        #calculate the local - relative position of the target, to steer towards Vector3
        #localTarget = transform.InverseTransformPoint(offsetTargetPos);
        local_target_x = target_x - current_x
        local_target_y = target_y - current_y

        #work out the local angle towards the target float
        #targetAngle = Mathf.Atan2(localTarget.x, localTarget.z) * Mathf.Rad2Deg;
        
        #targetAngle = Mathf.Atan2(localTarget.x, localTarget.z) * Mathf.Rad2Deg;
        targetAngle = math.degrees(math.atan2(local_target_y, local_target_x)) - current_yaw 
        
        targetAngle = self.normalize_angle( targetAngle)

        #get the amount of steering needed to aim the car towards the target
        #steer = Mathf.Clamp(targetAngle * m_SteerSensitivity, -1, 1) * Mathf.Sign(m_CarController.CurrentSpeed);
        steer = numpy.clip(targetAngle*SteerSensitivity, -1.0, 1.0) #TODO: Implement Reverse Gear Change * numpy.sign(currentSpeed)

        if accel > 0:
            throttle = accel
            brake = 0
        else:
            brake = accel * -1
            throttle = 0
        steering = steer

        #print("AIController accel:",accel,",targetAngel:", targetAngel," ,steer:",steer, ",throttle: ", throttle, ",brake: ", brake)

        return throttle, brake, steering
