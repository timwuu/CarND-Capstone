import math

class YawController(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel

        self.min_angle = -max_steer_angle
        self.max_angle = max_steer_angle


        #  steer_angle = wheel_angle( wheel_base, radius ) * steer_ratio
    def get_angle(self, radius):
        angle = math.atan(self.wheel_base / radius) * self.steer_ratio
        return max(self.min_angle, min(self.max_angle, angle))

    def get_steering(self, linear_velocity, angular_velocity, current_velocity):
        angular_velocity = current_velocity * angular_velocity / linear_velocity if abs(linear_velocity) > 0. else 0.

        if abs(current_velocity) > 0.1:
            max_yaw_rate = abs(self.max_lat_accel / current_velocity)
            angular_velocity = max(-max_yaw_rate, min(max_yaw_rate, angular_velocity))

        return self.get_angle(max(current_velocity, self.min_speed) / angular_velocity) if abs(angular_velocity) > 0. else 0.0;

    # target_radius := the radius of the target steering circle 
    # target_angular_velocity := current_velocity / target_radius
    # max_linear_velocity (tangent velocity) 
    #    = sqrt( max_lat_accel * target_radius)
    #    = sqrt( max_lat_accel * (current_velocty/target_angular_velocity)

    def get_max_linear_velocity(self, current_velocity, target_angular_velocity):
        return math.sqrt(self.max_lat_accel * current_velocity / target_angular_velocity)