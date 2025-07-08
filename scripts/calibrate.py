#!/usr/bin/env python
import rospy
import numpy as np
import yaml
import os

from geometry_msgs.msg import Point
from gs_interfaces.msg import Orientation

orient_roll, orient_pitch, orient_yaw = [], [], []
gyro_x, gyro_y, gyro_z = [], [], []
accel_x, accel_y, accel_z = [], [], []

data_len = 1000

got_orient = False
got_gyro   = False
got_accel  = False

output_yaml = os.path.expanduser('~/.ros/imu_cov.yaml')

def disp(arrive, bias=0.0):
    return float(np.var([i - bias for i in arrive], ddof = 1))

def try_save():
    if got_orient and got_gyro and got_accel:
        data = {
            'orientation_covariance':  orient_cov,
            'angular_velocity_covariance': gyro_cov,
            'linear_acceleration_covariance': accel_cov
        }
        os.makedirs(os.path.dirname(output_yaml), exist_ok=True)
        with open(output_yaml, 'w') as f:
            yaml.dump(data, f)
        print("wrote covariances to", output_yaml)
        rospy.signal_shutdown("done")

def accel_callback(data):
    global got_accel, accel_cov
    if len(accel_x) < data_len:
        accel_x.append(data.x)
        accel_y.append(data.y)
        accel_z.append(data.z)
        print("мяу")
    else:
        accel_cov = [
            disp(accel_x), 0.0,               0.0,
            0.0,               disp(accel_y), 0.0,
            0.0,               0.0,               disp(accel_z, 9.8)
        ]
        print("accel:", accel_cov)
        accel_sub.unregister()
        got_accel = True
        try_save()

def gyro_callback(data):
    global got_gyro, gyro_cov
    if len(gyro_x) < data_len:
        gyro_x.append(data.x)
        gyro_y.append(data.y)
        gyro_z.append(data.z)
    else:
        gyro_cov = [
            disp(gyro_x), 0.0,       0.0,
            0.0,       disp(gyro_y), 0.0,
            0.0,       0.0,       disp(gyro_z)
        ]
        print("gyro:", gyro_cov)
        gyro_sub.unregister()
        got_gyro = True
        try_save()

def orient_callback(data):
    global got_orient, orient_cov
    if len(orient_roll) < data_len:
        orient_roll.append(data.roll)
        orient_pitch.append(data.pitch)
        orient_yaw.append(data.azimuth)
    else:
        orient_cov = [
            disp(orient_roll), 0.0,            0.0,
            0.0,            disp(orient_pitch), 0.0,
            0.0,            0.0,            disp(orient_yaw)
        ]
        print("orient:", orient_cov)
        orient_sub.unregister()
        got_orient = True
        try_save()

if __name__ == '__main__':
    rospy.init_node('calibrate_cov', anonymous=True)

    # подставь свои реальные топики
    orient_sub = rospy.Subscriber('/geoscan/sensors/orientation', Orientation, orient_callback)
    gyro_sub   = rospy.Subscriber('/geoscan/sensors/gyro',        Point,       gyro_callback)
    accel_sub  = rospy.Subscriber('/geoscan/sensors/accel',       Point,       accel_callback)

    rospy.loginfo("start collecting imu data for cov")
    rospy.spin()
