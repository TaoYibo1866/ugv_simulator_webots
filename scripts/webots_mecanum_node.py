#!/usr/bin/env python2

import rospy
from tf.transformations import quaternion_multiply
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
from mecanum_simulator.msg import MecanumSensor
from controller import Robot
import os
from math import sin, cos, pi

cmd_vel = Twist()

def command_callback(msg):
    global cmd_vel
    cmd_vel = msg

robot = Robot()
time_step = int(robot.getBasicTimeStep())

# Motor
wheel1 = robot.getDevice('wheel1')
wheel2 = robot.getDevice('wheel2')
wheel3 = robot.getDevice('wheel3')
wheel4 = robot.getDevice('wheel4')
wheel1.setPosition(float('inf'))
wheel2.setPosition(float('inf'))
wheel3.setPosition(float('inf'))
wheel4.setPosition(float('inf'))
wheel1.setVelocity(0)
wheel2.setVelocity(0)
wheel3.setVelocity(0)
wheel4.setVelocity(0)

# Position Sensor
wheel1sensor = robot.getDevice('wheel1sensor')
wheel2sensor = robot.getDevice('wheel2sensor')
wheel3sensor = robot.getDevice('wheel3sensor')
wheel4sensor = robot.getDevice('wheel4sensor')
wheel1sensor.enable(time_step)
wheel2sensor.enable(time_step)
wheel3sensor.enable(time_step)
wheel4sensor.enable(time_step)

# IMU
inertial_unit = robot.getDevice('inertial unit')
inertial_unit.enable(time_step)

# Gyro
gyro = robot.getDevice('gyro')
gyro.enable(time_step)

robot.step(time_step)
prev_pos1 = wheel1sensor.getValue()
prev_pos2 = wheel2sensor.getValue()
prev_pos3 = wheel3sensor.getValue()
prev_pos4 = wheel4sensor.getValue()
qENU2NUE = [cos(pi / 4), 0, 0, sin(pi / 4)]
a = 0.228
b = 0.158
r = 0.045

rospy.init_node('webots_mecanum', anonymous=True)
rospy.Subscriber('/cmd_vel', Twist, command_callback)
pub = rospy.Publisher('mecanum_sensor', MecanumSensor, queue_size=1)
clock_pub = rospy.Publisher('/clock', Clock, queue_size=1)

while robot.step(time_step) != -1 and not rospy.is_shutdown():
    clock = Clock()
    clock.clock = rospy.Time.from_sec(robot.getTime())
    clock_pub.publish(clock)
    ros_time_now = rospy.Time.from_sec(robot.getTime())

    pos1 = wheel1sensor.getValue()
    pos2 = wheel2sensor.getValue()
    pos3 = wheel3sensor.getValue()
    pos4 = wheel4sensor.getValue()

    sensor = MecanumSensor()
    sensor.header.stamp = ros_time_now
    sensor.vel1 = (pos1 - prev_pos1) * 1000.0 / time_step
    sensor.vel2 = (pos2 - prev_pos2) * 1000.0 / time_step
    sensor.vel3 = (pos3 - prev_pos3) * 1000.0 / time_step
    sensor.vel4 = (pos4 - prev_pos4) * 1000.0 / time_step

    [qx, qy, qz, qw] = quaternion_multiply(qENU2NUE, inertial_unit.getQuaternion())
    sensor.orientation.x = qx
    sensor.orientation.y = qy
    sensor.orientation.z = qz
    sensor.orientation.w = qw

    [wx, wy, wz] = gyro.getValues()
    sensor.angular_velocity.x = wx
    sensor.angular_velocity.y = wy
    sensor.angular_velocity.z = wz
    pub.publish(sensor)

    prev_pos1 = pos1
    prev_pos2 = pos2
    prev_pos3 = pos3
    prev_pos4 = pos4

    cmd_vx = cmd_vel.linear.x
    cmd_vy = cmd_vel.linear.y
    cmd_wz = cmd_vel.angular.z * pi / 180

    wheel1.setVelocity(((cmd_vx + cmd_vy) + cmd_wz * (a+b)) / r)
    wheel2.setVelocity(((cmd_vx - cmd_vy) - cmd_wz * (a+b)) / r)
    wheel3.setVelocity(((cmd_vx + cmd_vy) - cmd_wz * (a+b)) / r)
    wheel4.setVelocity(((cmd_vx - cmd_vy) - cmd_wz * (a+b)) / r)