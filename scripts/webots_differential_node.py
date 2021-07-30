#!/usr/bin/env python2

import rospy
from tf.transformations import quaternion_multiply
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from vehicle_simulator.msg import DifferentialSensor
from controller import Robot
from math import sin, cos, pi
import numpy as np

cmd_vel = Twist()

def command_callback(msg):
    global cmd_vel
    cmd_vel = msg

robot = Robot()
time_step = int(robot.getBasicTimeStep())

# Motor
wheel1 = robot.getDevice('wheel1')
wheel2 = robot.getDevice('wheel2')
wheel1.setPosition(float('inf'))
wheel2.setPosition(float('inf'))
wheel1.setVelocity(0)
wheel2.setVelocity(0)

# Position Sensor
wheel1sensor = robot.getDevice('wheel1sensor')
wheel2sensor = robot.getDevice('wheel2sensor')
wheel1sensor.enable(time_step)
wheel2sensor.enable(time_step)

# IMU
inertial_unit = robot.getDevice('inertial unit')
inertial_unit.enable(time_step)

# Gyro
gyro = robot.getDevice('gyro')
gyro.enable(time_step)

# Lidar
lidar = robot.getDevice('lidar')
lidar.enable(time_step)

# Range Finder
range_finder = robot.getDevice('range-finder')
range_finder.enable(time_step)

# Camera
camera = robot.getDevice('camera')
camera.enable(time_step)

robot.step(time_step)
prev_pos1 = wheel1sensor.getValue()
prev_pos2 = wheel2sensor.getValue()
qENU2NUE = [cos(pi / 4), 0, 0, sin(pi / 4)]
l = 0.1
r = 0.031

rospy.init_node('webots_differential', anonymous=True)
rospy.Subscriber('/cmd_vel', Twist, command_callback)
clock_pub = rospy.Publisher('/clock', Clock, queue_size=1)
sensor_pub = rospy.Publisher('differential_sensor', DifferentialSensor, queue_size=1)
scan_pub = rospy.Publisher('scan', LaserScan, queue_size=1)
depth_pub = rospy.Publisher('depth_image', Image, queue_size=1)
rgb_pub = rospy.Publisher('rgb_image', Image, queue_size=1)

while robot.step(time_step) != -1 and not rospy.is_shutdown():
    clock = Clock()
    clock.clock = rospy.Time.from_sec(robot.getTime())
    clock_pub.publish(clock)
    ros_time_now = rospy.Time.from_sec(robot.getTime())

    pos1 = wheel1sensor.getValue()
    pos2 = wheel2sensor.getValue()

    sensor = DifferentialSensor()
    sensor.header.stamp = ros_time_now
    sensor.vel1 = (pos1 - prev_pos1) * 1000.0 / time_step
    sensor.vel2 = (pos2 - prev_pos2) * 1000.0 / time_step
    [qx, qy, qz, qw] = quaternion_multiply(qENU2NUE, inertial_unit.getQuaternion())
    sensor.orientation.x = qx
    sensor.orientation.y = qy
    sensor.orientation.z = qz
    sensor.orientation.w = qw
    [wx, wy, wz] = gyro.getValues()
    sensor.angular_velocity.x = wx
    sensor.angular_velocity.y = wy
    sensor.angular_velocity.z = wz
    sensor_pub.publish(sensor)

    scan = LaserScan()
    scan.header.stamp = ros_time_now
    scan.header.frame_id = 'lidar'
    scan.angle_min = -lidar.getFov() / 2.0
    scan.angle_max = +lidar.getFov() / 2.0
    scan.range_min = lidar.getMinRange()
    scan.range_max = lidar.getMaxRange()
    scan.angle_increment = lidar.getFov() / lidar.getHorizontalResolution()
    scan.time_increment = 0
    scan.scan_time = 0
    scan_buffer = lidar.getRangeImage()
    scan_buffer.reverse()
    scan.ranges = scan_buffer
    scan_pub.publish(scan)

    depth = Image()
    depth.header.stamp = ros_time_now
    depth.header.frame_id = 'depth_camera'
    depth.width = range_finder.getWidth()
    depth.height = range_finder.getHeight()
    depth.encoding = '32FC1'
    depth.is_bigendian = False
    depth.step = 4 * range_finder.getWidth()
    depth_buffer = np.array(range_finder.getRangeImage(), dtype=np.float32)
    depth.data = depth_buffer.tobytes()
    depth_pub.publish(depth)

    rgb = Image()
    rgb.header.stamp = ros_time_now
    rgb.header.frame_id = 'depth_camera'
    rgb.width = camera.getWidth()
    rgb.height = camera.getHeight()
    rgb.encoding = 'bgra8'
    rgb.is_bigendian = False
    rgb.step = 4 * range_finder.getWidth()
    rgb.data = camera.getImage()
    rgb_pub.publish(rgb)

    cmd_vx = cmd_vel.linear.x
    cmd_wz = cmd_vel.angular.z * pi / 180

    wheel1.setVelocity((cmd_vx - cmd_wz * l) / r)
    wheel2.setVelocity((cmd_vx + cmd_wz * l) / r)

    prev_pos1 = pos1
    prev_pos2 = pos2