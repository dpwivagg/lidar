#!/usr/bin/env python

"""
To run this node:
First, open serial.ino and check the serial port (Should be ACM0 or something similar)
Upload the serial.ino file to Arduino and run it
Make sure the roscore is running and set up whatever simulation arm is being used
Run the following commands in separate terminals:

    rosrun rosserial_python serial_node.py /dev/ttyACM0 # Replace this with the actual serial port
    rosrun RBE500 dataSmoother.py

"""

import rospy
from custom_sensor_msgs.msg import ImuMin
from geometry_msgs.msg import Twist
# TODO: Add an import for the correct message type or else we cannot use it


class Robot:
    def __init__(self):

        #  This constructor sets up class variables and pubs/subs

        #  This is the publisher. TODO: Change turtle1/cmd_vel to the topic.
        #  TODO: Change Twist to the type of message needed to control the arm
        self._vel_pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)

        # This subscribes to the Arduino/IMU. Do not change.
        rospy.Subscriber('/arduino/imu_data', ImuMin, self.sensorCallback, queue_size=1)

    def sensorCallback(self, data):
        angular_x = data.angular.x
        angular_y = data.angular.y
        angular_z = data.angular.z
        linear_x = data.linear.x
        linear_y = data.linear.y
        linear_z = data.linear.z

        # TODO: The data needs to be filtered somehow (KF?)

        # TODO: Change Twist() to the correct type of message and change the variables below as needed
        msg = Twist()

        msg.linear.x = linear_z / 5000
        msg.linear.y = linear_x / 5000
        msg.angular.z = angular_x / 5000

        # This line does not need to change
        self._vel_pub.publish(msg)


if __name__ == '__main__':
    rospy.sleep(1)
    rospy.init_node('sensor_read')
    turtle = Robot()

    while not rospy.is_shutdown():
        pass
