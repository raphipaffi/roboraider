#!/usr/bin/env python

"""
    A ROS Node for the Arduino microcontroller
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
from arduino_driver import Arduino
from base_controller import BaseController
from geometry_msgs.msg import Twist
import os, time
import thread

class ArduinoROS():
    def __init__(self):
        rospy.init_node('Arduino', log_level=rospy.DEBUG)
                
        # Cleanup when terminating the node
        rospy.on_shutdown(self.shutdown)
        
        # A cmd_vel publisher so we can stop the robot when shutting down
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # loop rate
        self.node_rate = int(rospy.get_param("~node_rate", 50))
        r = rospy.Rate(self.node_rate)
        
        # create instance of the Arduino driver
        self.port = rospy.get_param("~port", "/dev/ttyACM0")
        self.baud = int(rospy.get_param("~baud", 115200))
        self.com_timeout = rospy.get_param("~com_timeout", 0.5)
        
        rospy.loginfo("Requesting Arduino on port " + self.port + " at " + str(self.baud) + " baud with communication timeout " + str(self.com_timeout) + " s")
        self.controller = Arduino(self.port, self.baud, self.com_timeout)
        self.controller.connect()
        self.controller.reset_odometry()
        rospy.loginfo("Connected to Arduino on port " + self.port + " at " + str(self.baud) + " baud")
        
        # Initialize the base controller
        self.base_frame = rospy.get_param("~base_frame", 'base_link')
        self.myBaseController = BaseController(self.controller, self.base_frame)
        
        # Reserve a thread lock
        mutex = thread.allocate_lock()
    
        # Start polling the base controller
        while not rospy.is_shutdown():       
            mutex.acquire()
            self.myBaseController.poll()
            mutex.release()
            
            r.sleep()

    def shutdown(self):
        # Stop the robot
        try:
            rospy.loginfo("Stopping the robot...")
            self.cmd_vel_pub.Publish(Twist())
            rospy.sleep(2)
        except:
            pass
        rospy.loginfo("Shutting down Arduino Node...")
        
if __name__ == '__main__':
    myArduino = ArduinoROS()

