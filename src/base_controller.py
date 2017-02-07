#!/usr/bin/env python

"""
    A base controller class for the Arduino microcontroller
    
    Borrowed heavily from Mike Feguson's ArbotiX base_controller.py code.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses
"""
import roslib
import rospy
import os
import arduino_driver

from math import sin, cos, pi, degrees
from geometry_msgs.msg import Quaternion, Twist, Pose
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

 
""" Class to receive Twist commands and publish Odometry data """
class BaseController:
    def __init__(self, arduino, base_frame):
        self.arduino		= arduino
        self.base_frame 	= base_frame
        self.cmd_vel_timeout 	= float(rospy.get_param("~cmd_vel_timeout", 1.0))
	self.acc_lim 		= float(rospy.get_param('~acc_lim', 0.5))
	self.acc_lim_diff	= float(rospy.get_param('~acc_lim_diff', 0.5))

	self.stopped = False

        self.x = 0              # x coordinate of robot position [m]
        self.y = 0              # y coordinate of robot position [m]
        self.theta = 0          # orientation of robot [rad]
        self.v = 0              # linear velocity of robot (m/s)
        self.dtheta = 0         # angular velocity of robot (rad/s)

        self.v_des = 0          # desired linear velocity based on cmd_vel (m/s)
        self.dtheta_des = 0     # desired angular velocity based on cmd_vel (rad/s)

        self.v_cmd = 0      	# common part of commanded velocity for left and right wheel (m/s)
        self.vdelta_cmd = 0     # difference of commanded velocities (vright-vleft) (m/s)

	now = rospy.Time.now()
        self.last_cmd_vel = now	# time of last cmd_vel callback
	self.last_poll = now	# time of last cmd_vel callback
        
        # subscriptions
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)
        
        # Clear any old odometry info
        self.arduino.reset_odometry()
        
        # Set up the odometry broadcaster
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()
        
        rospy.loginfo("Started base controller")
        
    def poll(self):
	now = rospy.Time.now()	
	t_delta = now - self.last_poll

	v_step      = self.acc_lim 	* t_delta.to_sec()
	vdelta_step = self.acc_lim_diff * t_delta.to_sec()
        
        # Read the odometry
	pose = self.arduino.get_pose()
        self.x      = pose[0]
        self.y      = pose[1]
        self.theta  = pose[2]
        self.v      = pose[3]
        self.dtheta = pose[4]
	
        quaternion = Quaternion()
        quaternion.x = 0.0 
        quaternion.y = 0.0
        quaternion.z = sin(self.theta / 2.0)
        quaternion.w = cos(self.theta / 2.0)
    
        # Create the odometry transform frame broadcaster.
        self.odomBroadcaster.sendTransform(
            (self.x, self.y, 0), 
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            now,
            self.base_frame,
            "odom")
    
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = self.base_frame
        odom.header.stamp = now
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = self.dtheta

        self.odomPub.publish(odom)
            
        # if cmd_vel subsription timed out
        if now > (self.last_cmd_vel + rospy.Duration(self.cmd_vel_timeout)):
            self.v_des = 0
            self.dtheta_des = 0
            
	# calc commanded velocities
	if self.v_des > self.v_cmd:
		self.v_cmd += v_step
		if self.v_des < self.v_cmd:
		    self.v_cmd = self.v_des
	else:
		if self.v_des < self.v_cmd:
		    self.v_cmd -= v_step
		    if self.v_des > self.v_cmd:
			self.v_cmd = self.v_des

	if 0.454 * self.dtheta_des > self.vdelta_cmd:
		self.vdelta_cmd += vdelta_step
		if 0.454 * self.dtheta_des < self.vdelta_cmd:
			self.vdelta_cmd = 0.454 * self.dtheta_des
	else:
		if 0.454 * self.dtheta_des < self.vdelta_cmd:
			self.vdelta_cmd -= vdelta_step
			if 0.454 * self.dtheta_des > self.vdelta_cmd:
				self.vdelta_cmd = 0.454 * self.dtheta_des

        vleft_cmd  = self.v_cmd - self.vdelta_cmd/2.0
        vright_cmd = self.v_cmd + self.vdelta_cmd/2.0

	#rospy.loginfo("dtheta_des: " + str(self.dtheta_des) + " rad/s\tdtheta " + str(self.dtheta) + " rad/s\tvdelta_cmd " + str(self.vdelta_cmd) + " m/s")
	rospy.loginfo("vleft_cmd: " + str(vleft_cmd) + " m/s\tvright_cmd " + str(vright_cmd) + " m/s")
	
        # Set motor speeds
        if not self.stopped:
		self.arduino.motor_command(vleft_cmd, vright_cmd)
        
	self.last_poll = now;
            
    def stop(self):
        self.stopped = True
        self.arduino.motor_command(0, 0)
            
    def cmdVelCallback(self, req):
        # store velocity-based movement requests
        self.last_cmd_vel = rospy.Time.now()
        self.v_des = req.linear.x   # m/s
        self.dtheta_des = req.angular.z  # rad/s
