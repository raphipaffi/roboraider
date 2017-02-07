#!/usr/bin/env python

"""
    A Python driver for the Arduino microcontroller running the
    ROSArduinoBridge firmware.
    
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

import roslib
import rospy
import thread
from math import pi as PI, degrees, radians
import os
import time
import sys, traceback
from serial.serialutil import SerialException
from serial import Serial


class Arduino:

    def __init__(self, port="/dev/ttyACM0", baudrate=115200, timeout=0.5):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.writeTimeout = timeout
        self.interCharTimeout = timeout / 30.
        # Keep things thread safe
        self.mutex = thread.allocate_lock()
    
    def connect(self):
        try:
            print "Connecting to Arduino on port", self.port, "..."
            self.port = Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout, writeTimeout=self.writeTimeout)
            # The next line is necessary to give the firmware time to wake up.
            time.sleep(1)
            test = self.get_baud()
            if test != self.baudrate:
                time.sleep(1)
                test = self.get_baud()   
                if test != self.baudrate:
                    raise SerialException
            print "Connected at ", self.baudrate
            print "Arduino is ready."

        except SerialException:
            print "Serial Exception:"
            print sys.exc_info()
            print "Traceback follows:"
            traceback.print_exc(file=sys.stdout)
            print "Cannot connect to Arduino!"
            os._exit(1)

    def open(self): 
        self.port.open()

    def close(self): 
        self.port.close() 
    
    def send(self, cmd):
        self.port.write(cmd + '\r')

    def recv_raw(self, timeout):
        c = ''
        value = ''
        attempts = 0
        while c != '\r':
            c = self.port.read(1)
            value += c
            attempts += 1
            if attempts * self.interCharTimeout > timeout:
                return None

        value = value.strip('\r')
        return value
    
    def recv_raw_array(self, timeout):
        try:
            values = self.recv_raw(timeout).split()
            return values
        except:
            return []

    def execute(self, cmd, func):
        self.mutex.acquire()
        
        try:
            self.port.flushInput()
        except:
            pass
        
        ntries = 1
        attempts = 0
        
        try:
            self.port.write(cmd + '\r')
            value = self.recv_raw(self.timeout)
            while attempts < ntries and (value == '' or value == 'Invalid Command' or value == None):
                try:
                    self.port.flushInput()
                    self.port.write(cmd + '\r')
                    value = self.recv_raw(self.timeout)
                except:
                    print "Exception executing command: " + cmd
                attempts += 1
        except:
            self.mutex.release()
            print "Exception executing command: " + cmd
            value = None
        
        self.mutex.release()
        return func(value)

    def execute_array(self, cmd, func):
        self.mutex.acquire()
        
        try:
            self.port.flushInput()
        except:
            pass
        
        ntries = 1
        attempts = 0
        
        try:
            self.port.write(cmd + '\r')
            values = self.recv_raw_array(10*self.timeout)
            while attempts < ntries and (values == '' or values == 'Invalid Command' or values == [] or values == None):
                try:
                    self.port.flushInput()
                    self.port.write(cmd + '\r')
                    values = self.recv_raw_array(10*self.timeout)
                except:
                    print("Exception executing command: " + cmd)
                attempts += 1
        except:
            self.mutex.release()
            print "Exception executing command: " + cmd
            raise SerialException
            return []
        
        try:
            values = map(func, values)
        except:
            values = []

        self.mutex.release()
        return values
        
    def execute_ack(self, cmd):
        self.mutex.acquire()
        
        try:
            self.port.flushInput()
        except:
            pass
        
        ntries = 1
        attempts = 0
        
        try:
            self.port.write(cmd + '\r')
            ack = self.recv_raw(self.timeout)
            while attempts < ntries and (ack == '' or ack == 'Invalid Command' or ack == None):
                try:
                    self.port.flushInput()
                    self.port.write(cmd + '\r')
                    ack = self.recv_raw(self.timeout)
                except:
                    print "Exception executing command: " + cmd
            attempts += 1
        except:
            self.mutex.release()
            print "execute_ack exception when executing", cmd
            print sys.exc_info()
            return 0
        
        self.mutex.release()
        return ack == 'OK'

   
    def get_baud(self):
        return self.execute('b', int);

    def get_cycletime(self):
        return self.execute('t', float);

    def get_encoder_counts(self):
        values = self.execute_array('e', int)
        if len(values) != 2:
            print "Encoder count was not 2"
            raise SerialException
            return None
        else:
            return values

    def get_pose(self):
        values = self.execute_array('p', float)
        return values
        if len(values) != 5:
            print "Pose vector has not 5 elements"
            raise SerialException
            return None
        else:
            return values

    def reset_odometry(self):
        return self.execute_ack('r')

    def motor_command(self, v_left, v_right):
	# rospy.loginfo("motor_command: m %+.2f %+.2f" %(v_left, v_right))
        return self.execute_ack('m %+.2f %+.2f' %(v_left, v_right))

    def stop(self):
        self.motor_command(0, 0)




# Basic test for connectivity
if __name__ == "__main__":
    myArduino = Arduino(port="/dev/ttyACM0", baudrate=115200, timeout=0.5)
    myArduino.connect()
    myArduino.reset_odometry()
    time.sleep(3)

    # myArduino.motor_command(v_left=-0.05, v_right=0.05)

    dtheta_avg = 0.0;
    i = 0

    while 1:
	time.sleep(0.05)
	pose = myArduino.get_pose()
        #print "Current encoder counts", myArduino.get_encoder_counts()
        pose[2] = pose[2] * 57.3
        pose[4] = pose[4] * 57.3
        print "Current pose", pose
        #print "Controller cycle time", myArduino.get_cycletime()
	"""
	print "move forward"      
	myArduino.motor_command(v_left=0.03, v_right=0.03)
	time.sleep(1)
	print "move backward"      
	myArduino.motor_command(v_left=-0.03, v_right=-0.03)
	time.sleep(1)
	print "turn left"      
	myArduino.motor_command(v_left=-0.03, v_right=0.03)
	time.sleep(1)
	print "turn right"      
	myArduino.motor_command(v_left=0.03, v_right=-0.03)
	"""
	"""
	dtheta_avg += pose[4]
	i += 1
	print "dtheta:", dtheta_avg/i, "rad/s"
	"""
	
    
    print "Connection test successful."
    
    myArduino.stop()
    myArduino.close()
    
    print "Shutting down Arduino."
    