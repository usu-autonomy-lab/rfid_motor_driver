#!/usr/bin/python2

import serial
import rospy
import glob
import gui
import subprocess
import numpy
from time import sleep
from motor_serial.py import check_ports

baudrate = 9600

if __name__ == '__main__':
	params = gui.gui()
	baudrate = rospy.Rate(params[9])
	ports = check_ports()
	ser = serial.Serial(port, baudrate, timeout=1)
	print("Connected to: " + ser.portstr)
	sleep(5)
	print("Begin serial commands tests... ")

	ser.write("PA=1000;""BG;") # move slider forward
	sleep(5)

