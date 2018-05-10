# -*- coding: utf-8 -*-
"""
Remix by Chris Anderson, DIY Robocars
Adapted to run on the Sunfounder Robocar: https://www.sunfounder.com/robotic-drone/smartcar/picar-v.html
Assembly instructions here: https://diyrobocars.com/2018/05/07/demo-of-benwake-ce30-a-on-a-diy-robocar/
Sample code by fengqiang
Copyright (c) Benewake LLC.  All rights reserved.
"""

import serial   # you must have installed the pyserial libary already: "pip install pyserial"
import time
import motor
import car_dir as steer
import RPi.GPIO as GPIO

busnum = 1          # Edit busnum to 0 for Raspberry Pi 1. For RPi 2 and above, use 1
forward0 = 'True'
forward1 = 'False'
speed = 30
deadband = 1
gain = 30000 # steering servo gain
offset = 60  # how much the servo has to be turned to have the car go straight
probability = 0.8 # this is the baysian "prior", the likelyhood of preview measurement being the same as current one
old_data = 0

class serialPort:
    def __init__(self):
        self.baudrate = 115200
        self.portName = "/dev/ttyUSB1"
        self.port = None

    def setBardRate(self, baudRate):
        self.baudrate = baudRate

    def openPort(self, portName):
        if self.portName != portName and self.port != None:
            self.port.close()
            self.port = None
        self.portName = portName
        if self.port == None:
            print self.portName, self.baudrate
            self.port = serial.Serial(port = self.portName, baudrate = self.baudrate, bytesize = serial.EIGHTBITS, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, timeout = 1)

    def closePort(self):
        self.port.close()
        self.port = None

    def readPort(self, length):
        return self.port.read(length)

    def writePort(self, data):
        self.port.write(data)

    def clearPort(self):
        try:
            self.port.flush()
            self.port.read(100)
        except:
            return

    def lidarStart(self, width, depth, fps, protocol = 0):
        data = [0xaa, 0x55, 0x01 | (protocol << 5), width & 0xff, (width >> 8) & 0xff, depth & 0xff, (depth >> 8) & 0xff, fps]
        for i in range(20):
            self.writePort(data)
            ret = self.readPort(1)
            if ord(ret) == 0x90:
                ret = 0
                break
            elif ord(ret) == 0x91:
                ret = -1
            else:
                ret = -2
        return ret

    def lidarStop(self):
        data = [0xaa, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        while True:
            self.clearPort()
            self.writePort(data)
            ret = self.readPort(1)
            if ord(ret) == 0x90:
                break
            time.sleep(0.1)

    def lidarRead(self):
        while True:
            data = self.readPort(2)
            if ord(data[0]) != 0xaa or ord(data[1]) != 0x55:
                print data
                continue
            else:
                data = self.readPort(2)
                if (ord(data[0]) != 0x01 and ord(data[0]) != 0x03) :
                    continue
                else:
                    nPoints = ord(data[0])
                    break
        # Reading a point's distance and angle data  
        data = self.readPort(3 * nPoints)
        Z = ord(data[0]) | (ord(data[1]) << 8)
        theta = ord(data[2])
        if theta & 0x80:
            theta = -1 * (((~theta) + 1) & 0xff)
#        print "min =",Z,theta
        if nPoints == 3:
            Z1 = ord(data[3]) | (ord(data[4]) << 8)
            theta1 = ord(data[5])
            if theta1 & 0x80:
                theta1 = -1 * (((~theta1) + 1) & 0xff)
            Z2= ord(data[6]) | (ord(data[7]) << 8)
            theta2 = ord(data[8])
            if theta2 & 0x80:
                theta2 = -1 * (((~theta2) + 1) & 0xff)
            print "left =",Z1,theta1
            print "right =",Z2,theta2
#        print "\n"
        return (Z, theta)

    def lidarReadLine(self):
        Z = []
        theta = []
        while True:
            data = self.readPort(2)
            if ord(data[0]) != 0xaa or ord(data[1]) != 0x55:
                continue
            else:
                data = self.readPort(2)
                if ord(data[0]) != 0x40 or ord(data[1]) != 0x01:
                    continue
                else:
                    break
        # Reading 320 points of distance and angle data  
        data = self.readPort(3 * 320)
        for i in range(320):
            Z.append(ord(data[3 * i + 0]) | (ord(data[3 * i + 1]) << 8))
            theta_tmp = ord(data[2])
            if theta_tmp & 0x80:
                theta_tmp = -1 * (((~theta_tmp) + 1) & 0xff)
            theta.append(theta_tmp)
        return (Z, theta)


def kalman(new_data):  # this just smooths data by blending with prior measurement
	global old_data
	temp = new_data - ((new_data - old_data) * probability)
	old_data = temp
	return (temp)

def shutdown():
	global run
	print ("Stopping")
	motor.setSpeed(0)
	port.closePort()
	run = False
	

if __name__ == "__main__":

    run = True
    port = serialPort()
    motor.setup(busnum=busnum)
    steer.setup(busnum=busnum)
    print 'motor moving forward'
    motor.setSpeed(speed)
    motor.motor0(forward0)
    motor.motor1(forward1)
#    motor.setSpeed(0)
#    motor.motor0(forward0)
#    motor.motor1(forward1)
    steer.turn(0)
    port.openPort("/dev/ttyUSB1")
#    open lidar with width, depth in cm, refresh rate in hz
    port.lidarStart(30, 200, 10)
    while run:
	try:
		data = port.lidarRead()
		print ("distance: ", data[0])
		print ("angle: ", data[1])
		distance = data[0]
		if distance != 0:
			scale = gain/distance  # the further away something is, the less we have to turn
		else:
			scale = gain
		angle = data[1]
		angle = kalman(angle) # use a single-state kalman filter (basically a moving average) to reduce noise in measurements
		if (distance < 30) and (distance != 0):	# something is right in front of us!
			if old_data > 0:   # turn 30 in the direction of the last observed obstacle
				angle = 30
			else:
				angle = -30
		if (angle <= -deadband) or (angle >= deadband):  # this is the regular situation
			turn = int(-scale * (1.000/angle))  # use an inverse function; the further to the side the obstacle, the less you have to turn
		else:   # we're in the deadband, so just go straight
			turn= 0
		print("turn: ", turn)
		steer.turn(turn - offset)
	except KeyboardInterrupt:
		shutdown()

