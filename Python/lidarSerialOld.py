# -*- coding: utf-8 -*-
"""
Created on Thu Mar 02 18:32:36 2017

@author: fengqiang
Copyright (c) Benewake LLC.  All rights reserved.
"""

import serial
import RPi.GPIO as GPIO
import PCA9685 as pwm
import time    # Import necessary modules

MinPulse = 200
MaxPulse = 700

def servo_setup():
	global pwm
	pwm = pwm.PWM()

def servo_test():
	for value in range(MinPulse, MaxPulse):
		pwm.write(0, 0, value)
		pwm.write(14, 0, value)
		pwm.write(15, 0, value)
		time.sleep(0.002)




# ===========================================================================
# Raspberry Pi pin11, 12, 13 and 15 to realize the clockwise/counterclockwise
# rotation and forward and backward movements
# ===========================================================================
Motor0_A = 11  # pin11
Motor0_B = 12  # pin12
Motor1_A = 13  # pin13
Motor1_B = 15  # pin15

# ===========================================================================
# Set channel 4 and 5 of the servo driver IC to generate PWM, thus
# controlling the speed of the car
# ===========================================================================
EN_M0    = 4  # servo driver IC CH4
EN_M1    = 5  # servo driver IC CH5

pins = [Motor0_A, Motor0_B, Motor1_A, Motor1_B]

# ===========================================================================
# Adjust the duty cycle of the square waves output from channel 4 and 5 of
# the servo driver IC, so as to control the speed of the car.
# ===========================================================================
def setSpeed(speed):
	speed *= 40
	print 'speed is: ', speed
	pwm.write(EN_M0, 0, speed)
	pwm.write(EN_M1, 0, speed)

def motor_setup(busnum=None):
	global forward0, forward1, backward1, backward0
	global pwm
	if busnum == None:
		pwm = pwm.PWM()                  # Initialize the servo controller.
	else:
		pwm = pwm.PWM(bus_number=busnum) # Initialize the servo controller.

	pwm.frequency = 60
	forward0 = 'True'
	forward1 = 'True'
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BOARD)        # Number GPIOs by its physical location
	try:
		for line in open("config"):
			if line[0:8] == "forward0":
				forward0 = line[11:-1]
			if line[0:8] == "forward1":
				forward1 = line[11:-1]
	except:
		pass
	if forward0 == 'True':
		backward0 = 'False'
	elif forward0 == 'False':
		backward0 = 'True'
	if forward1 == 'True':
		backward1 = 'False'
	elif forward1 == 'False':
		backward1 = 'True'
	for pin in pins:
		GPIO.setup(pin, GPIO.OUT)   # Set all pins' mode as output

# ===========================================================================
# Control the DC motor to make it rotate clockwise, so the car will
# move forward.
# ===========================================================================

def motor0(x):
	if x == 'True':
		GPIO.output(Motor0_A, GPIO.LOW)
		GPIO.output(Motor0_B, GPIO.HIGH)
	elif x == 'False':
		GPIO.output(Motor0_A, GPIO.HIGH)
		GPIO.output(Motor0_B, GPIO.LOW)
	else:
		print 'Config Error'

def motor1(x):
	if x == 'True':
		GPIO.output(Motor1_A, GPIO.LOW)
		GPIO.output(Motor1_B, GPIO.HIGH)
	elif x == 'False':
		GPIO.output(Motor1_A, GPIO.HIGH)
		GPIO.output(Motor1_B, GPIO.LOW)

def forward():
	motor0(forward0)
	motor1(forward1)

def backward():
	motor0(backward0)
	motor1(backward1)

def forwardWithSpeed(spd = 50):
	setSpeed(spd)
	motor0(forward0)
	motor1(forward1)

def backwardWithSpeed(spd = 50):
	setSpeed(spd)
	motor0(backward0)
	motor1(backward1)

def stop():
	for pin in pins:
		GPIO.output(pin, GPIO.LOW)

# ===========================================================================
# The first parameter(status) is to control the state of the car, to make it
# stop or run. The parameter(direction) is to control the car's direction
# (move forward or backward).
# ===========================================================================
def ctrl(status, direction=1):
	if status == 1:   # Run
		if direction == 1:     # Forward
			forward()
		elif direction == -1:  # Backward
			backward()
		else:
			print 'Argument error! direction must be 1 or -1.'
	elif status == 0: # Stop
		stop()
	else:
		print 'Argument error! status must be 0 or 1.'

def test():
	while True:
		setup()
		ctrl(1)
		time.sleep(3)
		setSpeed(10)
		time.sleep(3)
		setSpeed(100)
		time.sleep(3)
		ctrl(0)


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
			# 检查帧头
			if ord(data[0]) != 0xaa or ord(data[1]) != 0x55:
				print data
				continue
			else:
				data = self.readPort(2)
				# 检查长度
				if (ord(data[0]) != 0x01 and ord(data[0]) != 0x03) :
					continue
				else:
					nPoints = ord(data[0])
					break
		# 读取一个点的距离和角度数据
		data = self.readPort(3 * nPoints)
		Z = ord(data[0]) | (ord(data[1]) << 8)
		theta = ord(data[2])
		if theta & 0x80:
			theta = -1 * (((~theta) + 1) & 0xff)
		print "min =",Z,theta
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
		print "\n"
		return (Z, theta)

	def lidarReadLine(self):
		Z = []
		theta = []
		while True:
			data = self.readPort(2)
			# 检查帧头
			if ord(data[0]) != 0xaa or ord(data[1]) != 0x55:
				continue
			else:
				data = self.readPort(2)
				# 检查长度
				if ord(data[0]) != 0x40 or ord(data[1]) != 0x01:
					continue
				else:
					break
		# 读取320个点的距离和角度数据
		data = self.readPort(3 * 320)
		for i in range(320):
			Z.append(ord(data[3 * i + 0]) | (ord(data[3 * i + 1]) << 8))
			theta_tmp = ord(data[2])
			if theta_tmp & 0x80:
				theta_tmp = -1 * (((~theta_tmp) + 1) & 0xff)
			theta.append(theta_tmp)
		return (Z, theta)


if __name__ == "__main__":

	port = serialPort()
	port.openPort("/dev/ttyUSB1")
	port.lidarStart(320, 320, 2)
	motor_setup()
	setSpeed(50)
	servo_setup()
	servo_test()

	while True:
		print "read data"
		print port.lidarRead()
#    port = serial.Serial(port = 'COM10',
#                         baudrate = 115200,
#                         bytesize = serial.EIGHTBITS,
#                         parity = serial.PARITY_NONE,
#                         stopbits = serial.STOPBITS_ONE)
#    port.open()
#    print port.isOpen()
#
#    time.sleep(1)
#    port.write([0xaa, 0x55, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02])
#    port.flush()
#    print ord(port.read(1))
#    port.close()
#    while True:
#        print "wait"
	stop()
