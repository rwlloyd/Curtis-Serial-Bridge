#!/usr/bin/env python
import sys
import rospy
import pdb
from std_msgs.msg import Float64
import subprocess, time
import serial

port = '/dev/ttyArduino'
print("only works with dead mans handle\n")
print("Starting the program\n")
ser = serial.Serial(port, 9600)
print("Opened the communication with a port\n")
#protocol should return two lines of hello
line = ser.readline()   #we actually want a blocking read here to check the arduino is saying hello
print(line)

def rescale(x, oldmin, oldmax, newmin, newmax):
	r = (x-oldmin)/(oldmax-oldmin)
	out = newmin + r*(newmax-newmin)
	return out

class Node:
	def __init__(self):
		now = rospy.get_rostime()
		self.mutex = False
		self.sub = rospy.Subscriber("speedcmd_meterssec",Float64, self.callback_cmd, queue_size=1)
		self.buffer = ["X"]*500   #reset buffer
		self.buffer_idx_next = 0
		rospy.loginfo("initialised speed node")

	def callback_cmd(self,msg):  
		velocity_ms = msg.data

		#TODO convert to actual meters per second command by measuring the vehicle speed on ground
		#convert to -1:1 range  (careful, these values are also used in joystick2speedms)
		if velocity_ms>0:
			velocity = velocity_ms/3.0
		else:
			velocity = velocity_ms/1.0  #reverse is slower

		print("Velocity: " + str(velocity))
		#speedbytes are converted linearly to voltage as 
		# V = (4096/V_usb)*speedbyte   : range 0:5V
		#where V_usb is the voltage supplied to arduino power. in theory 5V but often 4.97V etc.
		# voltage V is then sent to the vehicle controller, which has 0V=fast reverse, 5V fast fwd
		# there is also a dead zone around 1.92V-2.71V used for STOP and needed at ignition-on. 
		# dead zone appears as 133-200 speedbytes for toughbook (different for other USB supplies)
		byte_dead_zone_top = 220 # 200
		byte_max_speed = 255   #max accepted by arduino is 255 but can limit for safety here
		byte_dead_zone_base = 140 #132
		byte_max_reverse = 103 #103   #max reverse would be 0 but we cap for safety here
		byte_stop =  170#180  #164

		#only write to serial if we are not still waiting for a confirm read
		if not self.mutex:
			self.mutex=True   #get lock on device
			if abs(velocity)<0.01:
				speed_byte=byte_stop  #dead range center, good to startup
			elif velocity>=0.01:    #240 is racing fast ; use less for safety for now eg 210
				speed_byte = int(byte_dead_zone_top+velocity*(byte_max_speed-byte_dead_zone_top))  #from top of dead range tocapped max
			else:
				speed_byte = int(byte_dead_zone_base+velocity*(byte_dead_zone_base-byte_max_reverse))  #from bottom of dead range to cap max reverse
			lineout = "FA:%i\r\n"%speed_byte   #140 is stop.  240 is fast fwd.  80 for fast reverse.
			print("New command: " + lineout)
			cmd = list(lineout)
			rospy.loginfo(cmd)
			for char in cmd:
				ser.write(char.encode())

			#read
			chars=''
			b_lineDone = False
			while not b_lineDone:
				nwaiting = ser.inWaiting()
				print('start read %i bytes from OS serial'%nwaiting)
				chars = ser.read(nwaiting)  #read n bytes
				print("bytes read from OS serial: " + str(chars))
				if len(chars) != nwaiting:
					print('read length diff from req n chars')
				for i in range(0, len(chars)):  #copy the read bytes to our own buffer
					char=chars[i]
					self.buffer[self.buffer_idx_next] = char
					self.buffer_idx_next += 1
					if char=="\n":                #finished a line
						linein = self.buffer[0:self.buffer_idx_next]
						#if we do anything with the data do it here!
						self.buffer = ["X"]*500   #reset buffer
						self.buffer_idx_next=0
						b_lineDone=True
					else:
						b_lineDone=False
			self.mutex=False   #enables us to send a new command

if __name__ == '__main__':
	rospy.init_node('speedms2arduino',anonymous=True)
	ic = Node()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

