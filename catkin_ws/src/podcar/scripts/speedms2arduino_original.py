#!/usr/bin/env python
import sys
import rospy
import pdb
from std_msgs.msg import Float64
import subprocess, time
import serial

port = '/dev/ttyArduino'
#port = '/dev/ttyACM2'
print("only works with dead mans handle\n")
print("Starting the program\n")
ser = serial.Serial(port, 115200, timeout=0.1)
print("Opened the communication with a port\n")
#protocol should return two lines of hello
line = ser.readline()   #we actually want a blocking read here to check the arduino is saying hello
print(line)

def rescale(x, oldmin, oldmax, newmin, newmax):
	r = (x-oldmin)/(oldmax-oldmin)
	out = nemin + r*(newmax-newmin)
	return out

class Node:
	def __init__(self):
		now = rospy.get_rostime()
		self.received_reponse_to_last_command = True
		self.sub = rospy.Subscriber("speedcmd_meterssec",Float64, self.callback_cmd, queue_size=1)
		self.buffer = []
		for i in range(0,100):
			self.buffer.append('X')
		self.buffer_idx_next = 0
		rospy.loginfo("initialised speed node")

	def callback_cmd(self,msg):  
		velocity_ms = msg.data

		#HACK probably best to convert direct from ms to arduino bytes later on - need to actually measure speeds per byte though	
		#convert to -1:1 range  (careful, these values are also used in joystick2speedms)
		if velocity_ms>0:
			velocity = velocity_ms/3.0
		else:
			velocity = velocity_ms/1.0  #reverse is slower


		#speedbytes are converted linearly to voltage as 
		# V = (4096/V_usb)*speedbyte   : range 0:5V
		#where V_usb is the voltage supplied to arduino power. in theory 5V but often 4.97V etc.
		# voltage V is then sent to the vehicle controller, which has 0V=fast reverse, 5V fast fwd
		# there is also a dead zone around 1.92V-2.71V used for STOP and needed at ignition-on. 
		# dead zone appears as 133-200 speedbytes for toughbook (different for other USB supplies)

		byte_dead_zone_top = 200
		byte_max_speed = 255   #max accepted by arduino is 255 but can limit for safety here
		byte_dead_zone_base = 132
		byte_max_reverse = 103   #max reverse would be 0 but we cap for safety here
		byte_stop =  180  #164

		#only write to serial if we are not still waiting for a confirm read
		if self.received_reponse_to_last_command:
			if abs(velocity)<0.01:
				speed_byte=byte_stop  #dead range center, good to startup
			elif velocity>=0:    #240 is racing fast ; use less for safety for now eg 210
				speed_byte = int(byte_dead_zone_top+velocity*(byte_max_speed-byte_dead_zone_top))  #from top of dead range tocapped max
			else:
				speed_byte = int(byte_dead_zone_base+velocity*(byte_dead_zone_base-byte_max_reverse))  #from bottom of dead range to cap max reverse
			lineout = "FA:%i\r\n"%speed_byte   #140 is stop.  240 is fast fwd.  80 for fast reverse.
			cmd = list(lineout)
			rospy.loginfo(cmd)
			for char in cmd:
				ser.write(char.encode())
			self.received_reponse_to_last_command = False

		#noblocking read
		chars=''
		nwaiting = ser.inWaiting()
		print('start read')
		chars = ser.read(nwaiting)  #read n bytes
		print('done read')
		if len(chars) != nwaiting:
			print('read length diff from req n chars')

		for i in range(0, len(chars)):
			char=chars[i]
			self.buffer[self.buffer_idx_next] = char
			self.buffer_idx_next += 1

			if char=="\n":
				#finished a line
				linein = self.buffer[0:self.buffer_idx_next]
				rospy.loginfo(linein)
				self.buffer = []
				for i in range(0,100):
					self.buffer.append('X')
				self.buffer_idx_next=0
				self.received_reponse_to_last_command = True   #enables us to send a new command

if __name__ == '__main__':
	rospy.init_node('joystick2speed',anonymous=True)
	ic = Node()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

