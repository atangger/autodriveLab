#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2, cv_bridge,numpy

class Follower:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("window", 1)
		self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image, self.image_callback)
		self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',Twist, queue_size=1)
		self.twist = Twist()
		self.state = 0 # 0 for following  1 for waiting 2 for turning 3 for stop
		self.turnCnt = 0
		self.waitCnt = 0
		self.turnTime = 60
		self.waitTime = 40
		self.stopTime = 70

	def metSigal(self,image):
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		lower_green = numpy.array([40, 20, 20])
		upper_green = numpy.array([70, 255,250])

		mask = cv2.inRange(hsv, lower_green, upper_green)
		h, w, d = image.shape
		search_top = 3*h/4
		search_bot = 3*h/4 + 50
		mask[0:search_top, 0:w] = 0
		mask[search_bot:h, 0:w] = 0
		M = cv2.moments(mask)


		# cv2.imshow("Green", mask)

		# cv2.waitKey(3)

		if M['m00'] > 0:
			return 1

		lower_blue = numpy.array([110, 20, 20])
		upper_blue = numpy.array([130, 255,250])
		mask = cv2.inRange(hsv, lower_blue, upper_blue)
		h, w, d = image.shape
		search_top = 3*h/4
		search_bot = 3*h/4 + 50
		mask[0:search_top, 0:w] = 0
		mask[search_bot:h, 0:w] = 0
		M = cv2.moments(mask)

		# cv2.imshow("Blue", mask)

		if M['m00'] > 0:
			return 2

		lower_red = numpy.array([0, 20, 20])
		upper_red = numpy.array([10, 255,250])
		mask = cv2.inRange(hsv, lower_red, upper_red)
		h, w, d = image.shape
		search_top = 3*h/4
		search_bot = 3*h/4 + 50
		mask[0:search_top, 0:w] = 0
		mask[search_bot:h, 0:w] = 0
		M = cv2.moments(mask)

		cv2.imshow("Red", mask)
		cv2.waitKey(3)

		if M['m00'] > 0:
			return 3

		return 0

	def changeSate(self,signal):
		if signal == 1 or signal == 2:
			if self.state == 0:
				self.waitCnt = self.waitTime
				self.state = 1
			elif self.state == 1:
				if self.waitCnt > 0:
					self.waitCnt -=1
				else:
					self.turnCnt = self.turnTime
					self.state = 2
			elif self.state == 2:
				if self.turnCnt > 0:
					self.turnCnt -=1
				else:
					self.state = 0
			else:
				print("undefined state")
		elif signal == 0:
			if self.state == 0:
				pass
			elif self.state == 1:
				if self.waitCnt > 0:
					self.waitCnt -=1
				else:
					self.turnCnt = self.turnTime
					self.state = 2
			elif self.state == 2:
				if self.turnCnt > 0:
					self.turnCnt -=1
				else:
					self.state = 0
			else:
				pass
		elif signal == 3:
			if self.stopTime >0:
				self.stopTime -=1
			else:
				self.state = 3

		else:
			pass


	def image_callback(self, msg):
		image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		lower_yellow = numpy.array([ 20, 20, 170])
		upper_yellow = numpy.array([255, 255, 190])
		mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
		h, w, d = image.shape
		search_top = 3*h/4
		search_bot = 3*h/4 + 50
		mask[0:search_top, 0:w] = 0
		mask[search_bot:h, 0:w] = 0
		
		signal = self.metSigal(image)
		self.changeSate(signal)

		if self.state == 0 or self.state == 1:
			if self.state == 1:
				print("start to wait")
			M = cv2.moments(mask)
			if M['m00'] > 0:
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
				cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
				err = cx - w/2
				self.twist.linear.x = 0.2
				self.twist.angular.z = -float(err) / 100
				self.cmd_vel_pub.publish(self.twist)
		elif self.state == 2:
			print("start to turn")
			print("turn cnt = %d"%(self.turnCnt))
			self.twist.linear.x = 0.2
			if signal == 1:
				self.twist.angular.z = 0.1
			if signal == 2:
				self.twist.angular.z = -0.1
			self.cmd_vel_pub.publish(self.twist)
		else:
			self.twist.linear.x = 0
			self.twist.angular.z = 0
			self.cmd_vel_pub.publish(self.twist)

		cv2.imshow("windowroad", mask)
		cv2.imshow("origin", image)
		cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()