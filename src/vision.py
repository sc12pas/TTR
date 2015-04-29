import numpy as np
import rospy
import cv2
import cv_bridge
import time
import sys
import baxter_interface as bax
from matplotlib import pyplot as plt

from sensor_msgs.msg import Image

rospy.init_node('Game_run')

class Point:
	def __init__(self):
		self.x = 0
		self.y = 0

limbR = bax.Limb('right')
limbL = bax.Limb('left')
left_camera = bax.CameraController('left_hand_camera')
left_camera.resolution = (640,400)
left_camera.open()
left_image = None
right_camera = bax.CameraController('right_hand_camera')
right_camera.resolution = (640,400)
right_camera.open()
right_image = None


ortho_view = {'right_s0': 1.0526943144836427, 'right_s1': -0.8590292402343751, 'right_w0': -0.39538354762573247, 'right_w1': 1.4714710690979005, 'right_w2': 0.7792622393554688, 'right_e0': 0.6139758096496583, 'right_e1': 0.9894176070556642}

tuck_right = {'right_s0': -0.3183010131225586, 'right_s1': -1.3598739669067383, 'right_w0': -0.25885925765991213, 'right_w1': 0.45674277907104494, 'right_w2': 0.243519449798584, 'right_e0': 0.49240783234863283, 'right_e1': 2.4954032438415528}



def get_right(msg):
	global right_image
	right_image = cv_bridge.CvBridge().imgmsg_to_cv2(msg, desired_encoding='rgb8')

def get_left(msg):
	global left_image
	left_image = cv_bridge.CvBridge().imgmsg_to_cv2(msg, desired_encoding='rgb8')

left_sub = rospy.Subscriber( 'cameras/left_hand_camera/image', Image, get_left )

right_sub = rospy.Subscriber( 'cameras/right_hand_camera/image', Image, get_right )


def IDBlock(img):
	img = img[0:300, 160:520]
	thresh, im = cv2.threshold(img, 127,255,cv2.THRESH_BINARY)
	imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
	ret,thresh = cv2.threshold(imgray,127,255,0)
	contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cv2.drawContours(im,contours,-1,(0,255,0),-1)
	my_contours = []
	for cnt in contours:	
		if cv2.contourArea(cnt)>200: 
			my_contours.append(cnt)
	print "# contours:", len(my_contours)
	cv2.drawContours(im,my_contours,-1,(255,0,0),-1)
	if(len(my_contours)==1):
		tile_type = 1
	else:
		centres = []
		for i in range(len(my_contours)):
			mom = cv2.moments(my_contours[i])
			centre = Point()
			centre.x = (int(mom['m10']/mom['m00']))
			centre.y = (int(mom['m01']/mom['m00']))
			centres.append(centre)
			print centres[i].x, ",", centres[i].y
		line = cv2.line(im, (centres[0].x,centres[0].y), (centres[1].x,centres[1].y), (0,255,0), 1)
		print centres[0].x, centres[1].x, centres[0].y, centres[1].y
		if (centres[0].x > centres[1].x) and (centres[0].y > centres[1].y):
			tile_type = 2
		elif (centres[1].x > centres[0].x) and (centres[1].y > centres[0].y):
			tile_type = 2
		else: tile_type = 3
		print centres
		print tile_type
	#cv2.imwrite('img_o.jpg',imgray)
	cv2.imwrite('img.jpg',im)
	cv2.imshow('img',im)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

while left_image==None:
	pass

IDBlock(left_image)



