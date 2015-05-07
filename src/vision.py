import numpy as np
import rospy
import cv2
import cv_bridge
import time
import sys
import baxter_interface as bax
from matplotlib import pyplot as plt

import coords

from sensor_msgs.msg import Image

rospy.init_node('Game_run')

#-----------------Defining object classes-----------------------------------#

class Point:
	def __init__(self):
		self.x = 0
		self.y = 0

class BoardSpace:
	def __init__(self,posU,posD,piece,x1,y1,x2,y2):
		self.posU = {}
		self.posD = {}
		self.piece = 0
		self.x1 = 0
		self.y1 = 0
		self.x2 = 0
		self.y2 = 0



limbR = bax.Limb('right')
limbL = bax.Limb('left')
gripper = bax.Gripper('left')
left_camera = bax.CameraController('left_hand_camera')
left_camera.resolution = (640,400)
left_camera.open()
left_image = None
right_camera = bax.CameraController('right_hand_camera')
right_camera.resolution = (640,400)
right_camera.open()
right_image = None
board_spaces = []

current_piece = 0


#------------------------Defining all of the board spaces-------------------#

h1=210
h2=270
h3=330
h4=390
h5=460
h6=520
v1=51
v2=113
v3=174
v4=238
v5=305

A1 = BoardSpace(coords.A1U, coords.A1D, 0, h1,v1,h2,v2)
board_spaces.append(A1)

A2 = BoardSpace(coords.A2U, coords.A2D, 0, h2,v1,h3,v2)
board_spaces.append(A2)

A3 = BoardSpace(coords.A3U, coords.A3D, 0, h3,v1,h4,v2)
board_spaces.append(A3)

A4 = BoardSpace(coords.A4U, coords.A4D, 0, h4,v1,h5,v2)
board_spaces.append(A4)

A5 = BoardSpace(coords.A5U, coords.A5D, 0, h5,v1,h6,v2)
board_spaces.append(A5)

B1 = BoardSpace(coords.B1U, coords.B1D, 0, h1,v2,h2,v3)
board_spaces.append(B1)

B2 = BoardSpace(coords.B2U, coords.B2D, 0, h2,v2,h3,v3)
board_spaces.append(B2)

B3 = BoardSpace(coords.B3U, coords.B3D, 0, h3,v2,h4,v3)
board_spaces.append(B3)

B4 = BoardSpace(coords.B4U, coords.B4D, 0, h4,v2,h5,v3)
board_spaces.append(B4)

B5 = BoardSpace(coords.B5U, coords.B5D, 0, h5,v2,h6,v3)
board_spaces.append(B5)

C1 = BoardSpace(coords.C1U, coords.C1D, 0, h1,v3,h2,v4)
board_spaces.append(C1)

C2 = BoardSpace(coords.C2U, coords.C2D, 0, h2,v3,h3,v4)
board_spaces.append(C2)

C3 = BoardSpace(coords.C3U, coords.C3D, 0, h3,v3,h4,v4)
board_spaces.append(C3)

C4 = BoardSpace(coords.C4U, coords.C4D, 0, h4,v3,h5,v4)
board_spaces.append(C4)

C5 = BoardSpace(coords.C5U, coords.C5D, 0, h5,v3,h6,v4)
board_spaces.append(C5)

D1 = BoardSpace(coords.D1U, coords.D1D, 0, h1,v4,h2,v5)
board_spaces.append(D1)

D2 = BoardSpace(coords.D2U, coords.D2D, 0, h2,v4,h3,v5)
board_spaces.append(D2)

D3 = BoardSpace(coords.D3U, coords.D3D, 0, h3,v4,h4,v5)
board_spaces.append(D3)

D4 = BoardSpace(coords.D4U, coords.D4D, 0, h4,v4,h5,v5)
board_spaces.append(D4)

D5 = BoardSpace(coords.D5U, coords.D5D, 0, h5,v4,h6,v5)
board_spaces.append(D5)

#---------------------------Code to retrieve images from Lucas-----------------#

def get_right(msg):
	global right_image
	right_image = cv_bridge.CvBridge().imgmsg_to_cv2(msg, desired_encoding='rgb8')

def get_left(msg):
	global left_image
	left_image = cv_bridge.CvBridge().imgmsg_to_cv2(msg, desired_encoding='rgb8')

left_sub = rospy.Subscriber( 'cameras/left_hand_camera/image', Image, get_left )

right_sub = rospy.Subscriber( 'cameras/right_hand_camera/image', Image, get_right )

#--------------------------Functions for everything needed to play the game-----#


def place(space):
	global current_piece
	global current_space
	limbL.move_to_joint_positions(coords.ResetPos)
	limbL.move_to_joint_positions(space.posU)
	limbL.move_to_joint_positions(space.posD)
	gripper.open()
	space.piece = current_piece
	current_space = space
	limbL.move_to_joint_positions(space.posU)
	limbL.move_to_joint_positions(coords.ResetPos)

def startTurn():
	global current_piece
	current_piece = 0
	limbL.move_to_joint_positions(coords.ResetPos)
	limbL.move_to_joint_positions(coords.PickU)
	limbL.move_to_joint_positions(coords.PickD)
	while current_piece == 0:
		current_piece = IDBlock(left_image)
		rospy.sleep(5)
	limbL.move_to_joint_positions(coords.PickU)
	scanBoard()

def move_to(bs):
	limbL.move_to_joint_positions(coords.ResetPos)
	limbL.move_to_joint_positions(bs.posU)
	limbL.move_to_joint_positions(bs.posD)

def pickUp():
	limbL.move_to_joint_positions(coords.ResetPos)
	limbL.move_to_joint_positions(coords.PickU)
	limbL.move_to_joint_positions(coords.PickD)
	gripper.close()
	rospy.sleep(2)
	limbL.move_to_joint_positions(coords.PickU)
	limbL.move_to_joint_positions(coords.ResetPos)

def scanBoard():
	global board_spaces
	limbR.move_to_joint_positions(coords.ortho_view)
	board = right_image
	for i in range(len(board_spaces)):
		if board_spaces[i].piece == 0:
			bs = board_spaces[i]
			space = board[bs.y1:bs.y2, bs.x1:bs.x2]
			if cv2.mean(space) >= 0.15:
				limbR.move_to_joint_positions(coords.tuck_right)
				move_to(bs)
				bs.piece = IDBlock(left_image)
				break
	limbR.move_to_joint_positions(coords.tuck_right)

def turnOne():
	limbR.move_to_joint_positions(coords.tuck_right)
	limbL.move_to_joint_positions(coords.ResetPos)
	startTurn()
	pickUp()
	place(board_spaces[17])
	current_space = board_spaces[12]

def IDBlock(img):
	cv2.imwrite('imgbase.jpg',img)
	img = img[0:300, 160:520]
	thresh, im = cv2.threshold(img, 127,255,cv2.THRESH_BINARY)
	cv2.imwrite('img2.jpg',im)
	imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
	ret,thresh = cv2.threshold(imgray,127,255,0)
	contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cv2.drawContours(im,contours,-1,(0,255,0),-1)
	cv2.imwrite('conts.jpg',im)
	my_contours = []
	tile_type = 0
	for cnt in contours:	
		if cv2.contourArea(cnt)>2000: 
			my_contours.append(cnt)
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
		if (centres[0].x > centres[1].x) and (centres[0].y > centres[1].y):
			tile_type = 2
		elif (centres[1].x > centres[0].x) and (centres[1].y > centres[0].y):
			tile_type = 2
		else: tile_type = 3
	#cv2.imwrite('img_o.jpg',imgray)
	cv2.imwrite('img.jpg',im)
	cv2.imwrite('img0.jpg', img)
	#cv2.imshow('img',im)
	#cv2.waitKey(0)
	#cv2.destroyAllWindows()
	print tile_type
	return tile_type

while left_image==None:
	pass

limbL.move_to_joint_positions(coords.B5U)
limbL.move_to_joint_positions(coords.B5D)

IDBlock(left_image)



