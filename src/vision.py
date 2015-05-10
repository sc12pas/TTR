import numpy as np
import rospy
import cv2
import cv_bridge
import time
import sys
import baxter_interface as bax

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
		self.posU = posU
		self.posD = posD
		self.piece = piece
		self.x1 = x1
		self.y1 = y1
		self.x2 = x2
		self.y2 = y2



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
current_space = 0
next_space = 0
current_piece = 0
direction = ""
gameLoss = False


#------------------------Defining all of the board spaces-------------------#

h1=195
h2=260
h3=335
h4=400
h5=470
h6=535
v1=44
v2=113
v3=174
v4=250
v5=315

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


def place(num):
	global current_piece
	global current_space
	global board_spaces
	space = board_spaces[num]
	limbL.move_to_joint_positions(coords.ResetPos)
	limbL.move_to_joint_positions(space.posU)
	limbL.move_to_joint_positions(space.posD)
	gripper.open()
	space.piece = current_piece
	current_space = num
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
	limbL.move_to_joint_positions(coords.ResetPos)
	scanBoard()
	limbL.move_to_joint_positions(coords.ResetPos)

def move_to(bs):
	limbL.move_to_joint_positions(coords.ResetPos)
	limbL.move_to_joint_positions(bs.posU)
	limbL.move_to_joint_positions(bs.posD)

def move_from(bs):
	limbL.move_to_joint_positions(bs.posU)
	limbL.move_to_joint_positions(coords.ResetPos)


def pickUp():
	#limbL.move_to_joint_positions(coords.ResetPos)
	limbL.move_to_joint_positions(coords.PickU)
	limbL.move_to_joint_positions(coords.PickD)
	current_piece = IDBlock(left_image)
	gripper.close()
	rospy.sleep(2)
	limbL.move_to_joint_positions(coords.PickU)
	limbL.move_to_joint_positions(coords.ResetPos)

def rotate():
	limbL.move_to_joint_positions(coords.PickU)
	limbL.move_to_joint_positions(coords.rotateU)
	limbL.move_to_joint_positions(coords.rotateD)
	gripper.open()
	rospy.sleep(2)
	limbL.move_to_joint_positions(coords.rotateU)
	pickUp()

def nextSpace(piece):
	global current_space
	global current_piece
	global next_space
	global direction


	if direction == "up":
		print "piece = ", piece
		if piece == 1:
			next_space = current_space - 5
			if next_space <= -1:
				next_space = 66
		elif piece == 2:
			if current_space == (4 or 9 or 14):
				print "ho ho, ha ha", current_space
				next_space = 66
			else:
				next_space = current_space + 1
				direction = "right"
		elif piece == 3:
			if current_space == (0 or 5 or 10):
				print "You got PEEP'D!", current_space
				next_space = 66
			else:
				next_space = current_space - 1
				direction = "left"
	elif direction == "down":
		if piece == 1:
			next_space = current_space + 5
			if next_space >= 20:
				next_space = 66
		elif piece == 2:
			if current_space == (5 or 10 or 15):
				next_space = 66
			else:
				next_space = current_space - 1
				direction = "left"
		elif piece == 3:
			if current_space == (9 or 14 or 19):
				next_space = 66
			else:
				next_space = current_space + 1
				direction = "right"

	elif direction == "right":
		if piece == 1:
			if current_space == (4 or 9 or 14 or 19):
				next_space = 66
			else:
				next_space = current_space + 1
		elif piece == 2:
			if 1 <= current_space <= 4:
				next_space = 66
			else:
				next_space = current_space - 5
				direction = "up"
		elif piece == 3:
			if 16 <= current_space <= 19:
				next_space = 66
			else:
				next_space = current_space + 5
				direction = "down"

	elif direction == "left":
		if piece == 1:
			if current_space == (0 or 5 or 10 or 15):
				next_space = 66
			else:
				next_space = current_space - 1
		elif piece == 2:
			next_space = current_space + 5
			if next_space >= 20:
				next_space = 66
			else:
				direction = "down"
		elif piece == 3:
			next_space = current_space - 5
			if next_space <= -1:
				next_space = 66
			direction = "up"

	else:
		print "+++?????+++ Out of Cheese Error. Redo From Start"
		sys.exit()

	return next_space
	

def scanBoard():
	global board_spaces
	limbL.move_to_joint_positions(coords.ResetPos)
	limbL.move_to_joint_positions(coords.left_back)
	limbR.move_to_joint_positions(coords.ortho_view)
	board = right_image
	thresh, bd = cv2.threshold(board, 127,255,cv2.THRESH_BINARY)
	for i in range(len(board_spaces)):
		if board_spaces[i].piece == 0:
			bs = board_spaces[i]
			space = bd[bs.y1:bs.y2, bs.x1:bs.x2]
			if np.mean(space) >= 0.4:
				limbR.move_to_joint_positions(coords.tuck_right)
				move_to(bs)
				bs.piece = IDBlock(left_image)
				move_from(bs)
				break
	limbR.move_to_joint_positions(coords.tuck_right)

def turnOne():
	global direction 
	gripper.calibrate()
	limbR.move_to_joint_positions(coords.tuck_right)
	limbL.move_to_joint_positions(coords.ResetPos)
	startTurn()
	pickUp()
	place(17)
	direction = "up"
	#next_space = 12

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
	elif(len(my_contours)==0):
		tile_type = 0
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
	print tile_type
	return tile_type

while left_image==None:
	pass


turnOne()

while gameLoss == False:
	startTurn()
	while board_spaces[current_space].piece != 0:
		current_space = nextSpace(board_spaces[current_space].piece)

	pickUp()
	print "State: ", current_piece, " ", direction, " ", current_space
	if current_piece == 1:
		place(current_space)
		next = nextSpace(current_piece)
		if next == 66:
			gameLoss = True
	elif current_piece == 2:
		print "State: ", current_piece, " ", direction, " ", current_space
		next = nextSpace(current_piece)
		altnext = nextSpace(3)
		print "next: ", next, " , ", altnext
		if current_space == (6 or 7 or 8 or 11 or 12 or 13):
			if next == (6 or 7 or 8 or 11 or 12 or 13):
				place(current_space)
			elif altnext == (6 or 7 or 8 or 11 or 12 or 13):
				rotate()
				place(current_space)

		if next == 66:
			if altnext == 66:
				place(current_space)
				gameLoss = True
			else:			
				rotate()
				place(current_space)
		elif next == (6 or 7 or 8 or 11 or 12 or 13):
			place(current_space)
		elif altnext == (6 or 7 or 8 or 11 or 12 or 13):
			rotate()
			place(current_space)
		else:
			place(current_space)

	elif current_piece == 3:
		next = nextSpace(current_piece)
		altnext = nextSpace(2)
		print "next: ", next, " , ", altnext
		if current_space == (6 or 7 or 8 or 11 or 12 or 13):
			if next == (6 or 7 or 8 or 11 or 12 or 13):
				place(current_space)
			elif altnext == (6 or 7 or 8 or 11 or 12 or 13):
				rotate()
				place(current_space)

		if next == 66:
			if altnext == 66:
				place(current_space)
				gameLoss = True
			else:			
				rotate()
				place(current_space)
		elif next == (6 or 7 or 8 or 11 or 12 or 13):
			place(current_space)
		elif altnext == (6 or 7 or 8 or 11 or 12 or 13):
			rotate()
			place(current_space)
		else:
			place(current_space)

if gameLoss == True:
	print "+++Divide By Cucumber Error. Please Reinstall Universe And Reboot+++"
	sys.exit()

		
		 



