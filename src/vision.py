#vision.py
#Full code for the Final Year Project:
#"Tabletop Robotics: Board Games With a Baxter Robot"
# by Pascal Alexander Siddons, sc12pas@leeds.ac.uk


#----------------Importing required libraries-------------------------------#
import numpy as np
import rospy
import cv2
import cv_bridge
import time
import sys
import baxter_interface as bax
from select import select

import coords #The file containing all of the necessary joint angles

from sensor_msgs.msg import Image

rospy.init_node('Game_run')

#-----------------Defining object classes-----------------------------------#

class Point:
	"""A class to describe objects representing points in 2D coordinate systems"""
	def __init__(self):
		self.x = 0
		self.y = 0

class BoardSpace:
	"""A class to describe objects representing the 20 spaces on the game board"""
	def __init__(self,posU,posD,piece,x1,y1,x2,y2):
		self.posU = posU
		self.posD = posD
		self.piece = piece
		self.x1 = x1
		self.y1 = y1
		self.x2 = x2
		self.y2 = y2

#---------------Defining global variables and the parameters for the robots arms,-#
#------------------------Gripper and cameras.-------------------------------------#

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


#The first two parameters correspond to the joint angles required to 
#navigate to that particular space
A1 = BoardSpace(coords.A1U, coords.A1D, 0, h1,v1,h2,v2)
board_spaces.append(A1)
#The third is the piece currently occupying that space
#While the last four describe the area this space occupies in the orthogonal view
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

#---------------------------Code to retrieve images from Lucas------------------#

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
	"""A function to instruct Lucas to place a block in a particular space"""
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
	"""A function instructing Lucas to wait for the start of its turn before
	scanning the board to see where the Human has placed their tile"""
	global current_piece
	current_piece = 0
	limbL.move_to_joint_positions(coords.ResetPos)
	limbL.move_to_joint_positions(coords.PickU)
	limbL.move_to_joint_positions(coords.PickD)
	i = 0
	while current_piece == 0:
		current_piece = IDBlock(left_image)
		rospy.sleep(5)
		i+=1		
		if i >= 6:#After 30 seconds of inaction by the human player, Lucas assumes victory
			print "Huzzah! Another glorious victory for your robotic overlord!"
			sys.exit()
	limbL.move_to_joint_positions(coords.PickU)
	limbL.move_to_joint_positions(coords.ResetPos)
	scanBoard()
	limbL.move_to_joint_positions(coords.ResetPos)

def move_to(bs): #A function instructing Lucas to move the left arm to a particular space
	limbL.move_to_joint_positions(coords.ResetPos)
	limbL.move_to_joint_positions(bs.posU)
	limbL.move_to_joint_positions(bs.posD)

def move_from(bs): #A function instructing Lucas to move back from a particular space
	limbL.move_to_joint_positions(bs.posU)
	limbL.move_to_joint_positions(coords.ResetPos)


def pickUp(): #A Function instructing Lucas to pick up its tile for the turn
	limbL.move_to_joint_positions(coords.PickU)
	limbL.move_to_joint_positions(coords.PickD)
	current_piece = IDBlock(left_image)
	gripper.close()
	rospy.sleep(2)
	limbL.move_to_joint_positions(coords.PickU)
	limbL.move_to_joint_positions(coords.ResetPos)

def rotate(): # A function instructing Lucas to rotate its piece for the turn by 90 degrees
	limbL.move_to_joint_positions(coords.PickU)
	limbL.move_to_joint_positions(coords.rotateU)
	limbL.move_to_joint_positions(coords.rotateD)
	gripper.open()
	rospy.sleep(2)
	limbL.move_to_joint_positions(coords.rotateU)
	pickUp()

def nextSpace(piece):
	"""A function calculating, given the current piece and with knowledge of the current space
	and direction, what the space Lucas will be placing a piece in in its next turn will be"""
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
				next_space = 66
			else:
				next_space = current_space + 1
		elif piece == 3:
			if current_space == (0 or 5 or 10):
				next_space = 66
			else:
				next_space = current_space - 1
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
		elif piece == 3:
			if current_space == (9 or 14 or 19):
				next_space = 66
			else:
				next_space = current_space + 1

	elif direction == "right":
		if piece == 1:
			if current_space == 4 or current_space ==9 or current_space ==14 or current_space ==19:
				next_space = 66
			else:
				next_space = current_space + 1
		elif piece == 2:
			if 1 <= current_space <= 4:
				next_space = 66
			else:
				next_space = current_space - 5
		elif piece == 3:
			if 16 <= current_space <= 19:
				next_space = 66
			else:
				next_space = current_space + 5

	elif direction == "left":
		if piece == 1:
			if current_space == (0) or current_space == 5 or current_space == 10 or current_space == (15):
				next_space = 66
			else:
				next_space = current_space - 1
		elif piece == 2:
			next_space = current_space + 5
			if next_space >= 20:
				next_space = 66
		elif piece == 3:
			next_space = current_space - 5
			if next_space <= -1:
				next_space = 66

	else:
		print "+++?????+++ Out of Cheese Error. Redo From Start"
		sys.exit()
	if next_space <= -1:
		next_space = 66
	elif next_space >= 20:
		next_space = 66 
	#Defining next_space as 66 tells Lucas the next space is off of the board and that it has lost
	return next_space

	

def changeDirection():
	"""A function instructing Lucas to update the direction it is moving around the board in
	given the type of piece it has just placed"""
	global current_space
	global current_piece
	global next_space
	global direction


	if direction == "up":
		if current_piece == 3:
			direction = "left"
		elif current_piece == 2:
			direction = "right"

	elif direction == "down":
		if current_piece == 3:
			direction = "right"
		elif current_piece == 2:
			direction = "left"

	elif direction == "right":
		if current_piece == 3:
			direction = "down"
		elif current_piece == 2:
			direction = "up"

	elif direction == "left":
		if current_piece == 3:
			direction = "up"
		elif current_piece == 2:
			direction = "down"

def scanBoard():
	"""A function which scans all of the empty spaces on the board to
	see if any have changed. The one which is most likely to have changed is then
	scanned by Lucas"""
	global board_spaces
	limbL.move_to_joint_positions(coords.ResetPos)
	limbL.move_to_joint_positions(coords.left_back)
	limbR.move_to_joint_positions(coords.ortho_view)
	board = right_image
	thresh, bd = cv2.threshold(board, 127,255,cv2.THRESH_BINARY)
	array = []
	maxspace = -1
	meanmax = 0
	for i in range(len(board_spaces)):
		if board_spaces[i].piece == 0:
			bs = board_spaces[i]
			space = bd[bs.y1:bs.y2, bs.x1:bs.x2]		
			if np.mean(space) >= 0.4:		
				if np.mean(space) >= meanmax:
					meanmax = np.mean(space)
					maxspace = i
					line = cv2.line(bd, (bs.x1,bs.y1), (bs.x2,bs.y2), (0,255,0), 1)
	if maxspace >= 0:
		bs = board_spaces[maxspace]	
		cv2.imwrite('conts.jpg',bd)
		limbR.move_to_joint_positions(coords.tuck_right)
		move_to(bs)
		bs.piece = IDBlock(left_image)
		move_from(bs)
	limbR.move_to_joint_positions(coords.tuck_right)

def turnOne():# A function instructing Lucas with what to do on its first turn.
	global board_spaces
	global direction
	global next_space
	gripper.calibrate()
	limbR.move_to_joint_positions(coords.tuck_right)
	limbL.move_to_joint_positions(coords.ResetPos)
	startTurn()
	pickUp()
	if board_spaces[17].piece ==0:
		place(17)
		direction = "up"
		next_space = 12
	else:
		place(2)
		direction = "down"
		next_space = 7

def IDBlock(img):# A function to Identify the type of a block
	img = img[0:300, 160:520]
	thresh, im = cv2.threshold(img, 127,255,cv2.THRESH_BINARY)
	imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
	ret,thresh = cv2.threshold(imgray,127,255,0)
	contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cv2.drawContours(im,contours,-1,(0,255,0),-1)
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
		line = cv2.line(im, (centres[0].x,centres[0].y), (centres[1].x,centres[1].y), (0,255,0), 1)
		if (centres[0].x > centres[1].x) and (centres[0].y > centres[1].y):
			tile_type = 2
		elif (centres[1].x > centres[0].x) and (centres[1].y > centres[0].y):
			tile_type = 2
		else: tile_type = 3
	print tile_type
	return tile_type

def boardStatePrint():
	#This just prints out a nice little display of Lucas' current perspective of the board state into the console
	print "    1   2   3   4   5 "
	print "A","|",board_spaces[0].piece,"|",board_spaces[1].piece,"|",board_spaces[2].piece,"|",board_spaces[3].piece,"|",board_spaces[4].piece,"|"
	print "B","|",board_spaces[5].piece,"|",board_spaces[6].piece,"|",board_spaces[7].piece,"|",board_spaces[8].piece,"|",board_spaces[9].piece,"|"
	print "C","|",board_spaces[10].piece,"|",board_spaces[11].piece,"|",board_spaces[12].piece,"|",board_spaces[13].piece,"|",board_spaces[14].piece,"|"
	print "D","|",board_spaces[15].piece,"|",board_spaces[16].piece,"|",board_spaces[17].piece,"|",board_spaces[18].piece,"|",board_spaces[19].piece,"|"



while left_image==None:
	pass

	
#-------------Calling turnOne() and the main game loop--------------------------------------#

turnOne() #Start the game!

while gameLoss == False: # The main game loop, where the action happens and it all comes together.
	startTurn()
	print "State: ", current_piece, " ", direction, " ", current_space, board_spaces[current_space].piece
	current_space = next_space
	if board_spaces[current_space].piece != 0:
		while board_spaces[current_space].piece != 0:
			current_space = nextSpace(board_spaces[current_space].piece)
			changeDirection()
			if current_space == 66:
				gameLoss = True
				break
			print "State: ", current_piece, " ", direction, " ", current_space, board_spaces[current_space].piece

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

		if next == 66:
			if altnext == 66:
				place(current_space)
				gameLoss = True
			else:			
				rotate()
				place(current_space)
		elif next == 6 or next == 7 or next == 8 or next == 11 or next == 12 or next == 13:
			place(current_space)
			next_space = next
		elif altnext == 6 or altnext == 7 or altnext == 8 or altnext == 11 or altnext == 12 or altnext == 13:
			rotate()
			place(current_space)
			next_space = altnext
		else:
			place(current_space)
			next_space = next

	elif current_piece == 3:
		next = nextSpace(current_piece)
		altnext = nextSpace(2)
		print "next: ", next, " , ", altnext
		if next == 66:
			if altnext == 66:
				place(current_space)
				gameLoss = True
			else:			
				rotate()
				place(current_space)
		elif next == 6 or next ==7 or next ==8 or next ==11 or next ==12 or next == 13:
			place(current_space)
			next_space = next
		elif altnext == 6 or altnext ==7 or altnext ==8 or altnext ==11 or altnext ==12 or altnext ==13:
			rotate()
			place(current_space)
			next_space = altnext
		else:
			place(current_space)
			next_space = next
	changeDirection()
	boardStatePrint()
	print next_space

if gameLoss == True:
	print "+++Divide By Cucumber Error. Please Reinstall Universe And Reboot+++"
	sys.exit()
