#IMPORTS
import numpy as np
from math import hypot, sqrt
from random import randrange as rand
import cv2

#Value assignments
gui = 1             #acts as truth value if gui is open
clickCount = 0      #counts number of clicks for the double click to choose start and end points
max_dist = 20       #maximum distance between two nodes of a tree
delta_radius = 10   #radius around the end point to be determined as the finishing area

# Distance b/w two points
def dist(p1,p2):
  return hypot(p2[0]-p1[0],p2[1]-p1[1])


class Vertex:
  def __init__(self,pos,parent):
    self.pos = pos
    self.parent = parent


class RRT_star_connect :

	def __init__(self,env_image):
		print("Running...")
		print("First double click at the desired starting point, then double click at desired goal point")
		self._initPos = (0,0)
		self._targetPos = (0,0)
		self._map = env_image.copy()
		self._map[self._map==255] = 1
		self._height = env_image.shape[0]
		self._width = env_image.shape[1]
		self._path=[]


	def start_connect(self,env_image=None):
		"""
		Path-Planning-connect based algorithm
		"""

		goal = False
		count = 1
		self._path=[]
		newvertex=[]
		vertices=[]
		newvertex.append(Vertex(self._initPos,None))
		newvertex.append(Vertex(self._targetPos,None))
		vertices.append([newvertex[0]])
		vertices.append([newvertex[1]])

		# main loop
		while not goal:

			count = (count + 1) % 2     #to select the tree (either from the start or the one from the goal)
			# create random point
			newpoint = (rand(self._width),rand(self._height))
			nearest_dist = float('inf')
			# look for the nearest point in the tree
			for v in vertices[count]:
				curr_dist = dist(newpoint,v.pos)
				if curr_dist < nearest_dist:
					nearest = v
					nearest_dist = curr_dist

			newpoint=self.steer(nearest.pos,newpoint)

			# try to connect the point to the tree
			if not self.collide_line(nearest.pos,newpoint):
				newvertex[count] = Vertex(newpoint,nearest)
				vertices[count].append(newvertex[count])

				if gui:
					if count % 2 == 0:
						env_image=cv2.circle(env_image,newpoint,2,(255,0,0),-1)
						env_image=cv2.line(env_image,newpoint,nearest.pos,(255,0,0),1)
					else:
						env_image=cv2.circle(env_image,newpoint,2,(255,125,0),-1)
						env_image=cv2.line(env_image,newpoint,nearest.pos,(255,125,0),1)
					cv2.imshow('Path-Planning', env_image)
					cv2.waitKey(10)

				# try to connect the point to the other tree
				nearest_dist = float('inf')
				for v in vertices[(count+1)%2]:
					curr_dist = dist(newpoint,v.pos)
					if curr_dist < nearest_dist:
						nearest = v
						nearest_dist = curr_dist


				newpoint=self.steer(nearest.pos,newpoint)

				#connecting point to tree
				if not self.collide_line(newpoint,nearest.pos):
					#checking if goal reached
					if newpoint == vertices[count][-1].pos:
						goal = True
					else:
						newvertex[(count+1)%2] = Vertex(newpoint,nearest)
						vertices[(count+1)%2].append(newvertex[(count+1)%2])
						if gui:
							env_image=cv2.circle(env_image,newpoint,2,(255,120,0),-1)
							env_image=cv2.line(env_image,newpoint,nearest.pos,(255,120,0),1)
							cv2.imshow('Path-Planning', env_image)
							cv2.waitKey(10)

		#path building
		self._path =[]

		if count == 0:
			currentvertex1 = newvertex[count]
			currentvertex2 = nearest
		else:
			currentvertex1 = nearest
			currentvertex2 = newvertex[count]

		while currentvertex1.parent:
			self._path.append(currentvertex1.pos)
			currentvertex1 = currentvertex1.parent
		self._path.append(currentvertex1.pos)
		self._path.reverse()
		while currentvertex2.parent:
			self._path.append(currentvertex2.pos)
			currentvertex2 = currentvertex2.parent
		self._path.append(currentvertex2.pos)

		self.shorten_path()

		if gui:
			self.draw_path(env_image)

	def shorten_path(self):
		'''
		Shortening path (star)
		'''
		path=self._path[:]
		self._path=[]
		self._path.append(path[0])
		i = 0
		j = i + 2
		while j < len(path):
			if self.collide_line(path[i],path[j]):
				self._path.append(path[j-1])
				i = j-1
			j += 1
		self._path.append(path[j-1])


	def draw_path(self,env_image):
		"""
		Generating red path
		"""
		for i in range(1,len(self._path)):
			env_image=cv2.line(env_image,self._path[i-1],self._path[i],(0,0,255),4)
			cv2.imshow('Path-Planning', env_image)
			cv2.waitKey(10)


	def steer(self,startpoint,directedpoint):
		if dist(startpoint,directedpoint)<max_dist:
			return directedpoint
		else:
			d = sqrt((directedpoint[1]-startpoint[1])*(directedpoint[1]-startpoint[1])+(directedpoint[0]-startpoint[0])*(directedpoint[0]-startpoint[0]))
			return (int(startpoint[0]+max_dist/d*(directedpoint[0]-startpoint[0])),int(startpoint[1]+max_dist/d*(directedpoint[1]-startpoint[1])))


	def collide_line(self,start,end):
		"""
		Checking path feasibility
		"""
		img = np.zeros((self._height,self._width))
		img = cv2.line(img,start,end,1,1)
		intersection = np.logical_and( self._map, img)
		if np.count_nonzero(intersection)==0:
			return False
		else:
			return True

	def collide_circle(self,point,radius):
		"""
		Checking point feasibility
		"""
		img = np.zeros((self._height,self._width))
		img = cv2.circle(img,point,radius,1,-1)
		intersection = np.logical_and( self._map, img)
		if np.count_nonzero(intersection)==0:
			return False
		else:
			return True


	def test_goal(self,point):
		"""
		If target point is reached (with some delta error)
		"""
		if (self._targetPos[0]-delta_radius<point[0]<self._targetPos[0]+delta_radius) and (self._targetPos[1]-delta_radius<point[1]<self._targetPos[1]+delta_radius):
			return True
		else:
			return False

	'''
	mouse clicking function
	for selecting start and end points
	'''
	def pos_define(self,event,x,y,flags,param):
		global clickCount
		if event == cv2.EVENT_LBUTTONDBLCLK:
			if not self.collide_circle((x,y),delta_radius):
				if clickCount==0:
					self._initPos = (x,y)
					param=cv2.circle(param,(x,y),delta_radius,(0,255,0),-1)
				if clickCount==1:
					self._targetPos = (x,y)
					param=cv2.circle(param,(x,y),delta_radius,(0,0,255),-1)
				clickCount +=1


if __name__ == '__main__':
	env_image = cv2.imread("map.png",1)
	loaded_env= cv2.cvtColor(env_image,cv2.COLOR_BGR2GRAY)
	ret,loaded_env = cv2.threshold(loaded_env,175,255,cv2.THRESH_BINARY)

	rrt = RRT_star_connect(env_image=loaded_env)
	
	#deciding start and goal point (using double click)
	if gui:
		cv2.imshow('Path-Planning', env_image)
		cv2.setMouseCallback('Path-Planning',rrt.pos_define, param=env_image)
		while(clickCount<2):
			cv2.imshow('Path-Planning', env_image)
			if cv2.waitKey(20) & 0xFF == 27:
				break
		rrt.start_connect(env_image)

	if gui:
		while(1):
			cv2.imshow('Path-Planning', env_image)
			if cv2.waitKey(10) & 0xFF == 27:
				break
		cv2.destroyAllWindows()
	print("quit")