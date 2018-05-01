import numpy as np
import math
class Robot(object):
	def __init__(self):
		self.dh 			= []
		self.transformation = []

	def tf_matrix(self,theta,dh):
		return np.array([[math.cos(theta), -math.sin(theta)*math.cos(dh[2]),  math.sin(theta)*math.sin(dh[2]), dh[1]*math.cos(theta)],\
						 [math.sin(theta),  math.cos(theta)*math.cos(dh[2]), -math.cos(theta)*math.sin(dh[2]), dh[1]*math.sin(theta)],\
						 [			    0,                  math.sin(dh[2]),                  math.cos(dh[2]),   	  		   dh[0]],\
						 [			    0,								   0,								0,					   1]])

	def composite_transformation(self,thetas):
		self.transformation	 = np.identity(4)
		thetas		 		 = np.array(thetas)+[0,np.pi/2,0,0,0,0]
		for i in range(self.dh.shape[0]):
			self.transformation  = np.dot(self.transformation,self.tf_matrix(thetas[i],self.dh[i,:]))
		return self.transformation

class ABB_IRB_140(Robot):
	def __init__(self):
		Robot.__init__(self)
		self.position = []
		self.dh 	  = np.array([[352,  70,  np.pi/2],\
			   	   			   	  [  0, 360,    	0],\
			   	  				  [  0,   0,  np.pi/2],\
			   	   				  [380,   0, -np.pi/2],\
			   	   				  [  0,   0,  np.pi/2],\
			  	   				  [ 65,   0,  	 	0]])

		self.thetas   = np.zeros(self.dh.shape[0])

	def forwardKinematics(self,thetas):
		return self.composite_transformation(thetas)

	def inverseKinematics(self,transformation):
		print(transformation)
		if np.linalg.norm(transformation[0:3,3])<1092:
			posWrist_0 = -np.dot(transformation,[[0],[0],[65],[-1]])
			theta_1    = math.atan2(posWrist_0[1],posWrist_0[0])
			posWrist_1 = -np.dot(np.linalg.solve(self.tf_matrix(theta_1,self.dh[0,:]),transformation),[[0],[0],[65],[-1]])
			temp 	   = (-(posWrist_1[0]**2+posWrist_1[1]**2)+(360**2+380**2))/(2*360*380)
			theta_3	   = math.atan2((1-temp**2)**0.5,temp) - np.pi/2
			theta_2    = -np.pi/2 + (math.atan2(380*math.cos(theta_3),(380*math.sin(theta_3) + 360)) + math.atan2(posWrist_1[1],posWrist_1[0]))
			thetas 	   = [theta_1,theta_2,theta_3]
			a 		   = np.identity(4)
			for i in range(3):
				a 	   = np.dot(a,self.tf_matrix(thetas[i],self.dh[i,:]))
			rotation   = np.linalg.solve(a,transformation)
			theta_5	   = math.atan2(((1-rotation[2,2]**2)**0.5),rotation[2,2])
			cos		   = rotation[0,2]/math.sin(theta_5)
			sin		   = rotation[1,2]/math.sin(theta_5)
			theta_4	   = math.atan2(sin,cos)
			cos		   = -rotation[2,0]/math.sin(theta_5)
			sin		   = rotation[2,1]/math.sin(theta_5)
			theta_6	   = math.atan2(sin,cos)
			return [theta_1,theta_2,theta_3]
		else:
			print('Position is not possible')
			return False

if __name__ =="__main__":
	thetas = [np.pi/2,np.pi/5,np.pi/7,np.pi,0,0]
	a = ABB_IRB_140()
	print(-np.dot(a.forwardKinematics([0.3217505543966422, -0.7516620081033731, -0.8180807255803945]),[[0],[0],[65],[-1]])) # wrist position
	# print(a.forwardKinematics(thetas))
	b = (a.inverseKinematics(a.forwardKinematics(thetas)))
	# print(thetas)
	# print(b)
	print(-np.dot(a.forwardKinematics(b),[[0],[0],[65],[-1]])) # wrist position
	# print(a.forwardKinematics(b))
