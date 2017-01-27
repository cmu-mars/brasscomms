
from __future__ import with_statement
from threading import Lock
import os.path
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *
from geometry_msgs.msg import *
import tf.transformations as tf
import rospy

# This is the model for the obstacle
OBS_MODEL = os.path.expanduser('~/catkin_ws/src/cp_gazebo/models/box.sdf')

# These are translation coordinates between the map and gazebo 
# The two things should really be syncrhonized
X_MAP_TO_GAZEBO_TRANSLATION = 56
Y_MAP_TO_GAZEBO_TRANSLATION = 42

class GazeboInterface:
    # Class to manage interaction with Gazebo. This class should be instantiated only once.
        
	obstacle_names = None # Name of known obstacles
	obstacle_sequence = None # The next number in the sequence
	lock = None # A lock to manage multiple threads
	zero_q = None # A quarternion for zero twist

	def __init__(self):
		self.obstacle_names = []
		self.obstacle_sequence = 0
		self.lock = Lock ()
		self.zero_q = tf.quaternion_from_euler(0, 0, 0) # Zero twist obstacle
		file_xml = open(OBS_MODEL)
		self.obs_xml = file_xml.read()

		# Services to gazebo
		self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
		self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
		rospy.wait_for_service('/gazebo/get_model_state')
		rospy.wait_for_service('/gazebo/spawn_gazebo_model')

	def translateMapToGazebo(self,mx, my):
	    # Translate from map coordinates to gazebo coordinates
	    # :param mx: x coordinate in map
	    # :param my: y coordinate in map
	    # :return x, y in gazebo
		return mx - X_MAP_TO_GAZEBO_TRANSLATION, my - Y_MAP_TO_GAZEBO_TRANSLATION

	def translateGazeboToMap(self, gx, gy):
	    # Translate from gazebo coordinates to map coordinates
	    # :param mx: x coordinate in gazebo
	    # :param my: y coordinate in gazebo
	    # :return x, y in map
		return gx + X_MAP_TO_GAZEBO_TRANSLATION, gy + Y_MAP_TO_GAZEBO_TRANSLATION

	def get_turtlebot_state(self):
	    # Queries gazebo to get the current state of the turtlebot
	    # :return x,y in map coordinates; w as the yaw
		try:
			tb = self.get_model_state('mobile_base', '')
			q = (tb.pose.orientation.x,
			    tb.pose.orientation.y,
			    tb.pose.orientation.z,
			    tb.pose.orientation.w)
			w = tf.euler_from_quaternion(q)[2]
			x, y =  self.translateGazeboToMap (tb.pose.position.x, tb.pose.position.y)
			return x, y, w
		except rospy.ServiceException, e:
			rospy.logerror("Could not get state of robot: %s"%e)
			return None, None, None

	def place_new_obstacle(self,x,y):
	    # Place a new obstacl at te map coordinates indicated. The obstacles is
	    # given an automatic naming that is returned
	    # :param x: x coordinated indicating center of obstacle in the map
	    # :param y: y coordinate indicating center of obstacle in map
	    # :return the name of the obstacle, or None if failed

		# set up position of the obstacle
		gx, gy = self.translateMapToGazebo(x,y)
		pose = Pose ()
		pose.position.x = gx
		pose.position.y = gy
		pose.position.z = 0.75 # constant because obstacle is 1.5 meters tall
		pose.orientation.x = self.zero_q[0]
		pose.orientation.y = self.zero_q[1]
		pose.orientation.z = self.zero_q[2]
		pose.orientation.w = self.zero_q[3]

        # Generate obstacles name (in threadsafe manner)
		obs_name = ''
		with self.lock:
		    obs_name = "Obstacle_%s" %str(self.obstacle_sequence)
		    
		# Create gazebo request and call
		req = SpawnModelRequest()
		req.model_name = obs_name
		req.model_xml = self.obs_xml
		req.initial_pose = pose
		try:
			res = self.spawn_model(req)
			if res.success:
				with self.lock:
					self.obstacle_sequence = self.obstacle_sequence + 1
					self.obstacle_names.append(obs_name)
				return obs_name
			else:
				rospy.logerror("Could not place obstacle. Message: %s" %res.status_message)
				return None
		except rospy.ServiceException, e:
			rospy.logerror("Could not place obstacle. Message %s" %e)
			return None
			
	def delete_obstacle(self, obs_name):
	    # Deletes a model (obstacle) from gazebo. The obs_name must exist

		with self.lock:
			if obs_name not in self.obstacle_names:
				rospy.logerror("Nonexistent obstacle [%s]"%obs_name)
				return False
		req = DeleteModel()
		req.model_name = obs_name
		try:
			res = self.delete_model(req)
		
			if res.success:
				with self.lock:
					self.obstacle_names.remove(obs_name)
				return True
			else:
				rospy.logerror("Could not place obstacle. Message: %s" %res.status_message)
				return False
		except rospy.ServiceException, e:
			rospy.logerror("Could not place obstacle. Message %s"%e)
			return False
          
# This is just for testing  
if __name__ == "__main__":
	# In testing
	rospy.init_node("gazebo_interface_test")
	gazebo = GazeboInterface() 

	x,y,w = gazebo.get_turtlebot_state()
        print("Turtlebot is at (%s, %s), facing %s" %(str(x),str(y),str(w)))

	new_obs = gazebo.place_new_obstacle(19.5, 58.5)
	
	print("Successfully placed an obstacle called %s" %new_obs)
	
	new_obs = gazebo.place_new_obstacle(52, 76)
	print("Deleting %s" %new_obs)
	success = gazebo.delete_model(new_obs)
	print("Added and deleted a model, success=%s" %str(success))

	
