
from __future__ import with_statement
from threading import Lock
import os.path
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *
from geometry_msgs.msg import *
import tf.transformations as tf
import rospy
import math

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
		self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
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

	def set_turtlebot_position(self, mx, my, w):
		# Sets the position of the turtlebot in gazebo and AMCL
		# :param mx: x ccoordinate of the robot according to the map
		# :param my: y coordinate of the robot according to the map
		# :param w: rotation of the robot
		try:
			# Start by getting the current state of the robot
			tb = self.get_model_state('mobile_base', '')
			# Check to see if robot is moving. If it is, return an error
			#if (tb.twist.linear.x != 0 or tb.twist.linear.y != 0 or
			#	tb.twist.linear.z != 0 or tb.twist.angular.x!= 0 or
			#	tb.twist.angular.y!= 0 or tb.twist.angular.z != 0):
			#	rospy.logerr('Cannot set the robot position while it is moving')
			#	return False	
			
			# Set the position to the new position
			tb.pose.position.x, tb.pose.position.y = self.translateMapToGazebo(mx, my)
			q = (tb.pose.orientation.x,
			    tb.pose.orientation.y,
			    tb.pose.orientation.z,
			    tb.pose.orientation.w)
			# Set the angular rotation
			eu = tf.euler_from_quaternion(q)
			eu = eu[0], eu[1], w
			q = tf.quaternion_from_euler(eu[0], eu[1], eu[2])
			tb.pose.orientation.x = q[0]
			tb.pose.orientation.y = q[1]
			tb.pose.orientation.z = q[2]
			tb.pose.orientation.w = q[3]

			# Create the new position message
			ms = ModelState()
			ms.model_name='mobile_base'
			ms.pose = tb.pose
			ms.twist = tb.twist
			res = self.set_model_state(ms)
			if (res.success):
				# Tell the map where it is
				amcl = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
				ip = PoseWithCovarianceStamped()
				ip.header.stamp = rospy.Time.now()
				ip.header.frame_id = 'map'
				ip.pose.pose.position.x = mx
				ip.pose.pose.position.y = my
				ip.pose.pose.position.z = 0
				ip.pose.pose.orientation.x = tb.pose.orientation.x
				ip.pose.pose.orientation.y = tb.pose.orientation.y
				ip.pose.pose.orientation.z = tb.pose.orientation.z
				ip.pose.pose.orientation.w = tb.pose.orientation.w
				ip.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
				print ('Publishing amcl location')
				amcl.publish(ip)
				return True
			else:
				rospy.logerr("Failed to set gazebo robot position")
				return False
		except rospy.ServiceException, e:
			rospy.logerr ("Could not set the position of the robot")
			return False

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
			v = math.sqrt(tb.twist.linear.x**2+ tb.twist.linear.y**2)
			#print ("accelaration is %s, %s, %s" %(tb.twist.linear.x,tb.twist.linear.y,tb.twist.linear.z))
			x, y =  self.translateGazeboToMap (tb.pose.position.x, tb.pose.position.y)
			return x, y, w, v
		except rospy.ServiceException, e:
			rospy.logerr("Could not get state of robot: %s"%e)
			return None, None, None, None

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
				rospy.logerr("Could not place obstacle. Message: %s" %res.status_message)
				return None
		except rospy.ServiceException, e:
			rospy.logerr("Could not place obstacle. Message %s" %e)
			return None
			
	def delete_obstacle(self, obs_name):
	    # Deletes a model (obstacle) from gazebo. The obs_name must exist

		with self.lock:
			if obs_name not in self.obstacle_names:
				rospy.logerr("Nonexistent obstacle [%s]"%obs_name)
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
				rospy.logerr("Could not place obstacle. Message: %s" %res.status_message)
				return False
		except rospy.ServiceException, e:
			rospy.logerr("Could not place obstacle. Message %s"%e)
			return False
          
# This is just for testing  
if __name__ == "__main__":
	# In testing
	rospy.init_node("gazebo_interface_test")
	gazebo = GazeboInterface() 

	
	success = gazebo.set_turtlebot_position(19.8,58.8,0)
	if success:
		print ("set position!")
	else:
		print ('did not set position')
	x,y,w,v = gazebo.get_turtlebot_state()
	print("Turtlebot is at (%s, %s), facing %s and going %s ms" %(str(x),str(y),str(w),str(v)))


	new_obs = gazebo.place_new_obstacle(19.5, 58.5)
	
	print("Successfully placed an obstacle called %s" %new_obs)
	
	new_obs = gazebo.place_new_obstacle(52, 76)
	print("Deleting %s" %new_obs)
	success = gazebo.delete_model(new_obs)
	print("Added and deleted a model, success=%s" %str(success))

	
