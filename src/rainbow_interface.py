from __future__ import with_statement
from threading import Lock
import subprocess
import time
import os
import rospy
from constants import AdaptationLevels

RAINBOW_PATH = os.path.expanduser('~/das/rainbow-brass')

class RainbowInterface:
	# Class to manage interaction with Rainbow

	def __init__(self):
		self.processStarted = False
		self.lock = Lock()
		self.target = None

	def getTarget (self, challenge_problem):
		target=None
		if challenge_problem == AdaptationLevels.CP1_Adaptation:
			target="brass-cp1"
		elif challenge_problem == AdaptationLevels.CP2_Adaptation:
			target="brass-cp2"
		return target

	def startRainbow(self):
		with self.lock:
			if self.processStarted:
				rospy.logerr("Trying to start Rainbow when Rainbow is already running.")
			self.processStarted = True

		if (self.target is None):
			return True
		rospy.loginfo("Starting Rainbow (DAS)...")
		ret = subprocess.call ([RAINBOW_PATH+"/brass.sh", "-w", RAINBOW_PATH, "-s", self.target, "/test/rainbow-start.log"])
		rospy.loginfo("Rainbow started, exit=%s"%str(ret))
		return ret == 0

	def launchRainbow(self, challenge_problem, log):
		"""
		Starts Rainbow process. Needs to (a) start in background, (b) wait some time until it is up

		"""
		print ("Configuring rainbow for %s"%challenge_problem)
		self.target = self.getTarget(challenge_problem)
		if (self.target is not None):
			time.sleep(10)
			print("Starting %s/run-oracle.sh %s"%(RAINBOW_PATH,self.target))
			subprocess.Popen([RAINBOW_PATH+"/run-oracle.sh", "-h", "-w", RAINBOW_PATH, self.target], stdout=log)
			time.sleep(40)


	def stopRainbow(self):
		rospy.loginfo("Stopping Rainbow (DAS)...")
		ret = subprocess.call ([RAINBOW_PATH+"/brass.sh","-q", "-w", RAINBOW_PATH, self.target])
		with self.lock:
			self.processStarted = False
