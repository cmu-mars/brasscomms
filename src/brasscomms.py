#! /usr/bin/env python
from __future__ import with_statement
import roslib
import rospy
import actionlib
import ig_action_msgs.msg
import sys
import tf


from threading import Lock

import datetime

from flask import Flask , request , abort
from enum import Enum
app = Flask(__name__)

shared_var_lock = Lock ()


## some definitions and helper functions
class Status(Enum):
    PERTURBATION_DETECTED  = 1
    MISSION_SUSPENDED = 2
    MISSION_RESUMED = 3
    MISSION_HALTED = 4
    MISSION_ABORTED = 5
    ADAPTATION_INITIATED = 6
    ADAPTATION_COMPLETED = 7
    ADAPTATION_STOPPED = 8
    ERROR = 9

class Error(Enum):
    TEST_DATA_URI_ERROR  = 1
    TEST_DATA_FORMAT_ERROR = 2
    DAS_LOG_URI_ERROR = 3
    DAS_ERROR = 4

## checks to see if a string represents an integer
def isint(x):
    try:
        int(s)
        return True
    except ValueError:
        return False

def isbool(x):
    if (x == 'true' or x == 'false'):
        return True
    return False

## returns true iff the first argument is a digit inclusively between the
## second two args. assumes that the second two are indeed digits, and that
## the second is less than the third.
def int_out_of_range(x,upper,lower) :
    return not(isint(x) and x >= lower and x <= upper)

## todo: this could be a horrible concurrency bug; i don't know yet.
start_percentage = -1
bot_status = Status.Starting

## callbacks to change the status
def done_cb(terminal, result):
    global bot_status
    bot_status = Status.Completed
    print "brasscomms received successful result from plan: %d" %(terminal)  

def active_cb():
    global bot_status
    bot_status = Status.Operational
    print "brasscoms received notification that goal is active"

### subroutines for forming API results
def formActionResult(arguments):
    now = datetime.datetime.now()
    ACTION_RESULT = {TIME : now.isoformat (), 
			ARGUMENTS: arguments}
    return ACTION_RESULT

def th_error():
    return Response(status=400)

def action_result(body):
    return Response(flask.jsonify(**body),status=200, mimetype='application/json')

### subroutines per URL in API wiki page order

@app.route('/action/start', methods=['POST'])
def action_start():
    assert request.path == '/action/start'
    assert request.method == 'POST'
    # todo: post error if these asserts fail

    print "starting challenge problem"
    try:
        igfile = open('/home/vagrant/catkin_ws/src/cp1_gazebo/instructions/newnav.ig', "r")
        igcode = igfile.read()
        goal = ig_action_msgs.msg.InstructionGraphGoal(order=igcode)
        global client
        client.send_goal( goal = goal, done_cb = done_cb, active_cb = active_cb)
    except Exception as e:
        ## todo: here post the error to the relevant location in brass-th
        print e
        print "Could not send the goal!"

    return 'starting challenge problem' ## todo

# TODO: Remove this API
@app.route('/action/map', methods=['GET'])
def action_map(arg):
    assert request.path == '/action/map'
    assert request.method == 'GET'

    return "this is a stub"

@app.route('/action/observe', methods=['GET'])
def action_map(arg):
    assert request.path == '/action/observe'
    assert request.method == 'GET'

    global gazebo

    try:
    	x, y, w = gazebo.get_turtlebot_state()
	observation = {x : x, y : y, w : w,
		       v : -1, # How to calculate velocity
		       voltage: -1 # Need to work this out
		      }
	return action_result(observation)
    except:
	return th_error()

@app.route('/action/set_battery', methods=['POST'])
def action_map(arg):
    assert request.path == '/action/set_battery'
    assert request.method == 'POST'

    return "this is a stub"

quarternion = tf.quaternion_from_euler(0, 0, 0) # Zero twist obstacle

@app.route('/action/place_obstacle', methods=['POST'])
def action_map(arg):
    assert request.path == '/action/place_obstacle'
    assert request.method == 'POST'
    assert request.headers['Content-Type'] == "application/json"
    
    params = request.get_json(silent=True)
    assert 'x' in params.keys()
    assert 'y' in params.keys()
    global gazebo

    obs_name = gazebo.place_new_obstacle(params[x], params[y])
    if obs_name is not None:
	ARGUMENTS = {obstacle_id : obs_name};
        ACTION_RESULT = createActionResult(ARGUMENTS)
        return action_result(ACTION_RESULT)
    else:
	return th_error()




@app.route('/action/remove_obstacle', methods=['POST'])
def action_map(arg):
    assert request.path == '/action/remove_obstacle'
    assert request.method == 'POST'
    assert request.headers['Content-Type'] == "application/json"
    
    params = request.get_json(silent=True)
    assert 'obstacle_id' in params.keys()

    obstacle_id = params[obstacle_id]

    global gazebo
    success = gazebo.delete_obstacle(obstacle_id)
    if success:
	ACTION_RESULT = createActionResult({})
	return action_result(ACTION_RESULT)
    else:
	return th_error()
 



# @app.route('/phase1/power/initial_settings', methods=['POST'])
# def initalSettings():
#     assert request.path == '/phase1/power/initial_settings'
#     assert request.method == 'POST'

#     adaptions = request.args.get('enable_adaptions','')
#     if not isbool (adaptions):
#         abort(400, 'adaptations must be either "true" or "false"')

#     startPercent = request.args.get('start_battery','')
#     if int_out_of_range (startPercent, 0, 100):
#         abort(400, 'start percentage out of range, must be between 0 and 100 inclusive')

#     # todo: if we're going to use it, set the global as well. this could be a
#     # horrible concurrency bug.
#     global start_percentage
#     start_percentage = startPercent

#     # assume that the locations are strings "x,y".
#     obsLoc = request.args.get('obstacle_location','')
#     [obsLoc_x , obsLoc_y] = obsLoc.split(',')
#     if not (isint(obsLoc_x) and isint(obsLoc_y)):
#         abort(400, 'object location is malformed. must of the form "int,int"')

#     adaptPercent = request.args.get('minimum_battery','')
#     if int_out_of_range(adaptPercent, 0, 100) or adaptPercent >= startPercent:
#         abort(400, 'adapt percentage out of range, must be between 0 and 100 inclusive')

#     return "todo: make a call here now that the data's all checked"

# @app.route('/phase1/power/get_robot_location', methods=['GET'])
# def location():
#     assert request.path == '/phase1/power/get_robot_location'
#     assert request.method == 'GET'
#     return 'todo: make a ROS call here to determine location'

# @app.route('/phase1/power/get_battery_level', methods=['GET'])
# def battery():
#     assert request.path == '/phase1/power/get_battery_level'
#     assert request.method == 'GET'
#     return 'todo: make a ROS/gazebo extension call here to determine battery level.'

# @app.route('/phase1/power/change_power', methods=['POST'])
# def changePower():
#     assert request.path == '/phase1/power/change_power'
#     assert request.method == 'POST'

#     ## todo: this could be a horrible concurrency bug.
#     global start_percentage

#     ## todo: technically, this is not part of the spec given in the API.
#     if(start_percentage == -1):
#         abort(400, 'tried to change power before setting initial settings')

#     currentPower = request.args.get('current_battery','')
#     if int_out_of_range(currentPower, 0, start_percentage):
#         abort(400, 'current battery out of range, must be between 0 and 100 inclusive')

#     if currentPower >= start_percentage :
#         abort(400, 'current battery larger than the original battery setting. batteries only lose power')

#     return 'todo: make a call here to change the power'

# @app.route('/phase1/power/add_obstacle', methods=['POST'])
# def addObstacle():
#     assert request.path == '/phase1/power/add_obstacle'
#     assert request.method == 'POST'

#     obs_loc = request.args.get('obstacle_location','')

#     [obsLoc_x , obsLoc_y] = obs_loc.split(',')
#     if not (isint(obsLoc_x) and isint(obsLoc_y)):
#         abort(400, 'object location malformed. must be of the form "int,int".')

#     return 'todo: make a call to add the obstactle here'

# @app.route('/phase1/power/remove_obstacle', methods=['POST'])
# def removeObstacle():
#     assert request.path == '/phase1/power/remove_obstacle'
#     assert request.method == 'POST'

#     return 'todo: make a call to remove the obstacle here'

# @app.route('/phase1/recalibration/start_challenge_problem', methods=['POST'])
# def recal_start():
#     assert request.path == '/phase1/recalibration/start_challenge_problem'
#     assert request.method == 'POST'

#     return 'todo: make a call to start navigation through map for CP2'

# @app.route('/phase1/recalibration/initial_settings', methods=['POST'])
# def recal_init():
#     assert request.path == '/phase1/recalibration/initial_settings'
#     assert request.method == 'POST'

#     adaptions = request.args.get('enable_adaptions','')
#     if not isbool (adaptions):
#         abort(400, 'adaptations must be either "true" or "false"')

#     kinect_dx = request.args.get('kinect_dx','')
#     if int_out_of_range(kinect_dx, -5, 5):
#         abort(400, 'initial kinect dx out of range')

#     kinect_dy = request.args.get('kinect_dy','')
#     if int_out_of_range(kinect_dy, -5, 5):
#         abort(400, 'initial kinect dy out of range')

#     kinect_dz = request.args.get('kinect_dz','')
#     if int_out_of_range(kinect_dz, -5, 5):
#         abort(400, 'initial kinect dz out of range')

#     kinect_dr = request.args.get('kinect_dr','')
#     if int_out_of_range(kinect_dr, -30, 30):
#         abort(400, 'initial kinect dr out of range')

#     kinect_dp = request.args.get('kinect_dp','')
#     if int_out_of_range(kinect_dp, -30, 30):
#         abort(400, 'initial kinect dp out of range')

#     kinect_dw = request.args.get('kinect_dw','')
#     if int_out_of_range(kinect_dw, -30, 30):
#         abort(400, 'initial kinect dw out of range')

#     return 'todo: make a call here; what is this supposed to do, exactly? no spec given'

# ## todo: are you allowed to call this if you have an initial perturbation
# ## or not?
# @app.route('/phase1/recalibration/change_settings', methods=['POST'])
# def recal_change():
#     assert request.path == '/phase1/recalibration/change_settings'
#     assert request.method == 'POST'

#     kinect_dx = request.args.get('kinect_dx','')
#     if int_out_of_range(kinect_dx, -5, 5):
#         abort(400, 'updated kinect dx out of range')

#     kinect_dy = request.args.get('kinect_dy','')
#     if int_out_of_range(kinect_dy, -5, 5):
#         abort(400, 'updated kinect dy out of range')

#     kinect_dz = request.args.get('kinect_dz','')
#     if int_out_of_range(kinect_dz, -5, 5):
#         abort(400, 'updated kinect dz out of range')

#     kinect_dr = request.args.get('kinect_dr','')
#     if int_out_of_range(kinect_dr, -30, 30):
#         abort(400, 'updated kinect dr out of range')

#     kinect_dp = request.args.get('kinect_dp','')
#     if int_out_of_range(kinect_dp, -30, 30):
#         abort(400, 'updated kinect dp out of range')

#     kinect_dw = request.args.get('kinect_dw','')
#     if int_out_of_range(kinect_dw, -30, 30):
#         abort(400, 'updated kinect dw out of range')

#     return 'todo: make a call here'

# @app.route('/phase1/power/get_current_state', methods=['GET'])
# def recal_state():
#     assert request.path == '/phase1/power/get_current_state'
#     assert request.method == 'GET'
#     return 'todo: make a ROS call here to determine status. specify output format.'

# @app.route('/phase1/recalibration/stop_challenge_problem', methods=['GET'])
# def recal_stop():
#     assert request.path == '/phase1/recalibration/stop_challenge_problem'
#     assert request.method == 'GET'
#     return 'todo: make a ROS call to stop the challenge problem'


# if you run this script from the command line directly, this causes it to
# actually launch the little web server and the node
#
# the host parameter above make the server visible externally to any
# machine on the network, rather than just this one. in the context of
# the simulator, this combined with configured port-forwarding in the
# Vagrant file means that you can run curl commands against the guest
# machine from the host. for debugging, this may be unsafe depending
# on your machine configuration and network attachements.
if __name__ == "__main__":
    ## start up the ros node and make an action server
    rospy.init_node("brasscomms")
    client = actionlib.SimpleActionClient("ig_action_server", ig_action_msgs.msg.InstructionGraphAction)
    client.wait_for_server()
    gazebo = GazeboInterface()
    app.run (host="0.0.0.0")
    
