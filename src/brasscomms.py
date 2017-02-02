#! /usr/bin/env python

### imports
from __future__ import with_statement
import roslib
import rospy
import actionlib
import ig_action_msgs.msg
import sys
import tf
import math

from threading import Lock

import datetime

from flask import *
from enum import Enum

import requests
import json
import os.path

from gazebo_interface import *
from map_util import *

from move_base_msgs.msg import MoveBaseAction

### some definitions and helper functions
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
    TEST_DATA_FILE_ERROR  = 1
    TEST_DATA_FORMAT_ERROR = 2
    DAS_LOG_FILE_ERROR = 3
    DAS_OTHER_ERROR = 4

class LogError(Enum):
    STARTUP_ERROR = 1
    RUNTIME_ERROR = 2

# returns true iff the first argument is a digit inclusively between the
# second two args. assumes that the second two are indeed digits, and that
# the second is less than the third.
def int_out_of_range(x,lower,upper) :
    return not(isinstance(x,int) and x >= lower and x <= upper)

## callbacks to change the status
def done_cb(terminal, result):
    # todo: log this instead of printing it
    print "brasscomms received successful result from plan: %d" %(terminal)

def active_cb():
    # todo: log this instead of printing it
    print "brasscoms received notification that goal is active"

### some globals
app = Flask(__name__)
shared_var_lock = Lock ()
th_url = "http://brass-th"
deadline = datetime.datetime.now() ## this is a default value; the result of observe will be well formed but wrong unless they call start first


def parse_config_file():
    config_file_path = '/test/data'

    if not (os.path.exists(config_file_path)
            and os.path.isfile(config_file_path)
            and os.access(config_file_path,os.R_OK)):
        th_das_error(Error.TEST_DATA_FILE_ERROR,'config file at ' + config_file_path + ' either does not exist, is not a file, is not readable')
    else:
        with open(config_file_path) as config_file:
            data = json.load(config_file)

        # start_loc
        if (not ('start_loc' in data.keys())):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file does not contain start_loc')
        if(not (isinstance(data['start_loc'],unicode))):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file binding for start_loc is not a string')
        if(not (isWaypoint(data['start_loc']))):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file binding for start_loc is not a waypoint id')

        # start_yaw
        if (not ('start_yaw' in data.keys())):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file does not contain start_yaw')
        if(not (isinstance(data['start_yaw'],float))):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file binding for start_yaw is not a float')
        if(data['start_yaw'] < 0 or data['start_yaw'] > (2*math.pi)):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file binding for start_yaw is not in the range 0..2pi')

        # target_loc
        if (not ('target_loc' in data.keys())):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file does not contain target_loc')
        if(not (isinstance(data['target_loc'],unicode))):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file binding for target_loc is not a string')
        if(not (isWaypoint(data['target_loc']))):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file binding for target_loc is not a waypoint id')

        # enable_adaptation
        if (not ('enable_adaptation' in data.keys())):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file does not contain enable_adaptation')
        if (not (isinstance(data['enable_adaptation'], unicode))):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file binding for enable_adaptation is not a string')
        if (not data['enable_adaptation'] in ["CP1_NoAdaptation", "CP2_NoAdaptation", "CP1_Adaptation", "CP2_Adaptation"]):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file binding for enable_adaptation is not one of the enumerated forms')

        # initial_voltage
        if (not ('initial_voltage' in data.keys())):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file does not contain initial_voltage')
        if (not (isinstance(data['initial_voltage'], int))):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file binding for initial_voltage is not an integer')
        if (data['initial_voltage'] < 104 or data['initial_voltage'] > 166):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file binding for initial_voltage is out of range')

        # initial_obstacle
        if (not ('initial_obstacle' in data.keys())):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file does not contain inital_obstacle')
        if (not (isinstance(data['initial_obstacle'],bool))):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file binding for initial_obstacle is not a bool')

        # initial_obstacle_location
        if (not ('initial_obstacle_location' in data.keys())):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file does not contain initial_obstacle_location')
        if (not (isinstance(data['initial_obstacle_location'], dict))):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file binding for initial_obstacle_location is not a dict')
        # ... initial_obstacle_location[x]
        if (not ('x' in data['initial_obstacle_location'].keys())):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file does not contain initial_obstacle_location x coordinate')
        if (not (isinstance(data['initial_obstacle_location']['x'], float))):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file binding for initial_obstacle_location x coordinate is not a float')
        if (data['initial_obstacle_location']['x'] < -1 or data['initial_obstacle_location']['x'] > 24):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file binding for initial_obstacle_location x coordinate is out of range')
        # ... initial_obstacle_location[y]
        if (not ('y' in data['initial_obstacle_location'].keys())):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file does not contain initial_obstacle_location y coordinate')
        if (not (isinstance(data['initial_obstacle_location']['y'], float))):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file binding for initial_obstacle_location y coordinate is not a float')
        if (data['initial_obstacle_location']['y'] < -1 or data['initial_obstacle_location']['y'] > 24):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file binding for initial_obstacle_location y coordinate is out of range')

        # sensor_perturbation
        if (not ('sensor_perturbation' in data.keys())):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file does not contain sensor_perturbation')
        if (not (isinstance(data['sensor_perturbation'], dict))):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file binding for sensor_perturbation is not a dict')
        if (not (set(['x', 'y', 'z', 'p', 'w', 'r']).issubset(data['sensor_perturbation'].keys()))):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file binding for sensor_perturbation does not include all of: x,y,z,p,w,r')
        # .. sensor_perturbation[x]
        if(int_out_of_range(data['sensor_perturbation']['x'], -3, 3)):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file binding for sensor_perturbation x is not an in-range integer')
        # .. sensor_perturbation[y]
        if(int_out_of_range(data['sensor_perturbation']['y'], -3, 3)):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file binding for sensor_perturbation y is not an in-range integer')
        # .. sensor_perturbation[z]
        if(int_out_of_range(data['sensor_perturbation']['z'], -3, 3)):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file binding for sensor_perturbation z is not an in-range integer')
        # .. sensor_perturbation[p]
        if(int_out_of_range(data['sensor_perturbation']['p'], -6, 6)):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file binding for sensor_perturbation p is not an in-range integer')
        # .. sensor_perturbation[w]
        if(int_out_of_range(data['sensor_perturbation']['w'], -6, 6)):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file binding for sensor_perturbation w is not an in-range integer')
        # .. sensor_perturbation[r]
        if(int_out_of_range(data['sensor_perturbation']['r'], -6 , 6)):
            th_das_error(Error.TEST_DATA_FORMAT_ERROR, 'config file binding for sensor_perturbation r is not an in-range integer')

        # todo: stop the world if the file doesn't parse?

    # nb: we silently ignore anything extra information that might be
    # present at any level of the config.
    return data

### subroutines for forming API results
def formActionResult(result):
    now = datetime.datetime.now()
    ACTION_RESULT = {"TIME" : now.isoformat (),
                     "ARGUMENTS": result}
    return ACTION_RESULT

def th_error():
    return Response(status=400)

def action_result(body):
    with_time = formActionResult(body)
    return Response(json.dumps(with_time),status=200, mimetype='application/json')

### subroutines for forming and sending messages to the TH
def th_das_error(err,msg):
    global th_url
    now = datetime.datetime.now()
    error_contents = {"TIME" : now.isoformat(),
                      "ERROR" : err.name,
                      "MESSAGE" : msg}
    # todo: this r should be th_ack or th_err; do we care?
    try:
        r = requests.post(th_url+'/error', data = json.dumps(error_contents))
    except Exception as e:
        log_das_error(LogError.STARTUP_ERROR, "Fatal: couldn't connect to TH at " + th_url + "/error: " + str(e))

def log_das_error(error, msg):
    log_file_path = '/test/log'
    now = datetime.datetime.now()
    try:
        with open(log_file_path, 'a') as log_file :
            error_contents = {"TIME" : now.isoformat(),
                              "TYPE" : error.name,
                              "MESSAGE" : msg}
            data = json.dumps(error_contents)
            log_file.write(data + "\n")
    except StandardError as e:
        th_das_error(Error.DAS_LOG_FILE_ERROR,'log file at ' + log_file_path + ' could not be accessed')

def das_ready():
    global th_url
    now = datetime.datetime.now()
    contents = {"TIME" : now.isoformat ()}
    # todo: this r should be th_ack or th_err; do we care?
    try:
        r = requests.post(th_url+'/ready', data = json.dumps(contents))
    except Exception as e:
        log_das_error(LogError.STARTUP_ERROR, "Fatal: couldn't connect to TH at " + th_url + "/ready: " + str(e))

### helperfunctions for test actions

# also logs invalid action calls
def isValidActionCall(request, path, method) :
    if(request.path != path):
        log_das_error(LogError.RUNTIME_ERROR,'internal fault: '+ path + ' called improperly')
        return False
    elif(request.method != method):
        log_das_error(LogError.RUNTIME_ERROR, path + ' called with wrong HTTP method')
        return False
    else:
        return True

### subroutines per endpoint URL in API wiki page order
@app.route('/action/query_path', methods=['GET'])
def action_query_path():
    if (not isValidActionCall(request, '/action/query_path', 'GET')) :
        return th_error()

    global config

    try:
        with open('/home/vagrant/catkin_ws/src/cp_gazebo/instructions/' + config["start_loc"] + '_to_' + config["target_loc"] + '.json') as path_file:
            data = json.load(path_file)
            return action_result({ 'path' : data['path'] })
    except Exception as e:
        log_das_error(LogError.RUNTIME_ERROR, "error in reading the files for query_path: " + str(e))
        return th_error()

@app.route('/action/start', methods=['POST'])
def action_start():
    if (not isValidActionCall(request, '/action/start', 'POST')) :
        return th_error()

    global config
    global deadline

    print "starting challenge problem"
    try:
        ig_path = '/home/vagrant/catkin_ws/src/cp_gazebo/instructions/' + config["start_loc"] + '_to_' + config["target_loc"] + '.ig'
        igfile = open(ig_path, "r")
        igcode = igfile.read()
        # todo: when is it safe to close this file? does the 'with' pragma do this more cleanly?
        goal = ig_action_msgs.msg.InstructionGraphGoal(order=igcode)
        global client
        client.send_goal( goal = goal, done_cb = done_cb, active_cb = active_cb)

        # update the deadline to be now + the amount of time for the path given in the json file
        with open('/home/vagrant/catkin_ws/src/cp_gazebo/instructions/' + config["start_loc"] + '_to_' + config["target_loc"] + '.json') as config_file:
            data = json.load(config_file)
            deadline = datetime.datetime.now() + datetime.timedelta(seconds=data['time'])

    except Exception as e:
        log_das_error(LogError.RUNTIME_ERROR, "could not send the goal!: " + str(e))
        return th_error()

    return action_result({})  # todo: this includes time as well; is that out of spec?

@app.route('/action/observe', methods=['GET'])
def action_observe():
    if (not isValidActionCall(request, '/action/observe', 'GET')) :
        return th_error()

    global gazebo
    global deadline

    try:
        x, y, w , vel = gazebo.get_turtlebot_state()
        observation = {"x" : x, "y" : y, "w" : w,
                       "v" : vel ,
                       "voltage" : -1,  # todo: Need to work this out
                       "deadline" : deadline.isoformat()
                      }
        return action_result(observation)
    except Exception as e:
        log_das_error(LogError.RUNTIME_ERROR, "error in observe: " + str(e))
        return th_error()

@app.route('/action/set_battery', methods=['POST'])
def action_set_battery():
    if (not isValidActionCall(request, '/action/set_battery', 'POST')) :
        return th_error()

    if(request.headers['Content-Type'] != "application/json"):
        log_das_error(LogError.RUNTIME_ERROR, 'action/set_battery recieved post without json header')
        return th_error()

    params = request.get_json(silent=True)
    if (not ('ARGUMENTS' in params.keys())):
        log_das_error(LogError.RUNTIME_ERROR, 'action/set_battery got a post without ARGUMENTS')
        return th_error()

    if (not (isinstance(params['ARGUMENTS'], dict))):
        log_das_error(LogError.RUNTIME_ERROR, 'action/set_battery got a post where ARGUMENTS is not a dict')
        return th_error()

    if (not ('voltage' in params['ARGUMENTS'].keys())):
        log_das_error(LogError.RUNTIME_ERROR, 'action/set_battery got a post where ARGUMENTS doesnt give voltage')
        return th_error()

    if (int_out_of_range(params['ARGUMENTS']['voltage'],104,166)):
        log_das_error(LogError.RUNTIME_ERROR, 'action/set_battery got a post where ARGUMENTS gives voltage out of range')
        return th_error()

    ## todo : implement real stuff here when we have the battery
    ## model. also need to check that the argument voltage is less than the
    ## current voltage, not just a valid possible voltage?

    return action_result({})

@app.route('/action/place_obstacle', methods=['POST'])
def action_place_obstacle():
    if (not isValidActionCall(request, '/action/place_obstacle', 'POST')) :
        return th_error()

    if(request.headers['Content-Type'] != "application/json"):
        log_das_error(LogError.RUNTIME_ERROR, 'action/place_obstacle recieved post without json header')
        return th_error()

    # todo: is this right? params should not have x and y at the top level
    params = request.get_json(silent=True)
    if (not ('ARGUMENTS' in params.keys())):
        log_das_error(LogError.RUNTIME_ERROR, 'action/place_obstacle got a post without ARGUMENTS')
        return th_error()

    if (not (isinstance(params['ARGUMENTS'], dict))):
        log_das_error(LogError.RUNTIME_ERROR, 'action/place_obstacle got a post where ARGUMENTS is not a dict')
        return th_error()

    if (not ('x' in  params['ARGUMENTS'].keys() and 'y' in params['ARGUMENTS'].keys())) :
        log_das_error(LogError.RUNTIME_ERROR, 'action/place_obstacle got a post without both and x and y')
        return th_error()

    global gazebo

    obs_name = gazebo.place_new_obstacle(params['ARGUMENTS']["x"], params['ARGUMENTS']["y"])
    if obs_name is not None:
        ARGUMENTS = {"obstacle_id" : obs_name};
        return action_result(ARGUMENTS)
    else:
        log_das_error(LogError.RUNTIME_ERROR, 'gazebo cant place new obstacle at given x y')
        return th_error()

@app.route('/action/remove_obstacle', methods=['POST'])
def action_remove_obstacle():
    if (not isValidActionCall(request, '/action/remove_obstacle', 'POST')) :
        return th_error()

    if( request.headers['Content-Type'] != "application/json"):
        log_das_error(LogError.RUNTIME_ERROR, 'action_remove_obstacle recieved post without json header')
        return th_error()

    params = request.get_json(silent=True)
    if (not ('ARGUMENTS' in params.keys())):
        log_das_error(LogError.RUNTIME_ERROR, 'action/remove_obstacle got a post without ARGUMENTS')
        return th_error()

    if (not (isinstance(params['ARGUMENTS'], dict))):
        log_das_error(LogError.RUNTIME_ERROR, 'action/remove_obstacle got a post where ARGUMENTS is not a dict')
        return th_error()

    if (not 'obstacle_id' in params['ARGUMENTS'].keys()) :
        log_das_error(LogError.RUNTIME_ERROR, 'action/remove_obstacle recieved post with bogus obstacle id')
        return th_error()

    global gazebo
    success = gazebo.delete_obstacle(params['ARGUMENTS']["obstacle_id"])
    if success:
        return action_result({})
    else:
        # todo: implicitly, this is because it was a bad obstacle ID. can we confirm that?
        log_das_error(LogError.RUNTIME_ERROR, 'action/remove_obstacle gazebo call failed')
        return th_error()

@app.route('/action/perturb_sensor', methods=['POST'])
def action_perturb_sensor():
    if (not isValidActionCall(request, '/action/perturb_sensor', 'POST')) :
        return th_error()

    if(request.headers['Content-Type'] != "application/json"):
        log_das_error(LogError.RUNTIME_ERROR, '/action/perturb_sensor recieved post without json header')
        return th_error()

    params = request.get_json(silent=True)
    if (not ('ARGUMENTS' in params.keys())):
        log_das_error(LogError.RUNTIME_ERROR, '/action/perturb_sensor got a post without ARGUMENTS')
        return th_error()

    if (not (isinstance(params['ARGUMENTS'], dict))):
        log_das_error(LogError.RUNTIME_ERROR, '/action/perturb_sensor got a post where ARGUMENTS is not a dict')
        return th_error()

    if(not ('bump' in params['ARGUMENTS'].keys())):
        log_das_error(LogError.RUNTIME_ERROR, '/action/perturb_sensor recieved post without bump in the JSON object')
        return th_error()

    if(not (isinstance(params['ARGUMENTS']['bump'], dict))):
        log_das_error(LogError.RUNTIME_ERROR, '/action/perturb_sensor recieved post with bump, but not bound to a dict')
        return th_error()

    if (not (set(['x', 'y', 'z', 'p', 'w', 'r']).issubset(params['ARGUMENTS']['bump'].keys()))):
        log_das_error(LogError.RUNTIME_ERROR, '/action/perturb_sensor post with bump bound to a dict but missing one of the six fields')
        return th_error()

    if(int_out_of_range(params['ARGUMENTS']['bump']['x'], -3, 3)):
        log_das_error(LogError.RUNTIME_ERROR, '/action/perturb_sensor post with out of range x')
        return th_error()
    if(int_out_of_range(params['ARGUMENTS']['bump']['y'], -3, 3)):
        log_das_error(LogError.RUNTIME_ERROR, '/action/perturb_sensor post with out of range y')
        return th_error()
    if(int_out_of_range(params['ARGUMENTS']['bump']['z'], -3, 3)):
        log_das_error(LogError.RUNTIME_ERROR, '/action/perturb_sensor post with out of range z')
        return th_error()
    if(int_out_of_range(params['ARGUMENTS']['bump']['p'], -6, 6)):
        log_das_error(LogError.RUNTIME_ERROR, '/action/perturb_sensor post with out of range p')
        return th_error()
    if(int_out_of_range(params['ARGUMENTS']['bump']['w'], -6, 6)):
        log_das_error(LogError.RUNTIME_ERROR, '/action/perturb_sensor post with out of range w')
        return th_error()
    if(int_out_of_range(params['ARGUMENTS']['bump']['r'], -6 , 6)):
        log_das_error(LogError.RUNTIME_ERROR, '/action/perturb_sensor post with out of range r')
        return th_error()

    ## todo: currently we have no sensor to bump, so this doesn't do
    ## anything other than check the format of the request and reply with
    ## something well-formatted if it gets something well-formatted
    return action_result({})


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

    # make an interface into Gazebo
    gazebo = GazeboInterface()

    # parse the config file
    # todo: this posts errors to the TH, but we should stop the world when that happens
    config = parse_config_file()

    # this should block until the navigation stack is ready to recieve goals
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    move_base.wait_for_server()

    ## todo: call bradley's stuff to teleport the robot to the place it's actully starting not l1
    ## todo: this posts errors to the TH, but we should stop the world when that happens
    ## todo: this may happen too early
    das_ready()

    ## actually start up the flask service. this never returns, so it must
    ## be the last thing in the file
    app.run (host="0.0.0.0")
