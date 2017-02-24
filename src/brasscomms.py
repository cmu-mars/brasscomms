#! /usr/bin/env python

""" defines and implements the the TA RESTful interface """

### standard imports
from __future__ import with_statement
from threading import Lock

from os.path import exists, isfile
from os import access, R_OK
import json
import datetime
import requests

### relevant third party imports
from flask import Flask, Response, request

import rospy
import actionlib
import ig_action_msgs.msg
from move_base_msgs.msg import MoveBaseAction

### other brasscomms modules
from constants import (TH_URL, CONFIG_FILE_PATH, LOG_FILE_PATH, CP_GAZ,
                       JSON_MIME, Error, LogError, QUERY_PATH, Status,
                       START, OBSERVE, SET_BATTERY, PLACE_OBSTACLE,
                       REMOVE_OBSTACLE, PERTURB_SENSOR, DoneEarly,
                       AdaptationLevels, INTERNAL_STATUS, SubSystem,
                       TIME_FORMAT)
from gazebo_interface import GazeboInterface
from rainbow_interface import RainbowInterface
from map_util import waypoint_to_coords
from parse import (Coords, Bump, Config, TestAction,
                   Voltage, ObstacleID, SingleBumpName,
                   InternalStatus)

### some definitions and helper functions

def done_cb(terminal, result):
    """ callback for when the bot is at the target """
    if result:
        done_early("done_cb called with terminal %d and positive result %s" % (terminal, result),
                   DoneEarly.AT_TARGET)
        log_das(LogError.INFO,
                "done_cb called by ros with terminal %d and positive result %s" % (terminal, result))
    else:
        das_status(Status.TEST_ERROR,
                   "done_cb with terminal %d but with negative result %s; this is an error" % (terminal, result))

    ## todo: when we have the battery model implemented, also check here to
    ## see if the battery is empty and send that message instead

def active_cb():
    """ callback for when the bot is made active """
    log_das(LogError.INFO, "brasscoms received notification that goal is active")

### some globals
app = Flask(__name__)
deadline = None ## this is a default value; the result of observe will be
                ## well formed but wrong unless they call start first

## shared_var_lock = Lock() ## todo :commented out until we have occasion to use it

def parse_config_file():
    """ checks the appropriate place for the config file, and loads into an object if possible """
    if exists(CONFIG_FILE_PATH) and isfile(CONFIG_FILE_PATH) and access(CONFIG_FILE_PATH, R_OK):
        with open(CONFIG_FILE_PATH) as config_file:
            data = json.load(config_file)
            conf = Config(**data)
            return conf
    else:
        # todo: does sending this this sufficiently stop the world if
        # the file doesn't parse?  todo: return something?
        th_das_error(Error.TEST_DATA_FILE_ERROR,
                     '%s does not exist, is not a file, or is not readable' % CONFIG_FILE_PATH)

### subroutines for forming API results
def th_error():
    """ the response for a TH error """
    return Response(status=400)

def action_result(body):
    """ given a body, produce the action result with headers """
    return Response(json.dumps({"TIME" : timenow(),
                                "RESULT": body}),
                    status=200, mimetype=JSON_MIME)

### subroutines for forming and sending messages to the TH
def th_das_error(err, msg):
    """ posts a DAS_ERROR formed with the arguments """
    dest = TH_URL + "/error"
    error_contents = {"TIME" : timenow(),
                      "ERROR" : err.name,
                      "MESSAGE" : msg}
    try:
        requests.post(dest, data=json.dumps(error_contents))
    except Exception as e:
        log_das(LogError.RUNTIME_ERROR, "Fatal: cannot connect to TH at %s: %s" % (dest, e))

def log_das(error, msg):
    """ formats the arguments per the API and inserts them to the log """
    try:
        with open(LOG_FILE_PATH, 'a') as log_file:
            error_contents = {"TIME" : timenow(),
                              "TYPE" : error.name,
                              "MESSAGE" : msg}
            data = json.dumps(error_contents)
            log_file.write(data + "\n")
    except StandardError as e:
        th_das_error(Error.DAS_LOG_FILE_ERROR, '%s could not be accessed: %s' % (LOG_FILE_PATH, e))

def done_early(message, reason):
    """ POSTs action message to the TH that we're done early """
    dest = TH_URL + "/action/done"
    contents = {"TIME" : timenow(),
                "TARGET" : message,
                "ARGUMENTS" : {"done" : reason.name}}
    log_das(LogError.INFO, "ending early: %s; %s" % (reason.name, message))

    try:
        requests.post(dest, data=json.dumps(contents))
    except Exception as e:
        log_das(LogError.RUNTIME_ERROR,
                "Fatal: couldn't connect to TH to indicate early termination at %s: %s" % (dest, e))

def das_ready():
    """ POSTs DAS_READY to the TH, or logs if failed"""
    dest = TH_URL + "/ready"
    contents = {"TIME" : timenow()}
    try:
        requests.post(dest, data=json.dumps(contents))
    except Exception as e:
        log_das(LogError.STARTUP_ERROR,
                "Fatal: couldn't connect to TH to send DAS_READY at %s: %s" % (dest, e))

def das_status(status, message):
    dest = TH_URL + "/action/status"
    contents = {"TIME" : timenow(),
                "STATUS": status.name,
                "MESSAGE": message}
    try:
        requests.post(dest, data=json.dumps(contents))
    except Exception as e:
        log_das(LogError.RUNTIME_ERROR,
                "Fatal: couldn't connect to TH to send DAS_STATUS at %s: %s" % (dest, e))


STATE_LOCK = Lock()
READY_LIST = []

def indicate_ready(subsystem):
    ready = False
    global config
    with STATE_LOCK:
        READY_LIST.append(subsystem)
        if config.enable_adaptation == AdaptationLevels.CP1_NoAdaptation or config.enable_adaptation == AdaptationLevels.CP2_NoAdaptation:
            ready = SubSystem.BASE in READY_LIST
        else:
            ready = SubSystem.BASE in READY_LIST and SubSystem.DAS in READY_LIST
            if not ready and SubSystem.DAS in READY_LIST:
                log_das(LogError.INFO, 'DAS is ready before base - this should not happen')
    if ready:
        das_ready()

def check_action(req, path, methods):
    """ return true if the request respects the methods, false and log it otherwise """

    # check that the method is running from actually getting hit
    if req.path != path:
        log_das(LogError.RUNTIME_ERROR, 'internal fault: %s called improperly' % path)
        return False

    # check that it's being called in a way it's designed for
    if not request.method in methods:
        log_das(LogError.RUNTIME_ERROR,
                '%s called with bad HTTP request: %s not in %s' % (path, req.method, methods))
        return False

    # if it's a post, make sure that it got JSON. req.method also needs to
    # be in methods, but that must be true from above
    if (req.method == 'POST') and (JSON_MIME not in request.headers['Content-Type']):
        log_das(LogError.RUNTIME_ERROR, '%s POSTed to without json header: %s' % (path, request.headers['Content-Type']))
        return False

    return True

def instruct(ext):
    """ given an extension, provides the path to the config-relevant file in instructions """
    global config

    return CP_GAZ + '/instructions/' + config.start_loc + '_to_' + config.target_loc + ext

def timestr(d):
    """ format the argument time to MIT's spec """

    ## the spec wants exactly 3 decimal places worth of fractions of a
    ## second. the ISO standard is 6, so we chop off the trailing 3 and
    ## then glue on the Z that they want at the end.
    return '%sZ' % d.strftime(TIME_FORMAT)[:-3]

def timenow():
    """ return the UTC now time, formatted to MIT's spec.  """
    return timestr(datetime.datetime.utcnow())

### subroutines per endpoint URL in API wiki page order
@app.route(QUERY_PATH.url, methods=QUERY_PATH.methods)
def action_query_path():
    """ implements query path end point """
    if not check_action(request, QUERY_PATH.url, QUERY_PATH.methods):
        return th_error()

    try:
        with open(instruct('.json')) as path_file:
            data = json.load(path_file)
            return action_result({'path' : data['path'], 'time' : data['time']})
    except Exception as e:
        log_das(LogError.RUNTIME_ERROR,
                "error in reading the files for %s: %s " % (QUERY_PATH.url, e))
        return th_error()

@app.route(START.url, methods=START.methods)
def action_start():
    """ implements start end point """
    if not check_action(request, START.url, START.methods):
        return th_error()

    try:
        j = request.get_json(silent=True)
        params = TestAction(**j)
    except Exception as e:
        log_das(LogError.RUNTIME_ERROR,
                '%s got a malformed test action POST: %s' % (START.url, e))
        return th_error()

    global deadline

    ## check to see if there's already an assigned goal, abort if so.
    global client
    if client.gh:
        log_das(LogError.RUNTIME_ERROR, "%s hit with an already active goal" % START.url)
        return th_error()

    log_das(LogError.INFO, "starting challenge problem")
    try:
        with open(instruct('.ig')) as igfile:
            igcode = igfile.read()
            goal = ig_action_msgs.msg.InstructionGraphGoal(order=igcode)
            client.send_goal(goal=goal, done_cb=done_cb, active_cb=active_cb)

        # update the deadline to be now + the amount of time for the path
        # given in the json file
        with open(instruct('.json')) as config_file:
            data = json.load(config_file)
            deadline = timestr(datetime.datetime.utcnow() + datetime.timedelta(seconds=data['time']))
    except Exception as e:
        log_das(LogError.RUNTIME_ERROR, "could not send the goal in %s: %s " % (START.url, e))
        return th_error()

    return action_result({})

@app.route(OBSERVE.url, methods=OBSERVE.methods)
def action_observe():
    """ implements observe end point """
    if not check_action(request, OBSERVE.url, OBSERVE.methods):
        return th_error()

    global gazebo
    global deadline

    try:
        x, y, w, vel = gazebo.get_turtlebot_state()
        observation = {"x" : x, "y" : y, "w" : w,
                       "v" : vel,
                       "voltage" : -1,  # todo: Need to work this out
                       "deadline" : str(deadline)
                      }
        return action_result(observation)
    except Exception as e:
        log_das(LogError.RUNTIME_ERROR, "error in %s: %s " % (OBSERVE.url, e))
        return th_error()

@app.route(SET_BATTERY.url, methods=SET_BATTERY.methods)
def action_set_battery():
    """ implements set_battery end point """
    if not check_action(request, SET_BATTERY.url, SET_BATTERY.methods):
        return th_error()

    try:
        j = request.get_json(silent=True)
        params = TestAction(**j)
        params.ARGUMENTS = Voltage(**params.ARGUMENTS)
    except Exception as e:
        log_das(LogError.RUNTIME_ERROR,
                '%s got a malformed test action POST: %s' % (SET_BATTERY.url, e))
        return th_error()

    ## todo : implement real stuff here when we have the battery
    ## model. also need to check that the argument voltage is less than the
    ## current voltage, not just a valid possible voltage?

    ## todo: register a callback with the battery sim here to hit
    ## done_early in case we run out. alt: we may need to subscribe to
    ## the topic below.

    return action_result({})

@app.route(PLACE_OBSTACLE.url, methods=PLACE_OBSTACLE.methods)
def action_place_obstacle():
    """ implements place_obstacle end point """
    if not check_action(request, PLACE_OBSTACLE.url, PLACE_OBSTACLE.methods):
        return th_error()

    try:
        j = request.get_json(silent=True)
        params = TestAction(**j)
        params.ARGUMENTS = Coords(**params.ARGUMENTS)
    except Exception as e:
        log_das(LogError.RUNTIME_ERROR,
                '%s got a malformed test action POST: %s' % (PLACE_OBSTACLE.url, e))
        return th_error()

    global gazebo

    obs_name = gazebo.place_new_obstacle(params.ARGUMENTS.x, params.ARGUMENTS.y)
    if obs_name is not None:
        return action_result({"obstacleid" : obs_name,
                              "topleft_x" : params.ARGUMENTS.x - 1.2,
                              "topleft_y" : params.ARGUMENTS.y - 1.2,
                              "botright_x" : params.ARGUMENTS.x + 1.2,
                              "botright_y" : params.ARGUMENTS.x + 1.2})
    else:
        log_das(LogError.RUNTIME_ERROR, 'gazebo cant place new obstacle at given x y')
        return th_error()

@app.route(REMOVE_OBSTACLE.url, methods=REMOVE_OBSTACLE.methods)
def action_remove_obstacle():
    """ implements remove_obstacle end point """
    if not check_action(request, REMOVE_OBSTACLE.url, methods=REMOVE_OBSTACLE.methods):
        return th_error()

    try:
        j = request.get_json(silent=True)
        params = TestAction(**j)
        params.ARGUMENTS = ObstacleID(**params.ARGUMENTS)
    except Exception as e:
        log_das(LogError.RUNTIME_ERROR,
                '%s got a malformed test action POST: %s' % (REMOVE_OBSTACLE.url, e))
        return th_error()

    try:
        global gazebo
        ## todo: this breaks for slightly mysterious reasons
        success = gazebo.delete_obstacle(params.ARGUMENTS.obstacleid)
        if success:
            return action_result({})
        else:
            log_das(LogError.RUNTIME_ERROR, '%s gazebo call failed' % REMOVE_OBSTACLE.url)
            return th_error()
    except Exception as e:
        log_das(LogError.RUNTIME_ERROR, '%s raised an exception: %s' % (REMOVE_OBSTACLE.url, e))
        return th_error()



@app.route(PERTURB_SENSOR.url, methods=PERTURB_SENSOR.methods)
def action_perturb_sensor():
    """ implements perturb_sensor end point """
    if not check_action(request, PERTURB_SENSOR.url, methods=PERTURB_SENSOR.methods):
        return th_error()

    try:
        j = request.get_json(silent=True)
        params = TestAction(**j)
        params.ARGUMENTS = SingleBumpName(**params.ARGUMENTS)
        params.ARGUMENTS.bump = Bump(**params.ARGUMENTS.bump)
    except Exception as e:
        log_das(LogError.RUNTIME_ERROR,
                '%s got a malformed test action POST: %s' % (PERTURB_SENSOR.url, e))
        return th_error()

    ## todo: currently we have no sensor to bump, so this doesn't do
    ## anything other than check the format of the request and reply with
    ## something well-formatted if it gets something well-formatted
    return action_result({})

@app.route(INTERNAL_STATUS.url, methods=INTERNAL_STATUS.methods)
def internal_status():
    """ implements the internal status, for communication from Rainbow """
    if not check_action(request, INTERNAL_STATUS.url, methods=INTERNAL_STATUS.methods):
        return th_error()

    try:
        j = request.get_json(silent=True)
        params = InternalStatus(**j)

        if params.STATUS == "RAINBOW_READY":
            # Rainbow is now ready to, so send das_ready()
            indicate_ready(SubSystem.DAS)
        else:
            das_status(filter(lambda x: x.name == params.STATUS, Status)[0],
                       params.MESSAGE)
    except IndexError as e:
        log_das(LogError.RUNTIME_ERROR,
                '%s got a POST with the unknown status name \'%s\', %s'
                % (INTERNAL_STATUS.url, params.STATUS, e))
    except Exception as e:
        log_das(LogError.RUNTIME_ERROR,
                '%s got a malformed internal status: %s' %(INTERNAL_STATUS.url, e))
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
    client = actionlib.SimpleActionClient("ig_action_server",
                                          ig_action_msgs.msg.InstructionGraphAction)
    client.wait_for_server()

    # make an interface into Gazebo
    try:
        gazebo = GazeboInterface()
    except Exception as e:
        log_das(LogError.STARTUP_ERROR, "Fatal: gazebo did not start up: %s" % e)
        th_das_error(Error.DAS_OTHER_ERROR, "Fatal: gazebo did not start up: %s" % e)
        raise

    # parse the config file
    try:
        config = parse_config_file()
    except Exception as e:
        log_das(LogError.STARTUP_ERROR, "Fatal: config file doesn't parse: %s" % e)
        th_das_error(Error.DAS_OTHER_ERROR, "Fatal: config file doesn't parse: %s" % e)
        raise

    # this should block until the navigation stack is ready to recieve goals
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    move_base.wait_for_server()

    # arrange the bot in the location specified by the config
    try:
        start_coords = waypoint_to_coords(config.start_loc)
        gazebo.set_turtlebot_position(start_coords['x'], start_coords['y'], config.start_yaw)
    except Exception as e:
        log_das(LogError.STARTUP_ERROR,
                "Fatal: config file inconsistent with map: %s" % e)
        th_das_error(Error.DAS_OTHER_ERROR, "Fatal: config file inconsistent with map: %s" % e)
        raise

    # start Rainbow
    try:
        rainbow_log = open("/test/rainbow.log", 'w')
        rainbow = RainbowInterface()
        rainbow.launchRainbow(config.enable_adaptation, rainbow_log)
        rainbow.startRainbow()
    except Exception as e:
        log_das(LogError.STARTUP_ERROR, "Fatal: config file inconsistent with map: %s" % e)
        th_das_error(Error.DAS_OTHER_ERROR, "Fatal: rainbow failed to start: %s" % e)
        raise

    ## todo: this may happen too early
    indicate_ready(SubSystem.BASE)

    ## actually start up the flask service. this never returns, so it must
    ## be the last thing in the file
    app.run(host="0.0.0.0")
