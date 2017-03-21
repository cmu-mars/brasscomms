#! /usr/bin/env python

""" defines and implements the the TA RESTful interface """

### standard imports
from __future__ import with_statement
from threading import Lock

from os.path import exists, isfile
from os import access, R_OK
import json
import datetime
import subprocess
import math
import time
import requests

### relevant third party imports
from flask import Flask, Response, request

import rospy
import actionlib
import ig_action_msgs.msg
from move_base_msgs.msg import MoveBaseAction
from std_msgs.msg       import (Int32, Bool, Float32MultiArray, Int32MultiArray,
                                MultiArrayLayout, MultiArrayDimension)
from kobuki_msgs.msg    import MotorPower
from mars_notifications.msg import UserNotification
from ig_action_msgs.msg import InstructionGraphResult

### other brasscomms modules
from constants import (TH_URL, CONFIG_FILE_PATH, LOG_FILE_PATH, CP_GAZ,
                       JSON_MIME, Error, LogError, QUERY_PATH, Status,
                       START, OBSERVE, SET_BATTERY, PLACE_OBSTACLE,
                       REMOVE_OBSTACLE, PERTURB_SENSOR, DoneEarly,
                       AdaptationLevels, INTERNAL_STATUS, SubSystem,
                       TIME_FORMAT, BINDIR, JSON_HEADER)
from gazebo_interface import GazeboInterface
from rainbow_interface import RainbowInterface
from map_util import waypoint_to_coords
from parse import (Coords, Bump, Config, TestAction,
                   Voltage, ObstacleID, SingleBumpName,
                   InternalStatus, ISMessage)

### some definitions and helper functions

def notify_cb(msg):
    """ callback to respond to any other components publishing a new deadline """
    global deadline
    try:
        deadline = int(msg.new_deadline)
    except ValueError:
        deadline = int(float(msg.new_deadline))
    except Exception as e:
        log_das(LogError.RUNTIME_ERROR, "malformed deadline message: %s" % e)
        th_das_error(Error.DAS_OTHER_ERROR, "internal fault: got a malformed deadline message: %s" % e)

def energy_cb(msg):
    """call back to update the global battery state from the ros topic"""
    global battery
    battery = msg.data

def motor_power_cb(msg):
    """call back for when battery runs out of power. we assume posting this to
       the TH will end the test soon. unsubscribes itself after one OFF
       message, to keep log size and message count down
    """
    rospy.loginfo("Received message from motor power: %s" %msg)
    if msg.state == MotorPower.OFF:
        done_early("energy_monitor indicated that the battery is empty", DoneEarly.BATTERY)
        ## if we see the message we want, we only need to see it once, so
        ## we unsubscribe
        global sub_motorpow
        sub_motorpow.unregister()

def done_cb(terminal, result):
    """ callback for when the bot is at the target """
    global client
    print "--------- result"
    print str(result)
    print
    if not_adapting() and 'successfully' in result.sequence:
        done_early("done_cb called with terminal %d and result %s" % (terminal, result),
                    DoneEarly.AT_TARGET)

def active_cb():
    """ callback for when the bot is made active """
    log_das(LogError.INFO, "brasscoms received notification that goal is active")

### some globals
app = Flask(__name__)
battery = -1
desired_volts = -1
desired_bump = None
deadline = -1 ## sim time default value

def parse_config_file():
    """ checks the appropriate place for the config file, and loads into an object if possible """
    if exists(CONFIG_FILE_PATH) and isfile(CONFIG_FILE_PATH) and access(CONFIG_FILE_PATH, R_OK):
        with open(CONFIG_FILE_PATH) as config_file:
            data = json.load(config_file)
            conf = Config(**data)
            return conf
    else:
        th_das_error(Error.TEST_DATA_FILE_ERROR,
                     '%s does not exist, is not a file, or is not readable' % CONFIG_FILE_PATH)
        raise ValueError("config file doesn't exist, isn't a file, or isn't readable")

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

    log_das(LogError.INFO, "posting error %s with message %s" % (err.name, msg))
    try:
        requests.post(dest, data=json.dumps(error_contents), headers=JSON_HEADER)
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
                "ARGUMENTS" : {"done" : reason.name,
                               "sim_time" : str(rospy.Time.now().secs)}}
    log_das(LogError.INFO, "ending early: %s; %s" % (reason.name, message))

    try:
        requests.post(dest, data=json.dumps(contents), headers=JSON_HEADER)
    except Exception as e:
        log_das(LogError.RUNTIME_ERROR,
                "Fatal: couldn't connect to TH to indicate early termination at %s: %s" % (dest, e))

def das_ready():
    """ POSTs DAS_READY to the TH, or logs if failed"""
    dest = TH_URL + "/ready"
    contents = {"TIME" : timenow()}

    log_das(LogError.INFO, "posting das_ready")
    try:
        requests.post(dest, data=json.dumps(contents), headers=JSON_HEADER)
    except Exception as e:
        log_das(LogError.STARTUP_ERROR,
                "Fatal: couldn't connect to TH to send DAS_READY at %s: %s" % (dest, e))

def das_status(status, message):
    dest = TH_URL + "/action/status"
    contents = {"TIME" : timenow(),
                "STATUS": status.name,
                "MESSAGE": {"msg" : message,
                            "sim_time" : str(rospy.Time.now().secs)}}

    log_das(LogError.INFO, "posting status %s with message %s" % (status.name, message))
    try:
        requests.post(dest, data=json.dumps(contents), headers=JSON_HEADER)
    except Exception as e:
        log_das(LogError.RUNTIME_ERROR,
                "Fatal: couldn't connect to TH to send DAS_STATUS at %s: %s" % (dest, e))

STATE_LOCK = Lock()
READY_LIST = []

def indicate_ready(subsystem):
    """ asynchronously waites for both the DAS and base system to come up before posting DAS ready """

    log_das(LogError.INFO, "indicate_ready called with %s" % subsystem.name)
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
    if not req.method in methods:
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

def resp2str(r):
    """given a Response object, produce a helpful string"""
    return "response %s; status %s; headers %s; data %s" % (r, r.status, r.headers, r.get_data())

def timestr(d):
    """ format the argument time to MIT's spec """

    ## the spec wants exactly 3 decimal places worth of fractions of a
    ## second. the ISO standard is 6, so we chop off the trailing 3 and
    ## then glue on the Z that they want at the end.
    return '%sZ' % d.strftime(TIME_FORMAT)[:-3]

def timenow():
    """ return the UTC now time, formatted to MIT's spec.  """
    return timestr(datetime.datetime.utcnow())

def in_cp1():
    """ return true iff we're in either CP1 mode """
    global config
    if config.enable_adaptation == AdaptationLevels.CP1_NoAdaptation:
        return True
    if config.enable_adaptation == AdaptationLevels.CP1_Adaptation:
        return True
    return False

def in_cp2():
    """ return true iff we're in either CP2 mode """
    global config
    if config.enable_adaptation == AdaptationLevels.CP2_NoAdaptation:
        return True
    if config.enable_adaptation == AdaptationLevels.CP2_Adaptation:
        return True
    return False

def not_adapting():
    global config
    if config.enable_adaptation == AdaptationLevels.CP2_NoAdaptation:
        return True
    if config.enable_adaptation == AdaptationLevels.CP1_NoAdaptation:
        return True
    return False

### subroutines per endpoint URL in API wiki page order
@app.route(QUERY_PATH.url, methods=QUERY_PATH.methods)
def action_query_path():
    """ implements query path end point """
    if not check_action(request, QUERY_PATH.url, QUERY_PATH.methods):
        return th_error()

    log_das(LogError.INFO, "%s hit" % QUERY_PATH.url)

    try:
        with open(instruct('.json')) as path_file:
            data = json.load(path_file)
            res = action_result({'path' : data['path'], 'time' : data['time']})
            log_das(LogError.INFO, "%s returning %s" % (QUERY_PATH.url, resp2str(res)))
            return res
    except Exception as e:
        log_das(LogError.RUNTIME_ERROR,
                "error in reading the files for %s: %s " % (QUERY_PATH.url, e))
        return th_error()

@app.route(START.url, methods=START.methods)
def action_start():
    """ implements start end point """
    if not check_action(request, START.url, START.methods):
        return th_error()

    log_das(LogError.INFO, "%s hit with %s" % (START.url,request.get_json(silent=True)))

    try:
        j = request.get_json(silent=True)
        params = TestAction(**j)
    except Exception as e:
        log_das(LogError.RUNTIME_ERROR,
                '%s got a malformed test action POST: %s' % (START.url, e))
        return th_error()

    ## check to see if there's already an assigned goal, abort if so.
    global client
    if client.gh:
        log_das(LogError.RUNTIME_ERROR, "%s hit with an already active goal" % START.url)
        return th_error()

    global deadline
    global pub_user_notify
    global desired_volts
    global desired_bump
    global pub_setvoltage
    global pub_setcharging

    if config.enable_adaptation == AdaptationLevels.CP2_Adaptation:
        cw_log = open("/test/calibration_watcher.log", "w")
        cw_child = subprocess.Popen([BINDIR + "/calibration_watcher"],
                                    stdout=cw_log,
                                    stderr=cw_log,
                                    cwd="/home/vagrant/")

    if in_cp2() and (not bump_sensor(desired_bump)):
        log_das(LogError.RUNTIME_ERROR, "Fatal: could not set inital sensor pose in %s" % START.url)
        return th_error()

    try:
        pub_setcharging.publish(Bool(False))
        pub_setvoltage.publish(Int32(desired_volts))
    except Exception as e:
        log_das(LogError.RUNTIME_ERROR,
                '%s got an error trying to publish to set_voltage and set_charging: %s' % (START.url, e))
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
            deadline = int(data['time']) + rospy.Time.now().secs
            pub_user_notify.publish(UserNotification(new_deadline=str(deadline),
                                                     user_notification="initial deadline"))

    except Exception as e:
        log_das(LogError.RUNTIME_ERROR, "could not send the goal in %s: %s " % (START.url, e))
        return th_error()

    res = action_result({})
    log_das(LogError.INFO, "%s returning %s" % (START.url, resp2str(res)))
    return res

@app.route(OBSERVE.url, methods=OBSERVE.methods)
def action_observe():
    """ implements observe end point """
    if not check_action(request, OBSERVE.url, OBSERVE.methods):
        return th_error()

    log_das(LogError.INFO, "%s hit" % OBSERVE.url)

    global gazebo
    global deadline
    global battery

    try:
        x, y, w, vel = gazebo.get_turtlebot_state()
        observation = {"x" : x, "y" : y, "w" : w,
                       "v" : vel,
                       "voltage" : int(battery),
                       "deadline" : str(deadline),
                       "sim_time" : str(rospy.Time.now().secs)
                      }
        res = action_result(observation)
        log_das(LogError.INFO, "%s returning %s" % (OBSERVE.url, resp2str(res)))
        return res
    except Exception as e:
        log_das(LogError.RUNTIME_ERROR, "error in %s: %s " % (OBSERVE.url, e))
        return th_error()

@app.route(SET_BATTERY.url, methods=SET_BATTERY.methods)
def action_set_battery():
    """ implements set_battery end point """
    if not check_action(request, SET_BATTERY.url, SET_BATTERY.methods):
        return th_error()

    log_das(LogError.INFO, "%s hit with %s" % (SET_BATTERY.url, request.get_json(silent=True)))

    if in_cp2():
        log_das(LogError.RUNTIME_ERROR,
                '%s hit in CP2 when battery is not active' % SET_BATTERY.url)
        return th_error()

    try:
        j = request.get_json(silent=True)
        params = TestAction(**j)
        params.ARGUMENTS = Voltage(**params.ARGUMENTS)
    except Exception as e:
        log_das(LogError.RUNTIME_ERROR,
                '%s got a malformed test action POST: %s' % (SET_BATTERY.url, e))
        return th_error()

    ## write to the relevant topic
    global pub_setvoltage
    try:
        rospy.loginfo('Setting voltage to %s' %params.ARGUMENTS.voltage)
        pub_setvoltage.publish(Int32(params.ARGUMENTS.voltage))
    except Exception as e:
        log_das(LogError.RUNTIME_ERROR,
                '%s got an error trying to publish to set_voltage: %s' % (SET_BATTERY.url, e))
        return th_error()

    global desired_volts
    desired_volts = params.ARGUMENTS.voltage

    res = action_result({"sim_time" : str(rospy.Time.now().secs)})
    log_das(LogError.INFO, "%s returning %s" % (SET_BATTERY.url, resp2str(res)))
    return res

def place_obstacle(loc):
    """given a coodinate, places the obstacle there. returns true if this goes
       well, false otherwise."""
    global gazebo
    obs_name = gazebo.place_new_obstacle(loc.x, loc.y)
    return obs_name

@app.route(PLACE_OBSTACLE.url, methods=PLACE_OBSTACLE.methods)
def action_place_obstacle():
    """ implements place_obstacle end point """
    if not check_action(request, PLACE_OBSTACLE.url, PLACE_OBSTACLE.methods):
        return th_error()

    log_das(LogError.INFO, "%s hit with %s" % (PLACE_OBSTACLE.url, request.get_json(silent=True)))

    if in_cp2():
        log_das(LogError.RUNTIME_ERROR,
                '%s hit in CP2 when obstacles are not active' % PLACE_OBSTACLE.url)
        return th_error()

    try:
        j = request.get_json(silent=True)
        params = TestAction(**j)
        params.ARGUMENTS = Coords(**params.ARGUMENTS)
    except Exception as e:
        log_das(LogError.RUNTIME_ERROR,
                '%s got a malformed test action POST: %s' % (PLACE_OBSTACLE.url, e))
        return th_error()

    obs_name = place_obstacle(params.ARGUMENTS)
    if obs_name is not None:
        res = action_result({"obstacleid" : obs_name,
                              "topleft_x" :  params.ARGUMENTS.x - 1.2,
                              "topleft_y" :  params.ARGUMENTS.y - 1.2,
                              "botright_x" : params.ARGUMENTS.x + 1.2,
                              "botright_y" : params.ARGUMENTS.y + 1.2,
                              "sim_time" : str(rospy.Time.now().secs)})
        log_das(LogError.INFO, "%s returning %s" % (PLACE_OBSTACLE.url, resp2str(res)))
        return res
    else:
        log_das(LogError.RUNTIME_ERROR, 'gazebo cant place new obstacle at given x y')
        return th_error()

@app.route(REMOVE_OBSTACLE.url, methods=REMOVE_OBSTACLE.methods)
def action_remove_obstacle():
    """ implements remove_obstacle end point """
    if not check_action(request, REMOVE_OBSTACLE.url, methods=REMOVE_OBSTACLE.methods):
        return th_error()

    log_das(LogError.INFO, "%s hit with %s" % (REMOVE_OBSTACLE.url, request.get_json(silent=True)))

    if in_cp2():
        log_das(LogError.RUNTIME_ERROR,
                '%s hit in CP2 when obstacles are not active' % REMOVE_OBSTACLE.url)
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
        success = gazebo.delete_obstacle(params.ARGUMENTS.obstacleid)
        if success:
            res = action_result({"sim_time" : str(rospy.Time.now().secs)})
            log_das(LogError.INFO, "%s hit with %s" % (REMOVE_OBSTACLE.url, res))
            return res
        else:
            log_das(LogError.RUNTIME_ERROR, '%s gazebo call failed' % REMOVE_OBSTACLE.url)
            return th_error()
    except Exception as e:
        log_das(LogError.RUNTIME_ERROR, '%s raised an exception: %s' % (REMOVE_OBSTACLE.url, e))
        return th_error()

def call_set_joint(name, args, trans):
    """given the name of a binary, arguments, and a transform on those args,
        call it and look for errors. returns False if an error was
        encountered, True otherwise.
    """
    try:
        call = [BINDIR + name] + (map(trans, args))
        print "calling %s as %s" % (name, call)
        ret = subprocess.call(call)
        print "%s returned" % name

        if ret > 0:
            log_das(LogError.RUNTIME_ERROR,
                    '%s had non-zero return %d from calling %s'
                    % (PERTURB_SENSOR.url, ret, name))
            return False
    except Exception as e:
        log_das(LogError.RUNTIME_ERROR,
                '%s caught exception when calling %s: %s'
                % (PERTURB_SENSOR.url, name, e))
        return False
    return True

def bump_sensor(bump):
    """given a bump object, bumps the sensor accordingly. returns true if this
       goes well and false otherwise so that errors can be propagated as
       appropriate from the call site """
    if not (call_set_joint("/set_joint_rot", [bump.r, bump.p, bump.w],
                           lambda x: str(math.radians(x * 10)))):
        return False
    if not (call_set_joint("/set_joint_trans", [bump.x, bump.y, bump.z],
                           lambda x: str(x * 0.05))):
        return False

    global pub_perturb
    pub_perturb.publish(
        Int32MultiArray(layout=
                        MultiArrayLayout(dim=[MultiArrayDimension(label="bump",
                                                                 size=6,
                                                                 stride=0)],
                                         data_offset=0),
                        data=[bump.r, bump.p, bump.w,
                              bump.x, bump.y, bump.z]))
    return True

@app.route(PERTURB_SENSOR.url, methods=PERTURB_SENSOR.methods)
def action_perturb_sensor():
    """ implements perturb_sensor end point """
    if not check_action(request, PERTURB_SENSOR.url, methods=PERTURB_SENSOR.methods):
        return th_error()

    log_das(LogError.INFO, "%s hit with %s" % (PERTURB_SENSOR.url, request.get_json(silent=True)))

    if in_cp1():
        log_das(LogError.RUNTIME_ERROR,
                '%s hit in CP1 when the kinect is not active' % PERTURB_SENSOR.url)
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

    global desired_bump
    desired_bump = params.ARGUMENTS.bump

    ## rotate the joint, converting intervals of degrees to radians
    if not bump_sensor(params.ARGUMENTS.bump):
        return th_error()
    else:
        res = action_result({"sim_time" : str(rospy.Time.now().secs)})
        log_das(LogError.INFO, "%s returning %s" % (PERTURB_SENSOR.url, resp2str(res)))
        return res

@app.route(INTERNAL_STATUS.url, methods=INTERNAL_STATUS.methods)
def internal_status():
    """ implements the internal status, for communication from Rainbow """
    if not check_action(request, INTERNAL_STATUS.url, methods=INTERNAL_STATUS.methods):
        return th_error()

    log_das(LogError.INFO, "%s hit with %s" % (INTERNAL_STATUS.url, request.get_json(silent=True)))

    try:
        j = request.get_json(silent=True)
        params = InternalStatus(**j)
        params.MESSAGE = ISMessage(**params.MESSAGE)

        if params.STATUS == "RAINBOW_READY":
            # Rainbow is now ready to, so send das_ready()
            indicate_ready(SubSystem.DAS)
        elif params.STATUS == "MISSION_COMPLETED":
	    done_early(params.MESSAGE.msg, DoneEarly.AT_TARGET)
        else:
            das_status(filter(lambda x: x.name == params.STATUS, Status)[0],
                       params.MESSAGE.msg)
    except IndexError as e:
        log_das(LogError.RUNTIME_ERROR,
                '%s got a POST with the unknown status name \'%s\', %s'
                % (INTERNAL_STATUS.url, params.STATUS, e))
    except Exception as e:
        log_das(LogError.RUNTIME_ERROR,
                '%s got a malformed internal status: %s' %(INTERNAL_STATUS.url, e))

    res = action_result({})
    log_das(LogError.INFO, "%s returning %s" % (INTERNAL_STATUS.url, resp2str(res)))
    return res


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

    desired_volts = config.initial_voltage
    desired_bump = config.sensor_perturbation

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
	ok = rainbow.startRainbow()
        if not ok:
            raise Exception("Did not connect to Rainbow in a timely fashion")
    except Exception as e:
        log_das(LogError.STARTUP_ERROR, "Fatal: Rainbow failed to start: %s" % e)
        th_das_error(Error.DAS_OTHER_ERROR, "Fatal: rainbow failed to start: %s" % e)
        raise Exception("start up error")

    ## subscribe to the energy_monitor topics and make publishers
    pub_setcharging = rospy.Publisher("/energy_monitor/set_charging", Bool, queue_size=10)
    pub_setvoltage = rospy.Publisher("/energy_monitor/set_voltage", Int32, queue_size=10)
    pub_user_notify = rospy.Publisher("notify_user", UserNotification, queue_size=10)
    pub_perturb = rospy.Publisher("/calibration/perturb", Int32MultiArray, queue_size=10)

    sub_voltage = rospy.Subscriber("/energy_monitor/voltage", Int32, energy_cb)
    sub_motorpow = rospy.Subscriber("/mobile_base/commands/motor_power", MotorPower, motor_power_cb)
    sub_user_notify = rospy.Subscriber("notify_user", UserNotification, notify_cb)

    ## todo: is this happening at the right time?
    if (in_cp1()
            and config.initial_obstacle
            and (not place_obstacle(config.initial_obstacle_location))):
        log_das(LogError.STARTUP_ERROR, "Fatal: could not place inital obstacle")
        raise Exception("start up error")

    ## todo: this may happen too early
    indicate_ready(SubSystem.BASE)

    ## actually start up the flask service. this never returns, so it must
    ## be the last thing in the file
    app.run(host="0.0.0.0")
