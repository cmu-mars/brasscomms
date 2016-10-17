from flask import Flask , request
from enum import Enum
app = Flask(__name__)

status = Enum ('Status', 'Starting Operational Adapting ShuttingDown Completed ')

## subroutines for the first deliverable

@app.route('/logs/status/DASSTATUS', methods=['GET'])
def status():
    assert request.path == '/logs/status/DASSTATUS'
    assert request.method == 'GET'
    return 'todo: make a ROS call here to determine status'

@app.route('/phase1/power/start_challenge_problem', methods=['POST'])
def startChallengeProblem():
    assert request.path == '/phase1/power/start_challenge_problem'
    assert request.method == 'POST'
    return 'todo: make a ROS call here to start the bot\n you posted' + (str (request.form))

@app.route('/phase1/power/stop_challenge_problem', methods=['POST'])
def stopChallengeProblem():
    assert request.path == '/phase1/power/stop_challenge_problem'
    assert request.method == 'POST'

    ## signal_shutdown("comms requested shutdown")

    return 'todo: make a ROS call here to stop the bot'

## checks to see if a string represents an integer
def isint(x):
    try:
        int(s)
        return True
    except ValueError:
        return False


## returns true iff the first argument is a digit inclusively between the
## second two args. assumes that the second two are indeed digits, and that
## the second is less than the third.
def int_out_of_range(x,upper,lower) :
    return not(isint(x) and x >= lower and x <= upper)

## subroutines for the rest of the full API

@app.route('/phase1/power/initial_settings', methods=['POST'])
def initalSettings():
    assert request.path == '/phase1/power/initial_settings'
    assert request.method == 'POST'

    ## how to check that there's nothing else in the args?

    ## todo: type check all these values. they're probably just strings,
    ## make sure they meet the spec or barf appropriately
    adaptions = request.args.get('enable_adaptions','')
    if not (adaptions == 'true' or adaptions == 'false'):
        flask.abort(400)

    startPercent = request.args.get('start_battery','')
    if int_out_of_range (startPercent, 0, 100):
        flask.abort(400)

    # assume that the locations are strings "x,y".
    obsLoc = request.args.get('obstacle_location','')
    [obsLoc_x , obsLoc_y] = obsLoc.split(',')
    if not (isint(obsLoc_x) and isint(obsLoc_y)):
        flask.abort(400)

    adaptPercent = request.args.get('minimum_battery','')
    if int_out_of_range(adaptPercent, 0, 100) or adaptPercent >= startPercent:
        flask.abort(400)

    return "todo: make a call here now that the data's all checked"

@app.route('/phase1/power/get_robot_location', methods=['GET'])
def location():
    assert request.path == '/phase1/power/get_robot_location'
    assert request.method == 'GET'
    return 'todo: make a ROS call here to determine location'

@app.route('/phase1/power/get_battery_level', methods=['GET'])
def battery():
    assert request.path == '/phase1/power/get_battery_level'
    assert request.method == 'GET'
    return 'todo: make a ROS/gazebo extension call here to determine battery level.'

@app.route('/phase1/power/change_power', methods=['POST'])
def changePower():
    assert request.path == '/phase1/power/change_power'
    assert request.method == 'POST'

    currentPower = request.args.get('current_battery','')

    return 'todo: make a call here to change the power'

@app.route('/phase1/power/add_obstacle', methods=['POST'])
def addObstacle():
    assert request.path == '/phase1/power/add_obstacle'
    assert request.method == 'POST'

    obs_loc = request.args.get('obstacle_location','')

    return 'todo: make a call to add the obstactle here'

@app.route('/phase1/power/remove_obstacle', methods=['POST'])
def removeObstacle():
    assert request.path == '/phase1/power/remove_obstacle'
    assert request.method == 'POST'

    return 'todo: make a call to remove the obstacle here'

## these are under the recalibration heading rather than power; the APIs
## seem less-well defined, so while they're still stubbed in, it's not
## totally clear what they will do.

@app.route('/phase1/recalibration/start_challenge_problem', methods=['POST'])
def recal_start():
    assert request.path == '/phase1/recalibration/start_challenge_problem'
    assert request.method == 'POST'

    return 'todo: make a call to start navigation through map for CP2'

@app.route('/phase1/recalibration/initial_settings', methods=['POST'])
def recal_init():
    assert request.path == '/phase1/recalibration/initial_settings'
    assert request.method == 'POST'

    ## todo: check taht each of these produces something and that it's in range
    adaptions = request.args.get('enable_adaptions','')
    kinect_dx = request.args.get('kinect_dx','')
    kinect_dy = request.args.get('kinect_dy','')
    kinect_dz = request.args.get('kinect_dz','')
    kinect_dr = request.args.get('kinect_dr','')
    kinect_dp = request.args.get('kinect_dp','')
    kinect_dq = request.args.get('kinect_dw','')

    return 'todo: make a call here; what is this supposed to do, exactly? no spec given'

@app.route('/phase1/recalibration/change_settings', methods=['POST'])
def recal_change():
    assert request.path == '/phase1/recalibration/change_settings'
    assert request.method == 'POST'

    ## todo: check taht each of these produces something and that it's in range
    kinect_dx = request.args.get('kinect_dx','')
    kinect_dy = request.args.get('kinect_dy','')
    kinect_dz = request.args.get('kinect_dz','')
    kinect_dr = request.args.get('kinect_dr','')
    kinect_dp = request.args.get('kinect_dp','')
    kinect_dq = request.args.get('kinect_dw','')

    return 'todo: make a call here'

@app.route('/phase1/power/get_current_state', methods=['GET'])
def recal_state():
    assert request.path == '/phase1/power/get_current_state'
    assert request.method == 'GET'
    return 'todo: make a ROS call here to determine status. specify output format.'

@app.route('/phase1/recalibration/stop_challenge_problem', methods=['GET'])
def recal_stop():
    assert request.path == '/phase1/recalibration/stop_challenge_problem'
    assert request.method == 'GET'
    return 'todo: make a ROS call to stop the challenge problem'





if __name__ == "__main__":
    app.run()
