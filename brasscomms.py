from flask import Flask , request
app = Flask(__name__)

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


@app.route('/phase1/power/initial_settings', methods=['POST'])
def initalSettings():
    ## do i need these asserts, or was that just boiler plate? can this def
    ## get called by any thing other than a POST to the URL i specify?
    assert request.method == 'POST'

    ## how to check that there's nothing else in the args?

    ## todo: type check all these values. they're probably just strings,
    ## make sure they meet the spec or barf appropriately
    adaptions = request.args.get('enable_adaptions','')
    startPercent = request.args.get('start_battery','')
    obsLoc = request.args.get('obstacle_location','')
    adaptPercent = request.args.get('minimum_battery','')

@app.route('/phase1/power/get_robot_location', methods=['GET'])
def location():
    assert request.path == '/phase1/power/get_robot_location'
    assert request.method == 'GET'
    return 'todo: make a ROS call here to determine location'

@app.route('/phase1/power/get_battery_level', methods=['GET'])
def battery():
    assert request.path == '/phase1/power/get_battery_level'
    assert request.method == 'GET'
    return 'todo: make a ROS call here to determine battery level'

@app.route('/phase1/power/change_power', methods=['POST'])
def changePower():
    assert request.path == '/phase1/power/change_power'
    assert request.method == 'POST'

    currentPower = request.args.get('current_battery','')

@app.route('/phase1/power/add_obstacle', methods=['POST'])
def changePower():
    assert request.path == '/phase1/power/add_obstacle'
    assert request.method == 'POST'

    currentPower = request.args.get('obstacle_location','')

@app.route('/phase1/power/stop_challenge_problem', methods=['POST'])
def stopChallengeProblem():
    assert request.path == '/phase1/power/stop_challenge_problem'
    assert request.method == 'POST'
    return 'todo: make a ROS call here to stop the bot'




if __name__ == "__main__":
    app.run()
