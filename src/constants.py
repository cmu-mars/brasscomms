""" defines enums and string constants used throughout brasscomms """
import collections
from enum import Enum

TH_URL = "http://brass-th"
TA_URL = "http://brass-ta"
CONFIG_FILE_PATH = '/test/data'
LOG_FILE_PATH = '/test/log'
CP_GAZ = '/home/vagrant/catkin_ws/src/cp_gazebo'
JSON_MIME = 'application/json'

class Status(Enum):
    """ statuses for DAS_STATUS messages """
    PERTURBATION_DETECTED = 1
    MISSION_SUSPENDED = 2
    MISSION_RESUMED = 3
    MISSION_HALTED = 4
    MISSION_ABORTED = 5
    ADAPTATION_INITIATED = 6
    ADAPTATION_COMPLETED = 7
    ADAPTATION_STOPPED = 8
    ERROR = 9

class Error(Enum):
    """ errors for DAS_ERROR messages """
    TEST_DATA_FILE_ERROR = 1
    TEST_DATA_FORMAT_ERROR = 2
    DAS_LOG_FILE_ERROR = 3
    DAS_OTHER_ERROR = 4

class LogError(Enum):
    """" errors for DAS_ERROR log messages """
    STARTUP_ERROR = 1
    RUNTIME_ERROR = 2
    INFO = 3

class DoneEarly(Enum):
    BATTERY = 1
    AT_TARGET = 2

Endpoint = collections.namedtuple('Endpoint', 'url methods')
QUERY_PATH = Endpoint(url='/action/query_path', methods=['GET'])
START = Endpoint(url='/action/start', methods=['POST'])
OBSERVE = Endpoint(url='/action/observe', methods=['GET'])
SET_BATTERY = Endpoint(url='/action/set_battery', methods=['POST'])
PLACE_OBSTACLE = Endpoint(url='/action/place_obstacle', methods=['POST'])
REMOVE_OBSTACLE = Endpoint(url='/action/remove_obstacle', methods=['POST'])
PERTURB_SENSOR = Endpoint(url='/action/perturb_sensor', methods=['POST'])

class AdaptationLevels(Enum):
    """ adaptations levels for config file """
    CP1_NoAdaptation = 1
    CP2_NoAdaptation = 2
    CP1_Adaptation = 3
    CP2_Adaptation = 4
