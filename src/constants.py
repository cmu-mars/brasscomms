""" defines enums and string constants used throughout brasscomms """
import collections
from enum import Enum

TH_URL = "http://brass-th"
TA_URL = "http://brass-ta"
CONFIG_FILE_PATH = '/test/data'
LOG_FILE_PATH = '/test/log'
CP_GAZ = '/home/vagrant/catkin_ws/src/cp_gazebo'
BINDIR = '/home/vagrant/bin'
JSON_MIME = 'application/json'
JSON_HEADER = {'content-type': JSON_MIME}
CAL_ERROR_THRESH = 3

## this is all of the time format EXCEPT the trailing Z, which needs to get
## put on manually or else the truncation for %f down to three is brittle
TIME_FORMAT = '%Y-%m-%dT%H:%M:%S.%f'

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
    TEST_ERROR = 9

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
    """ enumeration of the reasons we might finish before timeout  """
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
INTERNAL_STATUS = Endpoint(url='/internal/status', methods=['POST'])

class AdaptationLevels(Enum):
    """ adaptations levels for config file """
    CP1_NoAdaptation = 1
    CP2_NoAdaptation = 2
    CP1_Adaptation = 3
    CP2_Adaptation = 4

class SubSystem(Enum):
    """ Subsystems comprising experiment """
    BASE = 1
    DAS = 2
