from enum import Enum

TH_URL = "http://brass-th"
TA_URL = "http://brass-th"
CONFIG_FILE_PATH = '/test/data'
LOG_FILE_PATH = '/test/log'
CP_GAZ = '/home/vagrant/catkin_ws/src/cp_gazebo'

""" defines enums and string constants used throughout brasscomms """

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

class EP(Enum):
    """ end point names """
    query_path = 1
    start = 2
    observe = 3
    set_battery = 4
    place_obstacle = 5
    remove_obstacle = 6
    perturb_sensor = 7

class AdaptationLevels(Enum):
    """ adaptations levels for config file """
    CP1_NoAdaptation = 1
    CP2_NoAdaptation = 2
    CP1_Adaptation = 3
    CP2_Adaptation = 4
