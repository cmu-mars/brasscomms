from enum import Enum

th_url = "http://brass-th"
ta_url = "http://brass-th"
config_file_path = '/test/data'
log_file_path = '/test/log'

## defines enums and string constants used throughout brasscomms
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

## each end point is ta_url/action/(one of these)
class EP(Enum):
    query_path = 1
    start = 2
    observe = 3
    set_battery = 4
    place_obstacle = 5
    remove_obstacle = 6
    perturb_sensor = 7

##AdaptationStrings = Enum('AdaptationStrings', 'CP1_NoAdaptation')
class AdaptionLevels(Enum):
    CP1_NoAdaptation = 1
    CP2_NoAdaptation = 2
    CP1_Adaptation = 3
    CP2_Adaptation = 4
