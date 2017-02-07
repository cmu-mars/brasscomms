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

th_url = "http://brass-th"
config_file_path = '/test/data'
log_file_path = '/test/log'

class EP(Enum):
