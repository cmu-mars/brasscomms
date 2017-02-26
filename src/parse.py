"""classes representing the results of parsing the config file and args to
end points, with attributes to enforce invariants
"""
import math
import datetime

from attr.validators import instance_of
import attr


from constants import AdaptationLevels, TIME_FORMAT

def in_range_inclusive(low=None, high=None, kind=None):
    """ produce range checkers approriate for attrs given lower and upper bounds"""
    def _isvalid(instance, attribute, value):
        if not isinstance(value, kind):
            raise ValueError('%s is not a %s' % (value, kind))
        if value < low or value > high:
            raise ValueError('%s not in [%s,%s]' % (value, low, high))

    return _isvalid

def time_string(instance, attribute, value):
    """ tries to parse the given value according to MIT spec for time formats, errors otherwise """
    try:
        if isinstance(value, unicode):
            datetime.datetime.strptime(value, TIME_FORMAT + 'Z')
        else:
            raise ValueError('%s not a unicode string, so cannot parse as datetime object' % value)
    except ValueError as ve:
        raise ValueError('string %s did not part as a datetime object under %s: %s'
                         % (value, TIME_FORMAT, ve))

## uses of the above that appear more than once
VALID_VOLT = in_range_inclusive(low=104, high=166, kind=int)
VALID_33 = in_range_inclusive(low=-3, high=3, kind=int)
VALID_66 = in_range_inclusive(low=-6, high=6, kind=int)

def ensure_cls(cl):
    """If the attribute is an instance of cls, pass, else try constructing."""

    def converter(val):
        """ convert val to the instance """
        if isinstance(val, cl):
            return val
        else:
            return cl(**val)

    return converter

def ensure_enum(cl):
    """If the attribute is an instance of cls, pass, else try constructing."""

    def converter(val):
        """ convert val to the instance """
        if isinstance(val, cl):
            return val
        else:
            return cl[val]

    return converter

@attr.s
class Coords(object):
    """ class with attributes for inital position """
    x = attr.ib(validator=in_range_inclusive(low=-1.0, high=100.0, kind=float))
    y = attr.ib(validator=in_range_inclusive(low=-1.0, high=112.0, kind=float))

@attr.s
class Bump(object):
    """ class with attributes for sensor bumps """
    x = attr.ib(validator=VALID_33)
    y = attr.ib(validator=VALID_33)
    z = attr.ib(validator=VALID_33)
    p = attr.ib(validator=VALID_66)
    w = attr.ib(validator=VALID_66)
    r = attr.ib(validator=VALID_66)

@attr.s
class SingleBumpName(object):
    """ class with attributes for just a thing called bump that's a mapping """
    bump = attr.ib(validator=instance_of(dict))

@attr.s
class Config(object):
    """ class with attributes for the config file """
    start_loc = attr.ib(validator=instance_of(unicode))
    start_yaw = attr.ib(validator=in_range_inclusive(low=0.0, high=2*math.pi, kind=float))
    target_loc = attr.ib(validator=instance_of(unicode))
    enable_adaptation = attr.ib(convert=ensure_enum(AdaptationLevels))
    initial_voltage = attr.ib(validator=VALID_VOLT)
    initial_obstacle = attr.ib(validator=instance_of(bool))
    initial_obstacle_location = attr.ib(convert=ensure_cls(Coords))
    sensor_perturbation = attr.ib(convert=ensure_cls(Bump))

@attr.s
class TestAction(object):
    """ class with attributes for test actions, leaving arguments unparsed """
    TIME = attr.ib(validator=time_string)
    ARGUMENTS = attr.ib(validator=instance_of(dict))

@attr.s
class Voltage(object):
    """ class with attributes for voltage """
    voltage = attr.ib(validator=VALID_VOLT)

@attr.s
class ObstacleID(object):
    """ class with attributes for obstacle ids """
    ## todo: also check here if it's a good name?
    obstacleid = attr.ib(validator=instance_of(unicode))

@attr.s
class InternalStatus(object):
    """ class with attributes for internal status from rainbow """
    STATUS = attr.ib(validator=instance_of(unicode))
    MESSAGE = attr.ib(validator=instance_of(unicode))
    TIME = attr.ib(validator=time_string)
