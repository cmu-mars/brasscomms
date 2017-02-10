"""classes representing the results of parsing the config file and args to
end points, with attributes to enforce invariants
"""
import math
import attr
import unicodedata
from attr.validators import instance_of

from constants import AdaptationLevels

def in_range_inclusive(low=None, high=None, kind=None):
    """ produce range checkers approriate for attrs given lower and upper bounds"""
    def _isvalid(instance, attribute, value):
        if not isinstance(value, kind):
            raise ValueError('%s is not a %s' % (value, kind))
        if value < low or value > high:
            raise ValueError('%s not in [%s,%s]' % (value , low, high))

    return _isvalid

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

def ensure_str():
    def converter(val):
        return str(unicodedata.normalize('NFKD', val).encode('ascii','ignore'))
    return converter

@attr.s
class Coords(object):
    """ class with attributes for inital position """
    x = attr.ib(validator=in_range_inclusive(low=-1.0, high=24.0, kind=float))
    y = attr.ib(validator=in_range_inclusive(low=-1.0, high=12.0, kind=float))

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
    ## todo: check that this is a valid time string
    TIME = attr.ib(validator=instance_of(unicode))
    ARGUMENTS = attr.ib(validator=instance_of(dict))

@attr.s
class Voltage(object):
    """ class with attributes for voltage """
    voltage = attr.ib(validator=VALID_VOLT)

@attr.s
class ObstacleID(object):
    """ class with attributes for obstacle ids """
    ## todo: also check here if it's a good name?
    obstacleid = attr.ib(validator=instance_of(str),convert=ensure_str())
