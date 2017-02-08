import attr
from enum import Enum
from attr.validators import instance_of

from constants import *

def in_range_inclusive(low=None, high=None, type=None):
    def _isvalid(instance, attribute, value):
        if value < low or value > high:
            raise ValueError('{} out of range'.format(value))
        if not isinstance(value, type):
            raise ValueError('{} out of range'.format(value))

    return _isvalid

def ensure_cls(cl):
    """If the attribute is an instance of cls, pass, else try constructing."""

    def converter(val):
        if isinstance(val, cl):
            return val
        else:
            return cl(**val)

    return converter

def ensure_enum(cl):
    """If the attribute is an instance of cls, pass, else try constructing."""

    def converter(val):
        if isinstance(val, cl):
            return val
        else:
            return cl[val]

    return converter

@attr.s
class InitObs(object):
    x = attr.ib(validator=in_range_inclusive(low=-1, high=24, type=float))
    y = attr.ib(validator=in_range_inclusive(low=-1, high=12, type=float))


@attr.s
class Bump(object):
    x = attr.ib(validator=in_range_inclusive(low=-3, high=3, type=int))
    y = attr.ib(validator=in_range_inclusive(low=-3, high=3, type=int))
    z = attr.ib(validator=in_range_inclusive(low=-3, high=3, type=int))
    p = attr.ib(validator=in_range_inclusive(low=-6, high=6, type=int))
    w = attr.ib(validator=in_range_inclusive(low=-6, high=6, type=int))
    r = attr.ib(validator=in_range_inclusive(low=-6, high=6, type=int))


@attr.s
class Config(object):
    start_loc = attr.ib(validator=instance_of(unicode))
    start_yaw = attr.ib(validator=instance_of(float))
    target_loc = attr.ib(validator=instance_of(unicode))
    enable_adaptation = attr.ib(convert=ensure_enum(AdaptationLevels))
    initial_voltage = attr.ib(validator=in_range_inclusive(low=104, high=166, type=int))
    initial_obstacle = attr.ib(validator=instance_of(bool))
    initial_obstacle_location = attr.ib(convert=ensure_cls(InitObs))
    sensor_perturbation = attr.ib(convert=ensure_cls(Bump))
