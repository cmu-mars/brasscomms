""" utility functions for working with waypoints and maps """
### imports
from __future__ import with_statement
import json

def waypoint_to_coords(waypoint_id):
    """ given a way point, produce its coordinates """
    if not is_waypoint(waypoint_id):
        raise KeyError('The specified waypointID does not exist')
    waypoint_list = load_waypoints(waypoint_id)
    waypoint = waypoint_list[0]
    return {waypoint['coord']}

def is_waypoint(waypoint_id):
    """ given a string, determine if it is actually a waypoint id """
    waypoint_list = load_waypoints(waypoint_id)
    if len(waypoint_list) > 1:
        raise ValueError('There is more than one waypoint with the same identifier in the waypoint locations config file')
    return len(waypoint_list) == 1

def load_waypoints(waypoint_id):
    """ produce the list of waypoints names matching the given ID """
    waypoint_locations_path = '/home/vagrant/catkin_ws/src/cp_gazebo/maps/Wean-entire-floor4-waypoint-locations.json'
    with open(waypoint_locations_path) as waypoints_file:
        data = json.load(waypoints_file)
        waypoint_list = data["map"]
        return filter(lambda waypoint: waypoint["node-id"] == waypoint_id, waypoint_list)
