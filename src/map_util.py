### imports
from __future__ import with_statement
import sys
import json
import os.path

def waypointToCoords(waypointID):
    if (not isWaypoint(waypointID)) :
        raise KeyError('The specified waypointID does not exist')
    waypointList = loadWaypointListFromFile(waypointID) 
    waypoint = waypointList[0]
    return {waypoint['coord']}

def isWaypoint(waypointID):
    waypointList =loadWaypointListFromFile(waypointID)
    if (len(waypointList) > 1) :
        raise ValueError('There is more than one waypoint with the same identifier in the waypoint locations config file')
    return len(waypointList) == 1

def loadWaypointListFromFile(waypointID):
    waypoint_locations_path = '/home/vagrant/catkin_ws/src/cp_gazebo/maps/Wean-entire-floor4-waypoint-locations.json'
    with open(waypoint_locations_path) as waypoints_file:
        data = json.load(waypoints_file)
        waypoint_list = data["map"]
        return filter(lambda waypoint: waypoint["node-id"] == waypointID, waypoint_list)

