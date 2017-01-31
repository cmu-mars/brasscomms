### imports
import sys
import json
import os.path

def waypointToCoords(waypointID):
    if (not isWaypoint(waypointID)) :
        raise KeyError('The specified waypointID does not exist')
    waypointList = loadWaypointListFromFile(waypointID) 
    waypoint = waypointList[0]
    return {'x': waypoint['coord']['x'], 'y': waypoint['coord']['y']}

def isWaypoint(waypointID):
    waypointList =loadWaypointListFromFile(waypointID)
    if (len(waypointList) > 1) :
        raise ValueError('There is more than one waypoint with the same identifier in the waypoint locations config file')
    return len(waypointList) == 1


def loadWaypointListFromFile(waypointID):
    waypoint_locations_path = '/home/vagrant/catkin_ws/src/cp_gazebo/maps/Wean-entire-floor4-waypoint-locations.json'
    waypoint_locations_file = open(ig_path, "r")
    map_dict = json.load(waypoint_locations_file)
    waypoint_list = map_dict["map"]
    return filter(waypoint_list, lambda waypoint: waypoint["node-id"] == waypointID)

