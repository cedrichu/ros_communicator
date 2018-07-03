#!/usr/bin/env python
import rospy
import socket                                         
import time
import cpQueue as Queue
from math import sqrt
import numpy as np



params = dict()

params['id'] = 'freight3'
params['queue'] = dict()
params['queue']['batch'] = 50
params['threshold'] = 1


neighbor_list = ['freight20']

params['ports'] = dict()
params['ports']['local'] = dict()
params['ports']['local']['receiver'] = ('',60998) 
params['ports'][neighbor_list[0]] = (neighbor_list[0],60998)




#shared information?
state = 'IDLE'
nodes = []
current_pose = None
current_node = None
current_distance = 0.0
current_vel = 0.0
delta = 0.5
send_count = 0 #recording how many responses received within each query
approaching_dict = dict() #The valid set of neighbors


def get_distance_from_node_to_pose(node, pose):
	return get_distance(node['x'], pose['x'], node['y'], pose['y']) #TODO: sync with reactor

def get_distance(x1, x2, y1, y2):
	return sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def get_current_node():
	global current_node, current_distance
	adjacent_nodes = []
	for node in nodes:
		if not is_passed(node):
			distance = get_distance_from_node_to_pose(node, current_pose)
			adjacent_nodes.append((node, distance))
	
	best_node = None
	closest_distance = 0.0
	try:
		best_node, closest_distance = min(adjacent_nodes, key=lambda dist: dist[1])
		current_node = best_node
		current_distance = closest_distance
	except Exception as e:
		rospy.logwarn('No valid adjacent_nodes')
	return best_node, closest_distance

#used for judging if the robot passes the node on the map 
def is_passed(node):
	moving_vec = np.array([np.cos(current_pose['theta']), np.sin(current_pose['theta'])])
	pose_vec = np.array([node['x'] - current_pose['x'], node['y'] - current_pose['y']])
	return True if np.inner(moving_vec, pose_vec) < 0 else False






