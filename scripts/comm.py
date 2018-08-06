#!/usr/bin/env python
import rospy
import socket                                         
import time
import cpQueue as Queue
from math import sqrt
import numpy as np



params = dict()
#robot name
params['id'] = 'freight3'
params['queue'] = dict()
params['queue']['batch'] = 50
#The coordination zone (distance from current node to curren pose)
params['threshold'] = 4.5
#Reset speed after coordination
params['max_speed'] = 1.5
#A mapping of bt addr and robots, which should be accessible from fetchcore in the beginning
params['addr2robot'] = {'00:16:6F:EA:FE:D3' : 'freight3' , '00:16:6F:EB:05:77': 'freight20'}
params['ports'] = dict()
params['ports']['local'] = dict()
#BT port #TODO: how to efficiently utilize all ports
params['ports']['local']['receiver'] = ('',1) 
#ros rate
params['rate'] = 15
#time for searching neighbors
params['search_period'] =  60


neighbor_list = []
#shared information
state = 'IDLE'
nodes = []
current_pose = None
current_node = None
current_distance = 0.0
current_vel = 0.0
send_count = 0 #recording how many responses received within each query
#send_number = 0 #recording how many requests are sent
search_time = time.time()
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

	if adjacent_nodes:
		best_node, closest_distance = min(adjacent_nodes, key=lambda dist: dist[1])
	current_node = best_node
	current_distance = closest_distance
	return best_node, closest_distance

#used for judging if the robot passes the node on the map 
def is_passed(node):
	moving_vec = np.array([np.cos(current_pose['theta']), np.sin(current_pose['theta'])])
	pose_vec = np.array([node['x'] - current_pose['x'], node['y'] - current_pose['y']])
	return True if np.inner(moving_vec, pose_vec) < 0 else False






