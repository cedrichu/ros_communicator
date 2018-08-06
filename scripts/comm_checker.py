#!/usr/bin/env python
import rospy
import socket                                         
import time
from bluetooth import *


import service
import messages
import comm

from geometry_msgs.msg import Twist

class CommChecker(service.persistent):
    def __init__(self):
        service.persistent.__init__(self)
        self.name = 'comm_checker'
        self.daemon = False

        self.rate = rospy.Rate(comm.params['rate'])
        #variable for coordination
        self.current_node = None
        self.distance = 0.0
        self.last_node = dict(x=0.0, y=0.0, id=-1)

        #self.comm_vel_pub = rospy.Publisher('/comm_vel', Twist, queue_size=10)
        
    
    def _search_neighbor(self):
        #TODO: when to set SEARCH state?? timer? now it is because of empty list
        if comm.state == 'SEARCH' or not comm.neighbor_list:
            nearby_devices = discover_devices(
                duration=5, lookup_names=False, flush_cache=True, lookup_class=False)
            comm.neighbor_list = []
            if nearby_devices:
                #Check if neibor devices have robots
                for addr in nearby_devices:
                    if addr in comm.params['addr2robot'].keys():
                        robot = comm.params['addr2robot'][addr]
                        comm.neighbor_list.append(robot)
                        comm.params['ports'][robot] = (addr, 1)
                        rospy.loginfo('Find robot %s on %s' %(robot, addr))

                time.sleep(8)
                #Connect to neighbor robots and store connections.
                if comm.neighbor_list:
                    for dest in comm.neighbor_list:
                        #establish connection with neighbor robots
                        messages.sender.ready(comm.params['ports'][dest])
                else:
                    rospy.loginfo('No neighbor is found.')
            else:
                rospy.loginfo('No neighbor is found.')

            comm.search_time = time.time()
            comm.state = 'IDLE'
            rospy.loginfo('Search is done. Current State: %s'%comm.state)



        return comm.neighbor_list

    def main(self):
        #get the near node the robot is approaching
        self.current_node, self.distance = comm.get_current_node()
        if not self.current_node and self.distance == 0.0:
            return
        #Changing the current node means that the robot already passes the node
        if self.current_node != self.last_node:
            rospy.loginfo('Switch from node %s to %s'%(self.last_node['id'], self.current_node['id']))

        self.checker()    
        self.last_node = self.current_node            
        self.rate.sleep()

    def checker(self):
       
        neighbors = self._search_neighbor()
        if not neighbors:
            return
                
        if comm.state == 'IDLE':
            
            if self.distance < comm.params['threshold'] and self.current_node == self.last_node:
                comm.send_number = len(neighbors)
                for dest in neighbors:
                    msg_content = [comm.params['id']]
                    msg = messages.create('request',comm.params['id'], dest,'comm_checker', msg_content)
                    messages.send(comm.params['ports'][dest],msg)
                    rospy.loginfo('Sent: '+ msg)
                comm.state = 'QUERY'
                rospy.loginfo('Current State: %s'%comm.state)
            elif (time.time() - comm.search_time) > comm.params['search_period']:
                comm.state = 'SEARCH'
                rospy.loginfo('Current State: %s'%comm.state)

        elif comm.state == 'DONE':

            if self.distance < comm.params['threshold'] and self.current_node == self.last_node:
                for dest in neighbors:
                    msg_content = [comm.params['id']]
                    msg = messages.create('request',comm.params['id'], dest,'comm_checker', msg_content)
                    messages.send(comm.params['ports'][dest],msg)
                    rospy.loginfo('Sent: '+ msg)
                comm.state = 'QUERY'
                rospy.loginfo('Current State: %s'%comm.state)

            if self.current_node != self.last_node:
                for dest in neighbors:
                    msg_content = [comm.params['id']]
                    msg = messages.create('stop',comm.params['id'], dest,'comm_checker', msg_content)
                    messages.send(comm.params['ports'][dest],msg)
                    rospy.loginfo('Sent: '+ msg)
                comm.approaching_dict.clear()
                comm.state = 'IDLE'
                rospy.loginfo('Current State: %s'%comm.state)

        elif comm.state == 'QUERY':
            
            if self.current_node != self.last_node:
                while comm.state != 'DONE':
                    pass
                for dest in neighbors:
                    msg_content = [comm.params['id']]
                    msg = messages.create('stop',comm.params['id'], dest,'comm_checker', msg_content)
                    messages.send(comm.params['ports'][dest],msg)
                    rospy.loginfo('Sent: '+ msg)
                comm.approaching_dict.clear()
                comm.state = 'IDLE'
                rospy.loginfo('Current State: %s'%comm.state)

        
  

if __name__ == '__main__':
    try:
        comm_checker = CommChecker()
        comm_checker.start()
    
    except rospy.ROSInterruptException:
        rospy.loginfo("Communicator checker finished.")
