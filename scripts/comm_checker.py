#!/usr/bin/env python
import rospy
import socket                                         
import time


import service
import messages
import comm


class CommChecker(service.persistent):
    def __init__(self):
        service.persistent.__init__(self)
        self.name = 'comm_checker'
        self.daemon = False

        self.rate = rospy.Rate(comm.params['rate']) # 10hz
        #variable for coordination
        self.current_node = None
        self.last_node = dict(x=0.0, y=0.0, id=-1)
        
    
    def _search_neighbor(self):
        return comm.neighbor_list

    def main(self):

       
        neighbors = self._search_neighbor() #TODO: BT-find neighbors
        #establish connection with neighbor robots
        # for dest in neighbors:
        #     messages.sender.ready(comm.params['ports'][dest])
        
        #get the near node the robot is approaching
        self.current_node, distance = comm.get_current_node()
        if not self.current_node and distance == 0.0:
            #rospy.loginfo('No current node and just pass this iteration')
            return

        if self.current_node != self.last_node:
            rospy.loginfo('Switch from node %s to %s'%(self.last_node['id'], self.current_node['id']))

        if comm.state == 'IDLE':
            
            if distance < comm.params['threshold'] and self.current_node == self.last_node:
                for dest in neighbors:
                    msg_content = [comm.params['id']]
                    msg = messages.create('request',comm.params['id'], dest,'comm_checker', msg_content)
                    messages.send(comm.params['ports'][dest],msg)
                    rospy.loginfo('Sent: '+ msg)
                comm.state = 'QUERY'
                rospy.loginfo('Current State: %s'%comm.state)
            else:
                for dest in neighbors:
                    msg_content = [comm.params['id']]
                    msg = messages.create('beacon',comm.params['id'], dest,'comm_checker', msg_content)
                    messages.send(comm.params['ports'][dest],msg)
                    rospy.loginfo('Sent: '+ msg)


        elif comm.state == 'DONE':

            if distance < comm.params['threshold'] and self.current_node == self.last_node:
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
                for dest in neighbors:
                    msg_content = [comm.params['id']]
                    msg = messages.create('stop',comm.params['id'], dest,'comm_checker', msg_content)
                    messages.send(comm.params['ports'][dest],msg)
                    rospy.loginfo('Sent: '+ msg)
                comm.approaching_dict.clear()
                comm.state = 'IDLE'
                rospy.loginfo('Current State: %s'%comm.state)

        self.last_node = self.current_node            
        self.rate.sleep()

  

if __name__ == '__main__':
    try:
        comm_checker = CommChecker()
        comm_checker.start()
    
    except rospy.ROSInterruptException:
        rospy.loginfo("Communicator checker finished.")
