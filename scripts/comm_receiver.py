#!/usr/bin/env python
import rospy
import socket                                         
import time
import threading
import operator



import service
import cpQueue as Queue
import messages

import comm

from geometry_msgs.msg import Twist



class receiverClient(service.receiver):
  def handler(self,msg):
    data = msg.strip().split('\n')
    for datum in data:
      if datum:
        messages.receive(datum)

class receiver(service.listener):
  def __init__(self):
    service.listener.__init__(self)
    self.name = "receiver thread"
    self.addr = comm.params['ports']['local']['receiver']
    self.client = receiverClient


class CommReceiver(service.persistent):
    def __init__(self, pub):
        service.persistent.__init__(self)
        self.name = 'comm_receiver'
        self.daemon = False
        
        self.pub = pub
        self.stop_pub = threading.Event()

        self.recv_thread = receiver()
        self.recv_thread.start()


    def _msg_handler(self, data):
        # This message is intended for this intersection, so route it appropriately
        msg = data.split('|')
        # Check to be sure message is properly formatted
        if len(msg) == 6:
            try:
                type = msg[0]
                ctime = int(msg[1])
            except Exception as e:
                rospy.logerr('Could not get the type and time of the message: %s ()'%(data,e))
                return

          #Route information
            if type in ['request']:
                self._request_handler(data)
            elif type in ['stop']:
                #TODO: stop sharing information
                self._stop_handler(data)
            elif type  in ['request-info']:
                self._request_info_handler(data)
            elif type in ['stop-resp']:
                self._stop_resp_handler(data)


    def _request_handler(self, data):      
        try:
            rospy.loginfo('Received: '+ data)
            msg = messages.parse(data)
        except Exception as e:
            rospy.logwarn('Could not parse data: %s (%s)'%(str(data),e))
        else:
            response = [comm.params['id'], comm.current_node['id'], comm.current_distance, comm.current_vel]
            response_msg = messages.create('request-info',comm.params['id'], msg['orig'],'comm_receiver', response)
            messages.send(comm.params['ports'][msg['orig']], response_msg)
            rospy.loginfo("Sent: " + response_msg)


    def _publish_coordinated_vel(self, vel):
        rospy.loginfo('thread starts '+ str(threading.current_thread()))
        while not self.stop_pub.is_set():
            self.pub.publish(vel)
        rospy.loginfo('thread stops! '+ str(threading.current_thread()))


    def _intersection_coordination(self):
        #Assume the processing time of each job is same
        #TODO: different processing time?
        vel = Twist()
        time_to_node = dict()
        #get neighbors' time_to_node 
        for key,value in comm.approaching_dict.iteritems():
            time_to_node[key] = value[0]#/max(value[1], 0.1)
        #Add my time_to_node into the dict
        rospy.loginfo('current distance: '+ str(comm.current_distance)+ ' current velocity: '+ str(comm.current_vel))
        time_to_node[comm.params['id']] = comm.current_distance#/max(comm.current_vel, 0.1)
        rospy.loginfo('current time_to_node: '+ str(time_to_node[comm.params['id']]))

        sorted_ttn = sorted(time_to_node.items(), key=operator.itemgetter(1))
        for i,ttn in enumerate(sorted_ttn):
            if ttn[0] == comm.params['id']:
                rank_t = (10**i)*sorted_ttn[0][1]
                vel.linear.x = (1-i)*0.5#comm.current_distance / rank_t
                rospy.loginfo('Rank: '+ str(i)+' Vel: '+str(vel.linear.x)+ ' TTN: '+str(rank_t)) 
                break

        return vel

    def _maintain_approaching_robots(self, msg):
        n_id, n_node_id, n_dist, n_vel = msg['data']
        if n_node_id == comm.current_node['id']: 
            comm.approaching_dict[n_id] = (n_dist, n_vel)
        else:
            # this robot already left the intersection
            if n_id in comm.approaching_dict.keys():
                comm.approaching_dict.pop(n_id)


    def _request_info_handler(self, data):
        try:
            rospy.loginfo('Received: '+ data+' current_node: '+str(comm.current_node['id']))
            
            #stop current running pub thread
            self.stop_pub.set()
            time.sleep(0.1)
            
            msg = messages.parse(data)
            n_id, n_node_id, n_dist, n_vel = msg['data'] 
        except Exception as e:
            rospy.logwarn('Could not parse data: %s (%s)'%(str(data),e))
        else:
            #Only if the robots that share same intersection will be added into the dict for coordination
            self._maintain_approaching_robots(msg)
            #Got the same number of response as queries, and then start to coordinate 
            comm.send_count += 1
            if comm.send_count == len(comm.neighbor_list):
                #if the list is empty, we don't need to apply velocity coordination
                rospy.loginfo('approaching_dict: '+str(comm.approaching_dict))
                if comm.approaching_dict: 
                    vel = self._intersection_coordination()
                    rospy.loginfo(str(vel))
                    self.pub_vel_thread = threading.Thread(target=self._publish_coordinated_vel, args=(vel,))
                    self.stop_pub.clear()
                    self.pub_vel_thread.start()
                else:
                    rospy.loginfo('rollback to single agent')
                    self.stop_pub.set() #no other approach robots, then just stop control
                comm.send_count = 0
                comm.state = 'DONE'
                rospy.loginfo('Current State: %s'%comm.state)
            #elif n_node in comm.nodes:

    def _stop_handler(self, data):
        try:
            rospy.loginfo('Received: '+ data)
            msg = messages.parse(data)
        except Exception as e:
            rospy.logwarn('Could not parse data: %s (%s)'%(str(data),e))
        else:
            response = [comm.params['id']]
            response_msg = messages.create('stop-resp',comm.params['id'], msg['orig'],'comm_receiver', response)
            messages.send(comm.params['ports'][msg['orig']], response_msg)
            rospy.loginfo('Sent: '+ response_msg)

    def _stop_resp_handler(self, data):
        try:
            rospy.loginfo('Received: '+ data)
            comm.send_count += 1
            if comm.send_count == len(comm.neighbor_list):
                self.stop_pub.set()
                comm.send_count = 0
        except Exception as e:
            rospy.logwarn('Could not parse data: %s (%s)'%(str(data),e))

    def main(self):

        if messages.receive_event.wait():
            messages.receive_event.clear()
            msgs_rem = True
            while msgs_rem:
                try:
                    data = messages.receive_queue.get_nowait()
                except Queue.Empty:
                  msgs_rem = False
                except Exception as e:
                  rospy.logwarn('Getting from queue raised exception (%s)'%e)
                else:
                  if len(data) == 3:
                    try:
                      self._msg_handler(data[2])
                    except Exception as e:
                      rospy.logwarn('Handler failed (%s) %s'%(e,str(data)))
                  else:
                    rospy.logwarn('Entry in queue is incorrectly formatted: %s'%data)
   

if __name__ == '__main__':
    try:
        comm_receiver = CommReceiver()
        comm_receiver.start()
    except rospy.ROSInterruptException:
        rospy.loginfo("Communicator receiver finished.") 


