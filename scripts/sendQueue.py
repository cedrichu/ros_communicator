#!/usr/bin/env python


import comm
import service
import socket
import time
import math
import threading
import cpQueue as Queue

import rospy

class sendClient(service.persistent):
  def __init__(self,addr):
    service.persistent.__init__(self)
    self.name = 'send-'+addr[0]
    self.addr = addr
    self.queue = Queue.Queue(maxsize=1000)
    self.event = threading.Event()
    self.batch = comm.params['queue']['batch']
    self.daemon = True
    
  def restart(self):
    self.quit()
    self.__init__(self.addr)
    self.start()
  def put(self,data):
    try:
      self.queue.put((0,0,data))
      self.event.set()
    except Exception as e:
      rospy.logerr('Cannot put data (%s) in queue for (%s): %s'%(str(data),str(self.addr),e))
  def send(self,data):
    sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    try:
      sock.connect(self.addr)
    except Exception as e:
      rospy.logwarn('Cannot connect to %s: %s'%(str(self.addr),e))
      return False
    else:
      try:
        sock.send(data)
      except:
        rospy.logwarn('Cannot send data to %s: %s'%(str(self.addr),e))
        return False
      else:
        return True
  def get(self):
    numMessages = 0
    addMessages = True
    msg = ''
    while addMessages and numMessages < self.batch:
      try:
        data = self.queue.get_nowait()
      except Queue.Empty:
        if numMessages > 0:
          addMessages = False
        else:
          self.event.clear()
          if self.event.wait(1.0):
            pass
          else:
            addMessages = False
      else:
        if len(data) == 3:
          msg += data[2]
          msg += '\n'
          numMessages += 1
        else:
          rospy.logwarn('Entry in queue incorrectly formatted: %s'%str(data))
    return msg.strip()
  def main(self):
    data = self.get()
    if data:
      tries = 1
      sent = self.send(data)
      while self.stayAlive and not sent:
        sleeptime = round(30/(1 + math.exp(-1.1*tries + 6)),1)
        rospy.logwarn('Sleeping for %.1f seconds (%d tries)'%(sleeptime,tries))
        time.sleep(sleeptime)
        sent = self.send(data)
        tries += 1

class sendQueue():
  def __init__(self):
    self.conns = dict()
    self.client = sendClient
    self.lock = threading.Lock()
    self.threads = []
    self.stayAlive = True
  def send(self,addr,data):
    #rospy.loginfo('Trying to send to %s'%str(addr))
    if self.stayAlive:
      if self.ready(addr):
        self.conns[addr].put(data)
      else:
        rospy.logerr('Cannot give data to sending client: %s %s'%(str(addr),data))
        rospy.logerr('Could not send data to %s: %s'%(str(addr),data))
  def ready(self,addr):
    with self.lock:
      if addr in self.conns.keys():
        if self.conns[addr].isAlive():
          return True
        else:
          try:
            self.conns[addr].restart()
            rospy.loginfo('Restarting sending client on %s'%str(addr))
          except Exception as e:
            rospy.logerr('Cannot restart sending client for %s'%str(addr))
            return False
          else:
            return True
      else:
        try:
          self.conns[addr] = self.client(addr)
          self.conns[addr].start()
        except Exception as e:
          rospy.logerr('Cannot create sending client for %s'%str(addr))
          return False
        else:
          return True
  def stop(self):
    self.stayAlive = False
    #for thread in self.threads:
      #thread.stop()
      #thread.stayAlive = False
