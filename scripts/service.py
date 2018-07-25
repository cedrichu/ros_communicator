#!/usr/bin/env python

import socket
from bluetooth import *
import threading
import time
import logging
import errno

import rospy

class persistent(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    self.stayAlive = True
    self.daemon = True
  def stop(self):
    self.stayAlive = False
  def quit(self):
    if self.isAlive():
      self.stop()
      self.join()
  def restart(self):
    self.quit()
    self.__init__()
    self.start()
  def pre(self):
    pass
  def main(self):
    pass
  def post(self):
    pass
  def run(self):
    rospy.loginfo('Starting %s'%self.name)
    self.pre()
    while self.stayAlive:
      self.main()
    self.post()
    rospy.loginfo('Stopping %s'%self.name)



class listener(persistent):
  def __init__(self):
    persistent.__init__(self)
    self.addr = ''
    self.client = ''
    #self.sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    self.sock = BluetoothSocket(RFCOMM)
    #self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  def pre(self):
    if self.addr and self.client:
      try:
        #self.sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        #self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        #self.sock.bind(self.addr)
        #self.sock.settimeout(1)
        #self.sock.listen(100)
        self.sock = BluetoothSocket(RFCOMM)
        self.sock.bind(self.addr)
        self.sock.listen(10)
        #TODO: support multiple robots??
        #self.client_sock, self.client_addr = self.sock.accept()
        #self.client(self.client_sock).start()

      except Exception as e:
        rospy.logfatal('Cannot listen on address (%s,%d): %s'%(self.addr[0],self.addr[1],e))
        self.stop()
    else:
      self.stop()
  
  def main(self):
    try:
      sock,addr = self.sock.accept()
      #except socket.timeout:
      #pass
    except Exception as e:
      rospy.logerr('Caught an unexpected exception while listening, restarting socket: %s'%e)
      self.post()
      self.pre()
    else:
      try:
        self.client(sock).start()
      except Exception as e:
        rospy.logfatal('Failed to start client thread (%n threads active): %s'%(threading.active_count(),e))
        try:
          sock.shutdown(1)
          sock.close()
        except Exception as e:
          rospy.logerr('Could not close client socket for listener: %s'%e)
  def post(self):
    try:
      self.sock.shutdown(1)
      self.sock.close()
    except Exception as e:
      rospy.logwarn('Caught an exception while trying to shutdown a listener socket, might already be closed: %s'%e)



class client(threading.Thread):
  def __init__(self,sock):
    threading.Thread.__init__(self)
    self.sock = sock
  def sockclose(self):
    try:
      self.sock.shutdown(1)
    except Exception as e:
      rospy.logwarn('Socket has already been shutdown: %s'%e)
    try:
      self.sock.close()
    except Exception as e:
      rospy.logwarn('Socket has already been closed: %s'%e)



class receiver(client):
  def handler(self,data):
    pass
  def run(self):
    data = ''
    while True:
      try:
        data = self.sock.recv(1024)
      except  socket.error, e:
        if e.errno == errno.EAGAIN:
          time.sleep(.01)
        else:
          raise e
          rospy.logerr('Could not receive data: %s'%e)
          break
      else:
        try:
          self.handler(data)
        except Exception as e:
          rospy.logerr('Could not handle data: %s (%s)'%(data,e))
        #data = d
        #break
        #if d == '\n':
          #break
    #self.sockclose()
    
    # try:
    #   self.handler(data)
    # except Exception as e:
    #   rospy.logerr('Could not handle data: %s (%s)'%(data,e))

