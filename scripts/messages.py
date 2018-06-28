#!/usr/bin/env python

import socket
import time
import json
import random
import sendQueue
import cpQueue as Queue
import threading


import rospy



sender = sendQueue.sendQueue()
send = sender.send
receive_queue = Queue.PriorityQueue()
receive_event = threading.Event()


def receive(data):
  global receive_queue,receive_event
  put(receive_queue,receive_event,data)

def put(queue,event,data):
  try:
    queue.put((0,0,data))
  except Exception as e:
    rospy.logwarn('Failed to put message in the queue: %s (%s)'%(data,e))
  else:
    event.set()

def parse(data):
  keys = ['type', 'time', 'orig', 'dest', 'source', 'data']
  funcs = [str, int, str, str, str, json.loads]
  dlist = data.split('|')
  if len(dlist) != 6:
    rospy.logwarn('This message has %d fields instead of 6 and cannot be dispatched: %s'%(len(dlist),data))
    dlist = ['',0,'','','','""']
  try:
    msg = dict(zip(keys, [func(val) for func,val in zip(funcs,dlist)]))
  except Exception as e:
    rospy.logwarn('Could not parse message (%s): %s, trying data as str'%(e,str(data)))
    try:
      funcs = [str, int, str, str, str, str]
      msg = dict(zip(keys, [func(val) for func,val in zip(funcs,dlist)]))
    except Exception as f:
      rospy.logwarn('Still could not parse message (%s): %s'%(f, str(data)))
      dlist = ['',0,'','','','']
      msg = dict(zip(keys, [func(val) for func,val in zip(funcs,dlist)]))
  return msg

def create(type,orig,dest,source,data,ctime=None):
  if not ctime:
    ctime = rospy.get_rostime().secs
  keys = ['type', 'time', 'orig', 'dest', 'source', 'data']
  vals = [type, ctime, orig, dest, source, data]
  msg = dict(zip(keys, vals))
  return format(msg)

def format(msg):
  keys = ['type', 'time', 'orig', 'dest', 'source', 'data']
  funcs = [str, lambda x: '%d'%x, str, str, str, json.dumps]
  return "|".join([func(msg[key]) for func,key in zip(funcs,keys)])
