"""A multi-producer, multi-consumer, circular, priority queue based on Queue.py"""

from time import time as _time
try:
  import threading as _threading
except ImportError:
  import dummy_threading as _threading
from collections import deque

__all__ = ['Empty', 'Priority', 'Queue']

class Empty(Exception):
    "Exception raised by Queue.get(block=0)/get_nowait()."
    pass

class Priority(Exception):
    "Exception raised by Queue when indexing an invalid priority."
    pass

class Queue:
  def __init__(self, maxsize=120000, priorities=3):
    self.maxsize = maxsize
    self.priorities = priorities
    self._init()
    self.mutex = _threading.Lock()
    
    self.event = _threading.Event()
    self.wait = self.event.wait
    self.set = self.event.set
    self.clear = self.event.clear

  def qsize(self):
    with self.mutex:
      n = self._qsize()
    return n

  def empty(self):
    with self.mutex:
      n = self._empty()
    return n

  def put(self, item):
    with self.mutex:
      if not len(item) == self.priorities:
        raise Priority
      elif not type(item[0]) == int:
        raise Priority
      else:
        self._put(item)
      
  def put_nowait(self, item):
    self.put(item)

  def get(self):
    with self.mutex:
      if not self._qsize():
        raise Empty
      for n in xrange(self._qlen()):
        if self._psize(n):
          item = self._get(n)
          return item

  def get_nowait(self):
    return self.get()

  def reset(self):
    self.queue = []
    for priority in xrange(self.priorities):
      self.queue.append(deque(maxlen=self.maxsize))

  def _init(self):
    self.queue = []
    for priority in xrange(self.priorities):
      self.queue.append(deque(maxlen=self.maxsize))

  def _empty(self):
    return not self._qsize()

  def _qsize(self):
    return sum([self._psize(n) for n in xrange(self._qlen())])

  def _qlen(self):
    return len(self.queue)

  def _psize(self,n):
    if n < self._qlen():
      return len(self.queue[n])
    else:
      raise Priority

  def _put(self, item):
    n = item[0]
    if n < self._qlen():
      self.queue[n].append(item)
    else:
      raise Priority

  def _get(self, n):
    if n < self._qlen():
      return self.queue[n].popleft()
    else:
      raise Priority

class PriorityQueue(Queue):
  pass
