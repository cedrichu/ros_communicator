#!/usr/bin/env python
import rospy

from fetchcore_msgs.msg import FetchcoreLane, FetchcoreLaneArray
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Point
from tf.transformations import euler_from_quaternion
from freight_base_controller_msgs.msg import DriveCommand

import comm
from comm_checker import CommChecker
from comm_receiver import CommReceiver


class Communicator():

	def __init__(self):

		rospy.init_node('communicator', anonymous=True)
		rospy.Subscriber("/fetchcore/annotations/lanes", FetchcoreLaneArray, self._nodes_handler)
		rospy.Subscriber("/base_controller/command_full", DriveCommand, self._vel_handler)
		rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._pose_handler)
		self.comm_vel_pub = rospy.Publisher('/comm_vel', Twist, queue_size=10)

		#wait for lanes and current_pose to be ready
		while not comm.nodes or not comm.current_pose:
			pass

		self.checker = CommChecker()
		self.receiver = CommReceiver(self.comm_vel_pub)


	def _nodes_handler(self, msg):
		for lane in msg.lanes:
			comm.nodes.append(dict(x=lane.end.x, y=lane.end.y, id=lane.end_id))
		rospy.loginfo('Nodes: '+str(comm.nodes))

	def _vel_handler(self, msg):
		comm.current_vel = msg.velocity.x


	def _pose_handler(self, msg):
		pose = msg.pose.pose
		_, _, angle = euler_from_quaternion(
			[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
		comm.current_pose = dict(x=pose.position.x, y=pose.position.y, theta=angle)
		#rospy.loginfo('Current pose: '+str(comm.current_pose))

	def start(self):
		self.checker.start()
		self.receiver.start()


if __name__ == '__main__':
    try:
        communicator = Communicator()
       	communicator.start()
    
    except rospy.ROSInterruptException:
        rospy.loginfo("Communicator checker finished.")






