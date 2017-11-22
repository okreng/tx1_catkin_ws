#!/usr/bin/env python
import rospy

#from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

# import some utils.
import numpy as np
import copy as copy


class InterpolateThrottle:
    def __init__(self):

        # Allow our topics to be dynamic.
        self.input_topic = rospy.get_param('~output_topic', '/joy_vel_raw')
        self.output_topic = rospy.get_param('~input_topic', '/joy_vel')
        self.max_delta_vel = rospy.get_param('~max_delta_vel')
        self.max_vel = rospy.get_param('~speed_max')
        self.min_vel = rospy.get_param('~speed_min')

        # Create topic subscribers and publishers
        self.output = rospy.Publisher(self.output_topic, Twist,queue_size=1)
        rospy.Subscriber(self.input_topic, Twist, self._process_throttle_command)
        
	# Variables
	self.last_vel = 0

	# run the node
        self._run()

    # Keep the node alive
    def _run(self):
        rospy.spin()
            
    def _process_throttle_command(self,input):
        input_vel = input.linear.x
        input_ang = input.angular.z

	# Do some sanity clipping
	input_vel = min(max(input_vel, self.min_vel), self.max_vel)

	desired_delta = input_vel-self.last_vel
	# Bound the delta
	clipped_delta = max(min(desired_delta, self.max_delta_vel), -self.max_delta_vel)
	smoothed_vel = self.last_vel + clipped_delta
	self.last_vel = smoothed_vel
	out_vel = Twist()
	out_vel.linear.x = smoothed_vel
	out_vel.angular.z = input_ang
	self.output.publish(out_vel)

# Boilerplate node spin up. 
if __name__ == '__main__':
    try:
        rospy.init_node('Throttle_Interpolator')
        p = InterpolateThrottle()
    except rospy.ROSInterruptException:
        pass
