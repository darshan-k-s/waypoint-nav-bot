#!/usr/bin/env python

import rospy
from driver.srv import waypoints


rospy.init_node('waypoints_client')

rospy.wait_for_service('/givewaypoints')

s = rospy.ServiceProxy('/givewaypoints', waypoints)

result = s(-8, -8)

print result

