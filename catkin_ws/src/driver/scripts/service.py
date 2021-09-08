#!/usr/bin/env python

import rospy
from driver.srv import *

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import math





# robot state variables
position_ = Point()
yaw_ = 0
frontdistance = 1000
# machine state
state_ = 0
# goal
desired_position_ = Point()

# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.3

# publishers
pub = None

# service callbacks
def go_to_point_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

# Odometry callback
def clbk_odom(msg):
    global position_, yaw_
    # position
    position_ = msg.pose.pose.position
    # Getting yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

    drive()


def change_state(state):
    global state_
    state_ = state
    print 'State changed to [%s]' % state_

# Extreme case rotation
def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

# Setting yaw of bot
def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    
    rospy.loginfo(err_yaw)
    
    if not(frontdistance < 1.5):
        twist_msg = Twist()
        if math.fabs(err_yaw) > yaw_precision_:
            if err_yaw > 0:
                twist_msg.angular.z = 0.7
                twist_msg.linear.x = 0.09
            else:
                twist_msg.linear.x = 0.09
                twist_msg.angular.z = -0.7
        
        pub.publish(twist_msg)
    else:
        pass
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        print 'Yaw error: [%s]' % err_yaw
        change_state(1)

# Going to point after setting the yaw
def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    
    if not(frontdistance < 1.5):
        if err_pos > dist_precision_:
            twist_msg = Twist()
            twist_msg.linear.x = -0.8
            twist_msg.angular.z = 0.2 if err_yaw > 0 else -0.2
            pub.publish(twist_msg)
        else:
            print 'Position error: [%s]' % err_pos
            change_state(2)
    else:
        pass
    
    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print 'Yaw error: [%s]' % err_yaw
        change_state(0)

# Point reached condition
def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)
    change_state(0)


def drive():
    global state_
    if state_ == 0:
        fix_yaw(desired_position_)
    elif state_ == 1:
        go_straight_ahead(desired_position_)
    elif state_ == 2:
        done()
    else:
        pass        


def main():
    global pub, active_
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    


def maincallback(dest):
    global desired_position_
    print dest
    desired_position_.x = dest.x
    desired_position_.y = dest.y
    main()
    return "Uploaded"




if __name__ == '__main__':
    rospy.init_node('waypoints_service')
    s = rospy.Service('/givewaypoints', waypoints, maincallback)
    rospy.spin()

