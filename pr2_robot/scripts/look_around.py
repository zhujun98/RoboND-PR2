#!/usr/bin/env python
import math

import rospy

from std_msgs.msg import Float64


def look_around():
    """Rotate the robot around
    
    To the left first, then to the right, and finally return to the 
    center.
    """
    pub_joint = rospy.Publisher('/pr2/world_joint_controller/command',
                                Float64, queue_size= 10)

    rate = rospy.Rate(5)

    start_time = 0
    while start_time == 0:
        start_time = rospy.Time.now().to_sec()    

    while not rospy.is_shutdown():
        elapsed = rospy.Time.now().to_sec() - start_time
        # TODO: Add a pose check here
        if elapsed < 16:
            print("Checking the left side...")
            pub_joint.publish(math.pi/2.)
        elif elapsed < 48:
            print("Checking the right side...")
            pub_joint.publish(-math.pi/2.)
        elif elapsed < 64:
            print("Returning to the center...")
            pub_joint.publish(0)
        else:
            break;

        rate.sleep()


if __name__ == '__main__':
    rospy.init_node("look_around", anonymous=True)

    try:
        look_around()
    except rospy.ROSInterruptException:
        pass
