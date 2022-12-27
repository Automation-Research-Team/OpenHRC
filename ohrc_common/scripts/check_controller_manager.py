#!/usr/bin/env python

import rospy

def main():
    rospy.init_node('controller_manager_checker')

    try:
        rospy.wait_for_service('controller_manager/list_controllers', timeout=5.0)
        rospy.loginfo('Controller manager is successfully launched!')

    except rospy.ROSException as e:
        rospy.logfatal('Controllers manager is not launched!')
        return

    rospy.spin() # wait for shutdown

if __name__ == '__main__':
    main()
