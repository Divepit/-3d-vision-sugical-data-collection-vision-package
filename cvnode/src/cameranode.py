#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

if __name__ == '__main__':
    # Initialise the node
    rospy.init_node("plain_py_node")
    # Display the namespace of the node handle
    rospy.loginfo("PLAIN PY NODE] namespace of node = " + rospy.get_namespace());
    # Spin as a single-threaded node
    rospy.spin()