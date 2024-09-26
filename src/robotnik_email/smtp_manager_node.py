#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from smtp_manager import SMTPManager


def main():

    rospy.init_node("smtp_manager_node")

    rc_node = SMTPManager()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()
