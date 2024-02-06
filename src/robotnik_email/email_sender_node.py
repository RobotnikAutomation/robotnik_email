#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from email_sender import SimpleEmailSender


def main():

    rospy.init_node("simple_email_sender")

    rc_node = SimpleEmailSender()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()
