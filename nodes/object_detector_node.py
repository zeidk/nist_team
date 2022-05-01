#!/usr/bin/env python

import rospy
from nist_team.object_detector import ObjectDetector


def main():
    rospy.init_node("object_detector")

    detector = ObjectDetector()

    rospy.spin()


if __name__ == "__main__":
    main()
