#!/usr/bin/env python

import rospy
from nist_team.gantry_mover import GantryMover


def main():
    rospy.init_node("gantry_mover")

    mover = GantryMover()

    rospy.spin()


if __name__ == "__main__":
    main()
