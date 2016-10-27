#!/usr/bin/env python

import rospy
from smores_reconfiguration import smores_reconfiguration_planner

class SMORESReconfigurationNode:
    def __init__(self):
        self.smores_reconfiguration_planner = None
        self._initialize()

    def _initialize(self):
        # Start the ros node
        rospy.init_node('SMORESReconfiguration', anonymous=True, log_level=rospy.DEBUG)
        rospy.loginfo("Start")

        self.smores_reconfiguration_planner = smores_reconfiguration_planner.SMORESReconfigurationPlanner()

    def main(self):
        rospy.loginfo("Begin Reconfiguration")
        self.smores_reconfiguration_planner.main()
        rospy.loginfo("Reconfiguration Finished")

if __name__ == "__main__":
    SRN = SMORESReconfigurationNode()
    SRN.main()
