import tf
import rospy
import time
import os
import yaml
from numpy import sqrt
from math import pi
from smores_controller import smores_controller
import rospkg
from std_msgs.msg import String

class SMORESReconfigurationPlanner:
    def __init__(self):
        self.tf = None
        self.smores_controller = None
        self.reconf_waitlist = []
        self._current_waitlist_id = 0
        self.tag_module_mapping = {}
        self.reconf_pub = None
        self._finished_path = True
        self.path = None
        self._repeat_cmd_num = 3
        self.do_reconf = False
        self.path_dict = {
                         "To_Prob_tag_1":[],
                         "To_Prob_tag_2":[],
                         "From_Prob_tag_1":[],
                         "From_Prob_tag_2":[]
                         }
        self.reconf_order_data = {}
        self.reconf_path_data_path = ""
        self._current_reconf_direction=""

        self._initialize()

    def _initialize(self):
        self.tf = tf.TransformListener()
        self.reconf_order_data = {
                "T2P":[
                {"move_m":"tag_1",
                 "target_m":"tag_0",
                 "undock_m":"tag_0",
                 "target_connect":"top",
                 "move_disconnect":"bottom",
                 "move_connect":"bottom",
                 "move_heading":0.0,
                 "undock_disconnect":"left"
                },
                {"move_m":"tag_2",
                 "target_m":"tag_1",
                 "undock_m":"tag_0",
                 "target_connect":"top",
                 "move_disconnect":"bottom",
                 "move_connect":"bottom",
                 "move_heading":0.0,
                 "undock_disconnect":"right"
                }
                ],
                "P2T":[
                {"move_m":"tag_2",
                 "target_m":"tag_0",
                 "undock_m":"tag_1",
                 "target_connect":"right",
                 "move_disconnect":"bottom",
                 "move_connect":"bottom",
                 "move_heading":-pi/2,
                 "undock_disconnect":"top"
                },
                {"move_m":"tag_1",
                 "target_m":"tag_0",
                 "undock_m":"tag_0",
                 "target_connect":"left",
                 "move_disconnect":"bottom",
                 "move_connect":"bottom",
                 "move_heading":pi/2,
                 "undock_disconnect":"top"
                }
                ]
                }

        self.up_angle = {21:10*pi/180, 15:10*pi/180, 4:10*pi/180, 1:10*pi/180, 11:10*pi/180, 18:10*pi/180, 3:10*pi/180, 13:25*pi/180, 7:10*pi/180}
        self.neutral_angle = {21:0*pi/180, 15:5*pi/180, 4:-5*pi/180, 1:0*pi/180, 11:-5*pi/180, 18:0*pi/180, 3:0*pi/180, 13:20*pi/180, 7:0*pi/180}
        self.tag_module_mapping = {"tag_0":18, "tag_1":11, "tag_2":15}
        #self.tag_module_mapping = {"tag_0": 1, "tag_1": 4, "tag_2": 11}
        self.smores_list = self.tag_module_mapping.values()

        rospy.Subscriber('/reconf_signal', String, self._reconf_signal_callback)
        self.reconf_path_data_path = os.path.join(rospkg.RosPack().get_path("smores_reconfiguration"), "data", "reconf_path.yaml")
        self.loadReconfPath()

        rospy.sleep(0.3)

    def loadReconfPath(self):
        rospy.loginfo("Loading path file from {}...".format(self.reconf_path_data_path))
        with open(self.reconf_path_data_path, "r") as pathfile:
            self.path_dict = yaml.load(pathfile)


    def _receivePathCallback(self, path):
        rospy.logdebug("Received a path ...")
        self.path = path
        self._finished_path = False

    def _reconf_signal_callback(self, data):
        data = str(data.data)
        if data == "P2T" or data == "T2P":
            self.reconf_waitlist = self.reconf_order_data[data]
            self._current_reconf_direction = data
            self.do_reconf = True
        else:
            rospy.logwarn("Cannot recognize {}".format(data))

    def getTagPosition(self, tag_id):
        rospy.logdebug("Getting position for {!r}".format(tag_id))
        while not rospy.is_shutdown():
            try:
                return self.tf.lookupTransform("tag_0", tag_id, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print e

    def _driveToTargetPoint(self, move_tag, pt_x, pt_y, timeout = 30.0, near_enough = 0.01):

        arrived = False
        error_code = ""
        start_time = time.time()
        move_module_id = self.tag_module_mapping[move_tag]
        rate = rospy.Rate(2)

        while not arrived:
            rate.sleep()
            if (time.time() - start_time) > timeout:
                rospy.logwarn("Drive to point timed out.")
                error_code = "TIMEOUT"
                break

            if (rospy.is_shutdown()):
                rospy.logwarn("Drive to point interupted.")
                error_code = "INTERUPT"
                break

            rospy.loginfo("Driving to point {}".format([pt_x, pt_y]))
            try:
                (move_pose, move_rot) = self.getTagPosition(move_tag)
            except TypeError as e:
                rospy.logerr("Cannot find position for {!r}: {}".format(move_tag, e))
                self.smores_controller.stopAllMotors(move_module_id)
                continue

            x = pt_x - move_pose[0]
            y = pt_y - move_pose[1]
            rospy.loginfo("Pose is {} and {}".format(move_pose[0], move_pose[1]))
            theta = tf.transformations.euler_from_quaternion(move_rot, 'rxyz')[2]

            [v,w] = self.smores_controller.global2Local(x, y, theta)
            self.smores_controller.driveWithLocal(move_module_id, v, w)

            if ((sqrt(x**2+y**2)<near_enough)):
                rospy.logdebug("Target point arrived.")
                arrived = True

        self.smores_controller.stopAllMotors(move_module_id)
        time.sleep(0.1)
        self.smores_controller.stopAllMotors(move_module_id)
        time.sleep(0.1)
        self.smores_controller.stopAllMotors(move_module_id)
        return arrived, error_code

    def _preReconf(self, reconf_data):
        move_module_id = self.tag_module_mapping[reconf_data["move_m"]]
        move_module_obj = self.smores_controller.getModuleObjectFromID(move_module_id)
        move_module_connect = reconf_data["move_connect"]
        move_module_disconnect = reconf_data["move_disconnect"]
        target_module_id = self.tag_module_mapping[reconf_data["target_m"]]
        target_module_obj = self.smores_controller.getModuleObjectFromID(target_module_id)
        target_module_connect = reconf_data["target_connect"]
        undock_module_id = self.tag_module_mapping[reconf_data["undock_m"]]
        undock_module_obj = self.smores_controller.getModuleObjectFromID(undock_module_id)
        undock_module_disconnect = reconf_data["undock_disconnect"]

        # Turn off all motor for magnet control
        for i in xrange(self._repeat_cmd_num):
            self.smores_controller.stopAllMotors(move_module_obj)
            time.sleep(0.1)
            self.smores_controller.stopAllMotors(target_module_obj)
            time.sleep(0.1)
            self.smores_controller.stopAllMotors(undock_module_obj)
            time.sleep(0.1)

        # Disconnect undock module magnets
        for i in xrange(self._repeat_cmd_num):
            move_module_obj.mag.control(move_module_disconnect, "off")
            time.sleep(0.1)
            undock_module_obj.mag.control(undock_module_disconnect, "off")
            time.sleep(0.1)

        # Stop for a bit before drive
        time.sleep(0.5)

        # Drop down the front wheel to detach from others
        for i in xrange(self._repeat_cmd_num):
            move_module_obj.move.command_position("tilt", -45*pi/180, 3)
            time.sleep(0.1)
        time.sleep(3)

        # Lift up the front wheel for better driving
        for i in xrange(self._repeat_cmd_num):
            move_module_obj.move.command_position("tilt", self.up_angle[move_module_id], 2)
            time.sleep(0.1)

        time.sleep(3)

        for i in xrange(self._repeat_cmd_num):
            move_module_obj.move.send_torque("tilt", 0.0)
            time.sleep(0.1)

        # Reset the front wheel for better docking
        if target_module_connect == "top":
            target_module_obj.move.command_position("pan", 0, 4)
            time.sleep(4)
            target_module_obj.move.send_torque("pan", 0.0)

        # Drive the move module out to avoid collision
        for i in xrange(self._repeat_cmd_num):
            self.smores_controller.driveForward(move_module_obj)
            time.sleep(0.1)
        time.sleep(2.0)
        self.smores_controller.stopAllMotors(move_module_obj)
        time.sleep(0.1)

    def _preDock(self, reconf_data):
        move_module_id = self.tag_module_mapping[reconf_data["move_m"]]
        move_module_obj = self.smores_controller.getModuleObjectFromID(move_module_id)
        move_module_connect = reconf_data["move_connect"]
        move_module_disconnect = reconf_data["move_disconnect"]
        target_module_id = self.tag_module_mapping[reconf_data["target_m"]]
        target_module_obj = self.smores_controller.getModuleObjectFromID(target_module_id)
        target_module_connect = reconf_data["target_connect"]
        move_heading = reconf_data["move_heading"]

        # Correct the heading
        self.correctHeading(reconf_data["move_m"], move_module_obj, move_heading)

        for i in xrange(self._repeat_cmd_num):
            move_module_obj.move.command_position("tilt", self.neutral_angle[move_module_id], 2)
            time.sleep(0.1)
        time.sleep(3)

        # Turn off all motor for magnet control
        for i in xrange(self._repeat_cmd_num):
            self.smores_controller.stopAllMotors(move_module_obj)
            time.sleep(0.1)
            self.smores_controller.stopAllMotors(target_module_obj)
            time.sleep(0.1)

        # Turn on magnects for new connections
        for i in xrange(self._repeat_cmd_num):
            move_module_obj.mag.control(move_module_connect, "on")
            time.sleep(0.1)
            target_module_obj.mag.control(target_module_connect, "on")
            time.sleep(0.1)


    def _postReconf(self, reconf_data):
        move_module_id = self.tag_module_mapping[reconf_data["move_m"]]
        move_module_obj = self.smores_controller.getModuleObjectFromID(move_module_id)
        move_module_connect = reconf_data["move_connect"]
        move_module_disconnect = reconf_data["move_disconnect"]
        target_module_id = self.tag_module_mapping[reconf_data["target_m"]]
        target_module_obj = self.smores_controller.getModuleObjectFromID(target_module_id)
        target_module_connect = reconf_data["target_connect"]

        # Wiggle the tilt for better docking
        for i in xrange(self._repeat_cmd_num):
            self.smores_controller.driveBackward(move_module_obj)
            time.sleep(0.1)
        for i in xrange(self._repeat_cmd_num):
            move_module_obj.move.command_position("tilt", -15*pi/180 + self.neutral_angle[move_module_id], 2)
            time.sleep(0.1)
        time.sleep(2)
        for i in xrange(self._repeat_cmd_num):
            move_module_obj.move.command_position("tilt", self.neutral_angle[move_module_id], 2)
            time.sleep(0.1)
        time.sleep(3)

        # Turn off all motor for magnet control
        for i in xrange(self._repeat_cmd_num):
            self.smores_controller.stopAllMotors(move_module_obj)
            time.sleep(0.1)
            self.smores_controller.stopAllMotors(target_module_obj)
            time.sleep(0.1)

        # Turn on magnects for new connections
        for i in xrange(self._repeat_cmd_num):
            move_module_obj.mag.control(move_module_connect, "on")
            time.sleep(0.1)
            target_module_obj.mag.control(target_module_connect, "on")
            time.sleep(0.1)

    def getTagOrientation(self, move_tag):
        try:
            (move_pose, move_rot) = self.getTagPosition(move_tag)
        except TypeError as e:
            rospy.logerr("Cannot find position for {!r}: {}".format(move_tag, e))
            return
        return tf.transformations.euler_from_quaternion(move_rot, 'rxyz')[2]

    def correctHeading(self, move_tag, move_module_obj, target_angle, close_enough=0.2):
        rospy.loginfo("Correct heading of {}".format(move_tag))
        angle = 2*pi
        self.smores_controller.spin(move_module_obj)
        rospy.sleep(0.1)
        self.smores_controller.spin(move_module_obj)
        rate = rospy.Rate(5)
        while not rospy.is_shutdown() and abs(target_angle - angle)>close_enough:
            angle = self.getTagOrientation(move_tag)
            rospy.loginfo("diff is {}".format(abs(target_angle-angle)))
            rate.sleep()
        rospy.loginfo("Heading is good!")
        self.smores_controller.stopAllMotors(move_module_obj)
        rospy.sleep(0.1)
        self.smores_controller.stopAllMotors(move_module_obj)

    def main(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            rate.sleep()
            if not self.do_reconf:
                rospy.loginfo("Waiting for reconf signal ...")
                continue

            if self.smores_controller is None:
                self.smores_controller = smores_controller.SMORESController(self.smores_list)

            if self._finished_path:
                self.path = None
                reconf_data = self.reconf_waitlist[self._current_waitlist_id]

                rospy.logdebug("Preparing for reconfiguration ...")
                self._preReconf(self.reconf_waitlist[self._current_waitlist_id])

                rospy.logdebug("Loading reconf path ...")
                self.path = self.path_dict[self._current_reconf_direction + "_" + reconf_data["move_m"]]
                self._finished_path = False
            else:
                rospy.logdebug("Start to follow a path ...")
                for pt_id, pt in enumerate(self.path):
                    x = pt[0]/100.0
                    y = pt[1]/100.0

                    move_tag = self.reconf_waitlist[self._current_waitlist_id]["move_m"]

                    if pt_id  == (len(self.path) - 1):
                        # Do a spin

                        self._preDock(self.reconf_waitlist[self._current_waitlist_id])
                        self._driveToTargetPoint(move_tag, x, y, timeout = 5.0)
                    else:
                        self._driveToTargetPoint(move_tag, x, y)

                    #raw_input("Arrived at point {},{}".format(x,y))

                rospy.logdebug("Finishing docking ...")
                self._postReconf(self.reconf_waitlist[self._current_waitlist_id])
                time.sleep(10)
                self.path = None
                self._finished_path = True
                self._current_waitlist_id += 1

            if self._current_waitlist_id == len(self.reconf_waitlist):
                rospy.logdebug("Reconfiguration Finished!")
                pub = rospy.Publisher("/reconf_status", String, queue_size=10)
                for i in xrange(10):
                    pub.publish(String("done"))
                    rospy.sleep(0.1)
                break


        # Stop all modules
        if self.smores_controller is None:
            return
        for i in self.smores_list:
            self.smores_controller.stopAllMotors(self.smores_controller.getModuleObjectFromID(i))

