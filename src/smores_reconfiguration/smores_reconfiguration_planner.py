import tf
import rospy
import time
from numpy import sqrt
from math import pi
from smores_controller import smores_controller
from nav_msgs.msg import Path
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

        self._initialize()

    def _initialize(self):
        self.tf = tf.TransformListener()
        self.reconf_waitlist = [
                {"move_m":"tag_1",
                 "target_m":"tag_0",
                 "undock_m":"tag_0",
                 "target_connect":"top",
                 "move_disconnect":"bottom",
                 "move_connect":"bottom",
                 "undock_disconnect":"left"
                },
                {"move_m":"tag_2",
                 "target_m":"tag_1",
                 "undock_m":"tag_0",
                 "target_connect":"top",
                 "move_disconnect":"bottom",
                 "move_connect":"bottom",
                 "undock_disconnect":"right"
                },
                ]
        self.up_angle = {21:0*pi/180, 15:10*pi/180, 4:15*pi/180}
        self.down_angle = {21:0*pi/180, 15:3*pi/180, 4:5*pi/180}
        self.tag_module_mapping = {"tag_0": 21, "tag_1": 15, "tag_2": 4}
        self.smores_list = self.tag_module_mapping.values()

        rospy.Subscriber('/smorePath', Path, self._receivePathCallback)
        rospy.Subscriber('/reconf_signal', String, self._reconf_signal_callback)
        self.reconf_pub = rospy.Publisher("/reconf_request", String, queue_size=10)

        rospy.sleep(0.3)

    def _receivePathCallback(self, path):
        rospy.logdebug("Received a path ...")
        self.path = path
        self._finished_path = False

    def _reconf_signal_callback(self, data):
        print "Recieve {}".format(data)
        self.do_reconf = True

    def getTagPosition(self, tag_id):
        rospy.logdebug("Getting position for {!r}".format(tag_id))
        while not rospy.is_shutdown():
            try:
                return self.tf.lookupTransform('/base_link', tag_id, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print e

    def _driveToTargetPoint(self, move_tag, pt_x, pt_y, timeout = 10.0, near_enough = 0.01):

        arrived = False
        error_code = ""
        start_time = time.time()
        move_module_id = self.tag_module_mapping[move_tag]

        while not arrived:

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
            theta = tf.transformations.euler_from_quaternion(move_rot, 'rxyz')[2]

            [v,w] = self.smores_controller.global2Local(x, y, theta)
            self.smores_controller.driveWithLocal(move_module_id, v, w)

            if ((sqrt(x**2+y**2)<near_enough)):
                rospy.logdebug("Target point arrived.")
                arrived = True

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

        # Lift up the front wheel for better driving
        move_module_obj.move.command_position("tilt", self.up_angle[move_module_id], 2)
        time.sleep(3)
        move_module_obj.move.send_torque("tilt", 0.0)

        # Reset the front wheel for better docking
        target_module_obj.move.command_position("pan", 0, 4)
        time.sleep(4)
        target_module_obj.move.send_torque("pan", 0.0)

        # Drop the front wheel for better docking
        target_module_obj.move.command_position("tilt", self.down_angle[target_module_id], 2)
        time.sleep(3)
        target_module_obj.move.send_torque("tilt", 0.0)

        # Drive the move module out to avoid collision
        self.smores_controller.driveForward(move_module_obj)
        self.smores_controller.driveForward(move_module_obj)
        time.sleep(1.5)
        self.smores_controller.stopAllMotors(move_module_obj)
        time.sleep(0.1)

    def _preDock(self, reconf_data):
        move_module_id = self.tag_module_mapping[reconf_data["move_m"]]
        move_module_obj = self.smores_controller.getModuleObjectFromID(move_module_id)
        # Put down the back plate for better docking
        move_module_obj.move.command_position("tilt", self.down_angle[move_module_id], 2)
        time.sleep(3)
        move_module_obj.move.send_torque("tilt", 0.0)
        time.sleep(0.1)

    def _postReconf(self, reconf_data):
        move_module_id = self.tag_module_mapping[reconf_data["move_m"]]
        move_module_obj = self.smores_controller.getModuleObjectFromID(move_module_id)
        move_module_connect = reconf_data["move_connect"]
        move_module_disconnect = reconf_data["move_disconnect"]
        target_module_id = self.tag_module_mapping[reconf_data["target_m"]]
        target_module_obj = self.smores_controller.getModuleObjectFromID(target_module_id)
        target_module_connect = reconf_data["target_connect"]

        # Turn off all motor for magnet control
        self.smores_controller.stopAllMotors(move_module_obj)
        time.sleep(0.1)
        self.smores_controller.stopAllMotors(target_module_obj)

        # Turn on magnects for new connections
        move_module_obj.mag.control(move_module_connect, "on")
        time.sleep(0.1)
        target_module_obj.mag.control(target_module_connect, "on")
        time.sleep(0.1)

        time.sleep(0.1)

    def main(self):
        rate = rospy.Rate(10)
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
                msg = String()
                msg.data = "{}:{}:{}:{}".format(reconf_data["move_m"],reconf_data["move_connect"],
                                                reconf_data["target_m"],reconf_data["target_connect"])

                rospy.logdebug("Preparing for reconfiguration ...")
                self._preReconf(self.reconf_waitlist[self._current_waitlist_id])

                rospy.logdebug("Sending a request for path ...")
                self.reconf_pub.publish(msg)

                while self._finished_path and not rospy.is_shutdown():
                    rospy.logdebug("Waiting for a path ...")
                    rate.sleep()
            else:
                rospy.logdebug("Start to follow a path ...")
                for pt_id, pose_stamped in enumerate(self.path.poses):
                    x = pose_stamped.pose.position.x
                    y = pose_stamped.pose.position.y

                    move_tag = self.reconf_waitlist[self._current_waitlist_id]["move_m"]
                    if pt_id  == (len(self.path.poses) - 1):
                        self._preDock(self.reconf_waitlist[self._current_waitlist_id])
                        self._driveToTargetPoint(move_tag, x, y, timeout = 10.0)
                    else:
                        self._driveToTargetPoint(move_tag, x, y)

                rospy.logdebug("Finishing docking ...")
                self._postReconf(self.reconf_waitlist[self._current_waitlist_id])
                time.sleep(10)
                self.path = None
                self._finished_path = True
                self._current_waitlist_id += 1

            if self._current_waitlist_id == len(self.reconf_waitlist):
                rospy.logdebug("Reconfiguration Finished!")
                pub = rospy.Publisher("/reconf_status", String, queue_size=10)
                for i in xrange(3):
                    pub.publish(String("done"))
                    rospy.sleep(1)
                break


        # Stop all modules
        for i in self.smores_list:
            self.smores_controller.stopAllMotors(self.smores_controller.getModuleObjectFromID(i))
