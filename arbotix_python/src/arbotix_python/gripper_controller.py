#!/usr/bin/env python

import rospy, actionlib

from controllers import *

from control_msgs.msg import GripperCommandAction, GripperCommand

MAX_ACTION_DURATION = 0.5  # seconds


class GripperController(Controller):
    def __init__(self, device, name):
        Controller.__init__(self, device, name)

        # parameters: rates and joints
        self.rate = rospy.get_param('~controllers/' + name + '/rate', 50.0)
        self.joints = rospy.get_param('~controllers/' + name + '/joints')
        self.index = rospy.get_param('~controllers/' + name + '/index', len(device.controllers))
        self.max_range = 0.037  # in m

        for joint in self.joints:
            self.device.joints[joint].controller = self

        # action server
        name = rospy.get_param('~controllers/' + name + '/action_name', 'gripper_command')
        self.server = actionlib.SimpleActionServer(name, GripperCommandAction, execute_cb=self.actionCb,
                                                   auto_start=False)
        rospy.Subscriber(self.name + '/command', GripperCommand, self.commandCb)

        self.executing = False

    def startup(self):
        self.server.start()

    def active(self):
        """ Is controller overriding servo internal control? """
        return self.server.is_active() or self.executing

    def actionCb(self, goal):
        rospy.loginfo(self.name + ": Action goal recieved: " + str(goal))
        try:
            result = self.setClampPosition(goal.command)
        except:
            rospy.logerr("Failed to set clamp position: %s" % sys.exc_info()[0])
            result = -1

        # set success
        if result == 1:
            self.server.set_succeeded()
        elif result == 0:
            self.server.set_aborted()
        else:
            self.server.set_preempted()

    def commandCb(self, cmd):
        self.setClampPosition(cmd)

    def setClampPosition(self, command):
        # 2,61799 = 150 degrees
        #servo_max_value = 2.61799
        #fakeDegrees = command.position * (2 * servo_max_value / self.max_range) - servo_max_value
        target = command.position  #fakeDegrees

        # try:
        #     indexes = [traj.joint_names.index(joint) for joint in self.joints]
        # except ValueError as val:
        #     rospy.logerr("Invalid joint in trajectory.")
        #     return -1

        start = rospy.Time.now()

        r = rospy.Rate(self.rate)
        last = [self.device.joints[joint].position for joint in self.joints]
        while rospy.Time.now() + rospy.Duration(0.01) < start:
            if self.server.is_preempt_requested():
                return 0
            rospy.sleep(0.01)

        # calculate the fake degree values for the servo controller
        # desired = [target for i in self.joints]
        desired = []
        for joint_name in self.joints:
            servo = self.device.joints[joint_name]  #.controller
            servo_span = servo.max_angle - servo.min_angle
            d = target * (servo_span / self.max_range) + servo.min_angle
            desired.append(d)

        # velocity = [self.device.joints[joint_name].max_speed for joint_name in self.joints]

        endtime = start + rospy.Duration(MAX_ACTION_DURATION)
        while rospy.Time.now() + rospy.Duration(0.01) < endtime:
            # check that preempt has not been requested by the client
            if self.server.is_preempt_requested():
                return 0

            err = [(d - c) for d, c in zip(desired, last)]
            velocity = [abs(x / (self.rate * (endtime - rospy.Time.now()).to_sec())) for x in err]

            rospy.logdebug(err)
            for i in range(len(self.joints)):
                if err[i] > 0.001 or err[i] < -0.001:
                    cmd = err[i]
                    top = velocity[i]
                    if cmd > top:
                        cmd = top
                    elif cmd < -top:
                        cmd = -top
                    last[i] += cmd
                    tickiditicks = self.device.joints[self.joints[i]].setControlOutput(last[i])
                    # rospy.loginfo(self.joints[i] + " => " + str(tickiditicks))
                else:
                    velocity[i] = 0
            r.sleep()

        return 0;


if __name__ == "__main___":
    pass
