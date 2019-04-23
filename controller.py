# coding=utf-8
import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import socket
from math import pi

ROBOT_IP = '172.30.1.1'
ROBOT_PORT = 1978
SOCKET_TIMEOUT = 2
ROS_RATE = 10

def radians_to_degrees(angle):
    return int(3600*angle/(2*pi))

def time_in_sec(time_from_start):
    return time_from_start.secs + time_from_start.nsecs/1e9

class Controller:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(SOCKET_TIMEOUT)
        self.joint_trajectory = rospy.Subscriber('/p04/p04_joint_controller/follow_joint_trajectory/goal',
                                                 FollowJointTrajectoryActionGoal, self.joint_trajectory_callback)
        self.gripper_trajectory = rospy.Subscriber('/p04/gripper_controller/follow_joint_trajectory/goal',
                                                   FollowJointTrajectoryActionGoal, self.gripper_trajectory_callback)
        rospy.init_node('p04_controller_server')
        self.rate = rospy.Rate(ROS_RATE)
        self.open_socket()
        while not rospy.is_shutdown():
            self.rate.sleep()

    def open_socket(self):
        self.sock.connect((ROBOT_IP, ROBOT_PORT))

    def send_message(self, message):
        self.sock.send(message.encode())

    def send_angle(self, joint_number, angle):
        self.send_message("robot:goto j{} {}".format(joint_number, angle))

    def send_gripper_state(self, state):
        self.send_message("robot:gripper {}".format(state))

    def get_gripper_state(self, pos):
        if pos < 0.001:
            return "open"
        elif pos > 0.02:
            return "close"
        else:
            return "release"

    def joint_trajectory_callback(self, msg):
        data = msg  # type: FollowJointTrajectoryActionGoal
        points = data.goal.trajectory.points
        for idx, p in enumerate(points):
            point = p  # type: JointTrajectoryPoint
            angles_degrees = list(map(radians_to_degrees, point.positions))
            for idx, angle in enumerate(angles_degrees):
                print(idx+1, angle)
                self.send_angle(idx+1, angle)
                rospy.sleep(0.01)
            # print(idx, time_in_sec(point.time_from_start))
    def gripper_trajectory_callback(self, msg):
        # gripper is open when pos is approx 0
        # closed when approx 0.026
        data = msg  # type: FollowJointTrajectoryActionGoal
        points = data.goal.trajectory.points
        for p in points:
            point = p  # type: JointTrajectoryPoint
            gripper_state = self.get_gripper_state(point.positions[0])
            delay = time_in_sec(point.time_from_start)
            rospy.sleep(delay)
            self.send_gripper_state(gripper_state)

Controller()
