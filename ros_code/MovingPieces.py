from math import pi
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from hiro_core.XamyabRobot import XamyabRobot, rospy
from collections import deque
import random

class MovingPieces:
    def __init__(self):
        self.robot = XamyabRobot(visualize_trajectory=True)
        self.default_gripper_quaternion = Quaternion(*quaternion_from_euler(pi, 0, pi/2))
        self.viewing_gripper_quaternion = Quaternion(*quaternion_from_euler(-pi/2, 0, pi/2))
        self.default_pos = Pose(position=Point(*[0.6256, 0.50, 0.2]), orientation=self.default_gripper_quaternion)
    def run(self, j):
        #self.reset()
        rospy.loginfo("PROJECT READY")
        for i in range(0,j):
            self.move_object(i)
    def test(self):
        rospy.loginfo("CLOSING GRIPPER 75%")
        self.robot.right_gripper.set_value(75)
    def reset(self):
        self.robot.right_gripper.open()
        rospy.loginfo("Going home")
        self.robot.right_manipulator.home()
    def move_to(self, pose_goal):
        pose_goal.position.z +=0.2
        pose_goal.orientation = self.default_gripper_quaternion
        self.robot.right_manipulator.set_pose_goal(pose_goal)
    def move_object(self,i):
        
        pose = self.pick_square_piece_pose(i)
        rospy.loginfo("Grabbing Object Number #" + str(i))
        self.move_to(pose)
        self.robot.right_gripper.close()
        rospy.loginfo("Reading Shape")
        self.move_to_cv()
        shape = self.identify_shape()
        pose = self.pick_final_square_pose(i)
        rospy.loginfo("Placing Object Number #" + str(i))
        self.move_to(pose)
        print(pose)
        self.robot.right_gripper.open()
        rospy.sleep(1)
    def pick_square_piece_pose(self, index):
        top_right_corner_square = [0.7, -0.7, 0.4]
        list_points = []
        for i in range(0,5):
            for j in range(0,5):
                list_points.append([0.7-0.035*i, -0.7-0.035*j, 0.4])
        transform_point = list_points[index]
        pose = Pose(position=Point(*transform_point))
        return pose
    def identify_shape(self):
        #Krishna's code does this
        return random.randint(0,24)
    def pick_final_square_pose(self, index):
        bottom_right_corner_square = [0.7, -0.3, 0.4]
        list_points = []
        for i in range(0,5):
            for j in range(0,5):
                list_points.append([0.7+0.035*i, -0.3+0.035*j, 0.4])
        transform_point = list_points[index]
        pose = Pose(position=Point(*transform_point))
        return pose
    def move_to_cv(self):
        transform_point = [0.7, 0, 0.5]
        pose_goal  = Pose(position = Point(*transform_point))
        pose_goal.position.z +=0.2
        pose_goal.orientation = self.viewing_gripper_quaternion
        self.robot.right_manipulator.set_pose_goal(pose_goal)
    def current_pos(self):
        print(self.robot.right_manipulator.get_current_pose())

if __name__ == '__main__':
    project = MovingPieces()
    #project.run(2)
    #project.test()
    project.current_pos()