from math import pi
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from hiro_core.XamyabRobot import XamyabRobot, rospy
from collections import deque
import random
import classify_shape
import cv2 
import numpy as np

class MovingPieces:
    def __init__(self):
        self.robot = XamyabRobot(visualize_trajectory=False)
        self.default_gripper_quaternion = Quaternion(*quaternion_from_euler(pi, 0, 0))
        self.viewing_gripper_quaternion = Quaternion(*quaternion_from_euler(pi, 0, 0))
        self.start_pos = Pose(position=Point(*[0.6256, 0, 0.4]), orientation=self.default_gripper_quaternion)
        self.place_height = 0.3
        self.pick_height = 0.265
    def run(self, j):
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
        pose_goal.position.z +=0
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
        top_right_corner_square = [0.67, -0.25, 0.4]
        top_right_corner_square[0] = top_right_corner_square[0]-0.03*(index/5)
        top_right_corner_square[1] = top_right_corner_square[1]-0.03*(index%5)
        print("Pick Pose: {}".format(top_right_corner_square))
        pose = Pose(position=Point(*top_right_corner_square))
        return pose
    def identify_shape(self):
        
        ret, frame = self.cam.read()
        predicted_shape = classify_shape.classify(frame)
        self.img_list.pop(predicted_shape[.0])
        return predicted_shape
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
        transform_point = [0.55, -0.15, 0.4]
        pose_goal  = Pose(position = Point(*transform_point))
        pose_goal.position.z +=0
        pose_goal.orientation = self.viewing_gripper_quaternion
        self.robot.right_manipulator.set_pose_goal(pose_goal)
    def current_pos(self):
        self.robot.right_gripper.set_value(90)
        pose = self.robot.right_manipulator.get_current_pose()
        print(pose)
    def move_to_board_corner(self):
        transform_point = [0.66, 0.07, 0.4]
        pose = Pose(position=Point(*transform_point))
        self.move_to(pose)
        pose.position.z = self.place_height
        self.move_to(pose)
    def pick_piece(self, i):
        pose = project.pick_square_piece_pose(i)
        rospy.loginfo("Grabbing Object Number #" + str(i))
        pose.position.z = 0.4
        self.move_to(pose)
        pose.position.z = self.pick_height
        self.move_to(pose)
        project.robot.right_gripper.close()
        pose.position.z = 0.4
        self.move_to(pose)
if __name__ == '__main__':
    project = MovingPieces()
    #project.run(2)
    #project.test()
    for i in range(25):
        project.robot.right_gripper.set_value(90)
        project.pick_piece(i)
        project.move_to_cv()
        project.move_to_board_corner()
        project.robot.right_gripper.set_value(90)
    # project.current_pos()