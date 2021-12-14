from math import pi
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from hiro_core.XamyabRobot import XamyabRobot, rospy
from collections import deque, Counter
import random
import sys
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
    def move_to(self, pose_goal, reset_orientation=True):
        pose_goal.position.z +=0
        if reset_orientation:
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
    def place_square_piece_pose(self, index):
        top_right_corner_square = [0.66, 0.07, 0.4]
        top_right_corner_square[0] = top_right_corner_square[0]-0.03*(index/5)
        top_right_corner_square[1] = top_right_corner_square[1]-0.03*(index%5)
        print("Place Pose: {}".format(top_right_corner_square))
        pose = Pose(position=Point(*top_right_corner_square), orientation=self.default_gripper_quaternion)
        return pose 
    def identify_shape(self):
        predicted_shapes = []
        for i in range(10):
            ret, frame = self.cam.read()
            predicted_shapes.append(classify_shape.classify(frame, self.img_list))
        occurence_count = Counter(predicted_shapes)
        predicted_shape = occurence_count.most_common(1)[0][0]
        self.img_list.pop(predicted_shape[0])
        return predicted_shape

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
    def pick_piece(self, i):
        project.robot.right_gripper.set_value(90)
        pose = project.pick_square_piece_pose(i)
        rospy.loginfo("Grabbing Object Number #" + str(i))
        pose.position.z = 0.4
        self.move_to(pose)
        pose.position.z = self.pick_height
        self.move_to(pose)
        project.robot.right_gripper.close()
        pose.position.z = 0.4
        self.move_to(pose)
    def place_piece(self, piece):
        i = piece[0] # Piece number
        pose = project.place_square_piece_pose(i)
        rospy.loginfo("Placing Object Number #{} at {} deg".format(str(i), str(piece[1])))
        pose.position.z = 0.4
        pose.orientation = Quaternion(*quaternion_from_euler(pi, 0, piece[1])) # TODO check if rotation is right
        self.move_to(pose, reset_orientation=False)
        pose.position.z = self.place_height
        self.move_to(pose, reset_orientation=False)
        project.robot.right_gripper.set_value(90)
        pose.position.z = 0.4
        self.move_to(pose)
if __name__ == '__main__':
    project = MovingPieces()
    # for i in range(25):
    #     project.pick_piece(i)
    #     project.move_to_cv()
    #     piece = project.identify_shape()
    #     project.place_piece(piece)

    project.place_piece((0,pi/4)) # Test placing piece
    # project.current_pos()