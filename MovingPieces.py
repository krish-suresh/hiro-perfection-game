from math import pi, radians
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from hiro_core.XamyabRobot import XamyabRobot, rospy
from collections import deque, Counter
import random
import sys
import classify_shape
import cv2 
import numpy as np
import time

class MovingPieces:
    def __init__(self):
        self.robot = XamyabRobot(visualize_trajectory=False)
        self.default_gripper_quaternion = Quaternion(*quaternion_from_euler(pi, 0, 0))
        self.viewing_gripper_quaternion = Quaternion(*quaternion_from_euler(pi, 0, 0))
        self.start_pos = Pose(position=Point(*[0.6256, 0, 0.4]), orientation=self.default_gripper_quaternion)
        self.place_height = 0.3
        self.pick_height = 0.27
        self.increment_spacing_x = 0.033
        self.increment_spacing_y = 0.033
        self.cam = cv2.VideoCapture(1)
        self.cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        focus = 100
        self.cam.set(28, focus) 
        ret, frame = self.cam.read()
        self.img_list = range(25)
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
        top_right_corner_square[0] = top_right_corner_square[0]-self.increment_spacing_x*(index/5)
        top_right_corner_square[1] = top_right_corner_square[1]-self.increment_spacing_y*(index%5)
        print("Pick Pose: {}".format(top_right_corner_square))
        pose = Pose(position=Point(*top_right_corner_square))
        return pose
    def place_square_piece_pose(self, index):
        top_right_corner_square = [0.66, 0.1, 0.4]
        top_right_corner_square[0] = top_right_corner_square[0]-self.increment_spacing_x*(index/5)
        top_right_corner_square[1] = top_right_corner_square[1]-self.increment_spacing_y*(index%5)
        print("Place Pose: {}".format(top_right_corner_square))
        pose = Pose(position=Point(*top_right_corner_square), orientation=self.default_gripper_quaternion)
        return pose 
    def identify_shape(self):
        predicted_shape = None
        for i in range(5):
            ret, frame = self.cam.read()
            predicted_shape =classify_shape.classify(frame, self.img_list)
        # print(predicted_shapes)
        # occurence_count = Counter(predicted_shapes)
        # predicted_shape = occurence_count.most_common(1)[0][0]
        self.img_list.remove(predicted_shape[0])
        return (predicted_shape[0], radians(predicted_shape[1]))

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

    # shape_number = 0
    # cam = cv2.VideoCapture(1)
    # cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    # time.sleep(1)
    # focus = 20
    # cam.set(28, focus)
    project.place_piece((0,0)) 
    raw_input('start?')
    for i in range(25):
        project.pick_piece(i)
        project.move_to_cv()
        piece = project.identify_shape()
        print(piece)
        project.place_piece(piece)
    #     while(1):
    #         _, frame = cam.read()
    #         cv2.imshow("frame", frame)
    #         k = cv2.waitKey(1)
    #         if k%256 == 97:
    #             cv2.imwrite("shapes_update/piece_{}.png".format(shape_number), frame)
    #             shape_number+=1
    #             print(shape_number)
    #             project.robot.right_gripper.set_value(90)
    #             break
    #         if k%256== 27:
    #             break
    # cam.release()
    # cv2.destroyAllWindows()
    # project.pick_piece(12)
    # project.move_to_cv()
    # piece = project.identify_shape()
    # piece = (0,pi/4) #project.identify_shape()
    # project.place_piece(piece)
    # while True:
    #     x = raw_input('open')
    #     project.robot.right_gripper.set_value(90)
    #     x = raw_input('close')
    #     project.robot.right_gripper.close()
    # project.cam.release()
    sys.exit()
    
    # project.current_pos()