#!/usr/bin/env python
# -*- coding:utf-8 -*-


from __future__ import print_function

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
 
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonIntefaceTutorial(object):
    """MoveGroupPythonIntefaceTutorial"""
    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()

        
        moveit_commander.roscpp_initialize(sys.argv)
       # rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        gripper_name = "gripper"
        gripper_group = moveit_commander.MoveGroupCommander(gripper_name)

        print("Methods available in move_group")
        print(dir(move_group))

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)
         
        planning_frame = move_group.get_planning_frame()
       # print("============ Planning frame: %s" % planning_frame)

        planning_frame_gripper = gripper_group.get_planning_frame()
        #print("============ Planning frame for gripper: {}".format(planning_frame_gripper))    

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        #print("============ End effector link: %s" % eef_link)


        # We can also print the name of the end-effector link for this group:
        eef_link_gripper = gripper_group.get_end_effector_link()
        #print("============ End effector link: %s" % eef_link_gripper)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        #print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        #print("============ Printing robot state")
        #print(robot.get_current_state())
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.gripper_group = gripper_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.eef_link_gripper = eef_link_gripper
        self.group_names = group_names


    def go_to_home_joint_state(self):
        
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        
        home_angles =  (0., -1, 0.3, 0.7) # joints 1,2,3,4
        
        joint_goal = home_angles 

        
        move_group.go(joint_goal, wait=True)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.02)

    def open_gripper(self):
        
        move_group = self.gripper_group

       
        current_position = move_group.get_current_joint_values()
        print("Current gripper position", current_position)
        
        open_angles =  (0.019,0.019)
        
        joint_goal = open_angles 

        move_group.go(joint_goal, wait=True)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.002)

    def close_gripper(self):
       
        move_group = self.gripper_group

        current_position = move_group.get_current_joint_values()

        current_position = move_group.get_current_joint_values()
        print("Current gripper position", current_position)
        
        close_angles =  (-0.01, -0.01)
        
        joint_goal = close_angles 

       
        move_group.go(joint_goal, wait=True)

        move_group.stop()

        
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.002)

    def go_to_init_joint_state(self):
        
        move_group = self.move_group

        
        joint_goal = move_group.get_current_joint_values()
        
        home_angles =  (0., -0.25, 0.17, 0.24) # joints 1,2,3,4
        
        joint_goal = home_angles 

        move_group.go(joint_goal, wait=True)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.02)


    def go_to_zero_position_goal(self):
        
        move_group = self.move_group

        init_position = (0.238, 0.0, 0.3)


        move_group.set_position_target(init_position)
        move_group.set_goal_position_tolerance(0.03)

        plan = move_group.go(wait=True)
        move_group.stop()
        
        current_pose = self.move_group.get_current_pose().pose
        return True #all_close(pose_goal, current_pose, 0.03)

    def go_to_home_position_goal(self):
        
        move_group = self.move_group

        home_position = (0.046, 0.0, 0.345)


        move_group.set_position_target(home_position)
        move_group.set_goal_position_tolerance(0.03)

        plan = move_group.go(wait=True)
        move_group.stop()
        
        current_pose = self.move_group.get_current_pose().pose
        return True #all_close(pose_goal, current_pose, 0.03)


    def execute_plan(self, plan):
        
        move_group = self.move_group

       
        move_group.execute(plan, wait=True)


    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        
        box_name = self.box_name
        scene = self.scene

        
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL




def main():
  try:
    print("\n----------------------------------------------------------")
    print("\nWelcome to the MoveIt MoveGroup Python Interface Tutorial")
    print("\n----------------------------------------------------------")
    print("\nPress Ctrl-D to exit at any time\n")
    print("\n============ Press `Enter` to begin the tutorial by setting up the moveit_commander ...\n")
    raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()

    print("\n============ Press `Enter` to go to init joint state ...\n")
    raw_input()
    tutorial.go_to_init_joint_state()


    print("\n============ Press `Enter` to go to home joint state ...\n")
    raw_input()
    tutorial.go_to_home_joint_state()

    print("\n============ Press `Enter` to open gripper  ...\n")
    raw_input()
    tutorial.open_gripper()

    print("\n============ Press `Enter` to close gripper  ...\n")
    raw_input()
    tutorial.close_gripper()

    print("\n============ Press `Enter` to go to init goal ...\n")
    raw_input()
    tutorial.go_to_zero_position_goal()


    print("\n============ Press `Enter` to go to home goal ...\n")
    raw_input()
    tutorial.go_to_home_position_goal()

    print("\n============ Press `Enter` to open gripper  ...\n")
    raw_input()
    tutorial.open_gripper()

    print("\n============ Press `Enter` to close gripper  ...\n")
    raw_input()
    tutorial.close_gripper()





  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return
