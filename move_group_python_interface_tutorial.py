#!/usr/bin/env python

from __future__ import print_function
from six.moves import input
import numpy as np
import quaternion
import random
import sys
import actionlib
import copy
import rospy

import franka_gripper.msg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
    
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        d = dist((x1, y1, z1), (x0, y0, z0))
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=10,
        )
        planning_frame = move_group.get_planning_frame()
        # print("============ Planning frame: %s" % planning_frame)

        eef_link = move_group.get_end_effector_link()
        # print("============ End effector link: %s" % eef_link)

        group_names = robot.get_group_names()
        # print("============ Available Planning Groups:", robot.get_group_names())


        # print("============ Printing robot state")
        # print(robot.get_current_state())
        # print("")

        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names


    def go_to_joint_state(self):

        move_group = self.move_group


        joint_goal = move_group.get_current_joint_values()
        print(joint_goal)
        joint_goal[0] = -pi / 2
        joint_goal[1] = -pi / 2
        joint_goal[2] = pi / 2
        joint_goal[3] = -pi / 2
        joint_goal[4] = pi / 2
        joint_goal[5] = pi  # 1/6 of a turn
        joint_goal[6] = pi / 2

        move_group.go(joint_goal, wait=True)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        # print(move_group.get_current_state())
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, x_length,y_length, z_length=0.2):

        move_group = self.move_group

        q = np.quaternion(-0.08, 1.0, -2.4, -0.05).normalized()
        pose_goal = geometry_msgs.msg.Pose()  # Создаем объект Pose для задания целевой позы
        pose_goal.orientation.x = q.x
        pose_goal.orientation.y = q.y
        pose_goal.orientation.z = q.z
        pose_goal.orientation.w = q.w
        pose_goal.position.x = x_length  # Устанавливаем новые координаты положения
        pose_goal.position.y = y_length
        pose_goal.position.z = z_length

        move_group.set_pose_target(pose_goal)

        success = move_group.go(wait=True)

        move_group.stop()

        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):

        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  
        wpose.position.y += scale * 0.2 
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1 
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  
        return plan, fraction


    def display_trajectory(self, plan):

        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)

        display_trajectory_publisher.publish(display_trajectory)
    def execute_plan(self, plan):

        move_group = self.move_group

        move_group.execute(plan, wait=True)


    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):

        box_name = self.box_name
        scene = self.scene


        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():

            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            is_known = box_name in scene.get_known_object_names()

            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()

        return False


    def add_box(self, name,timeout=4):
        box_name = self.box_name
        scene = self.scene

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "panda_hand"#"world" 
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.11
        box_name = str(name)
        # box_pose.pose.position.x = x  # Устанавливаем координаты положения
        # box_pose.pose.position.y = y
        # box_pose.pose.position.z = z
        scene.add_box(box_name, box_pose, size=(0.075, 0.075, 0.075))
        
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, name,timeout=4):

        box_name = str(name)
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        grasping_group = "panda_hand"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)

        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self,name1, timeout=4):

        box_name = str(name1)
        scene = self.scene
        eef_link = self.eef_link

        scene.remove_attached_object(eef_link, name=box_name)

        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_box(self, name, timeout=4):

        box_name = self.box_name
        scene = self.scene

        scene.remove_world_object(str(name))

        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )

def move_gripper(w, s):
    client = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = franka_gripper.msg.MoveGoal(width=w, speed=s)
    #goal.width = 0.022
    #goal.speed = 1.0
    
    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A move result
def main():
    try:
        tutorial = MoveGroupPythonInterfaceTutorial()
        # tutorial.go_to_pose_goal(0.5,0.5)
        # tutorial.add_box(1,0.5,0.5,0.09)
        # tutorial.add_box(2,0.4,0.4,0.09)
        # tutorial.add_box(3,0.3,0.3,0.09)
        # tutorial.attach_box(str(1))
        # tutorial.go_to_pose_goal(0.5,-0.5)
        # tutorial.detach_box(str(1))
        tutorial.go_to_joint_state()
        result = move_gripper(0.08, 1.0)
        while(True and not rospy.is_shutdown()):
            x1 = random.randint(3, 6)/10
            y1 = random.randint(3, 6)/10   
            tutorial.go_to_pose_goal(x1,y1,0.31)
            result = move_gripper(0.08, 1.0)
            tutorial.go_to_pose_goal(x1,y1)
            tutorial.add_box(1)
            result = move_gripper(0.05, 1.0)
            tutorial.attach_box(1)
            tutorial.go_to_pose_goal(x1, -y1)
            result = move_gripper(0.08, 1.0)
            tutorial.detach_box(1)
            tutorial.remove_box(1)
        # tutorial.go_to_pose_goal(0.4,0.4,0.2)
        # tutorial.attach_box(str(2))
        # tutorial.go_to_pose_goal(0.4,-0.4)
        # tutorial.detach_box(str(2))
        # tutorial.f()
        # tutorial.go_to_pose_goal(0.3,0.3,0.3)
        # tutorial.attach_box(str(3))
        # tutorial.go_to_pose_goal(0.3,-0.3)
        # tutorial.detach_box(str(3))
        
        # tutorial.go_to_joint_state()
        # tutorial.f()
        # i=0
        # while(i<100):
        #     tutorial.go_to_pose_goal(random.randint(3, 6)/10, random.randint(3, 6)/10)
        #     tutorial.add_box(i)
        #     tutorial.attach_box(i)
        #     tutorial.go_to_pose_goal(random.randint(3, 6)/10, random.randint(-6, -3)/10)
        #     tutorial.detach_box(i)
        #     tutorial.remove_box(i)
        #     i+=1
        # for i in range(3):
        #     tutorial.go_to_pose_goal(random.randint(2, 7)/10, random.randint(2, 7)/10)
        #     tutorial.add_box(i)
        #     tutorial.attach_box(i)
        #     tutorial.go_to_pose_goal(random.randint(2, 7)/10, random.randint(-7, -2)/10)
        #     tutorial.detach_box(i)
        # tutorial.go_to_joint_state()
        # tutorial.remove_box(0)
        # tutorial.remove_box(1)
        # tutorial.remove_box(2)

        # tutorial.remove_box()
        # input("============ Press `Enter` to plan and display a Cartesian path ...")
        # cartesian_plan, fraction = tutorial.plan_cartesian_path()

        # input(
        #     "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
        # )
        # tutorial.display_trajectory(cartesian_plan)

        # input("============ Press `Enter` to execute a saved path ...")
        # tutorial.execute_plan(cartesian_plan)

        # input("============ Press `Enter` to add a box to the planning scene ...")
        

        # input("============ Press `Enter` to attach a Box to the Panda robot ...")
        

        # input(
        #     "============ Press `Enter` to plan and execute a path with an attached collision object ..."
        # )
        # cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
        # tutorial.execute_plan(cartesian_plan)

        # input("============ Press `Enter` to detach the box from the Panda robot ...")
        

        # input(
        #     "============ Press `Enter` to remove the box from the planning scene ..."
        # )
        # tutorial.remove_box()

        # print("============ Python tutorial demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
