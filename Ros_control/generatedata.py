#!/usr/bin/env python

# simple_disco.py: Move the fetch arm through a simple disco motion
import xlwt
import random
import rospy
import time
import rostopic
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from moveit_ros_planning_interface import _moveit_move_group_interface
import moveit_commander

# Note: fetch_moveit_config move_group.launch must be running
# Safety!: Do NOT run this script near people or objects.
# Safety!: There is NO perception.
#          The ONLY objects the collision detection software is aware
#          of are itself & the floor.


def writerow(i, row):
    for j in range(0, len(row)):
        sheet.write(i, j, row[j])
    data.save('data2.xls')


if __name__ == '__main__':
    rospy.init_node("simple_disco")

    # Create move group interface for a fetch robot
    move_group = MoveGroupInterface("arm", "base_link")

    joint_names = ["shoulder_pan_joint",
                   "shoulder_lift_joint", "upperarm_roll_joint",
                   "elbow_flex_joint", "forearm_roll_joint",
                   "wrist_flex_joint", "wrist_roll_joint"]
    # disco_poses = [
    #                 [0.5, 0.0, 0.0, -0.8, 0.0, 0.8, 0.0],
    #                 # [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
    #                ]

    pose_group = moveit_commander.MoveGroupCommander("arm")

    data = xlwt.Workbook()
    sheet = data.add_sheet('sheet1', cell_overwrite_ok=True)
    row0 = ['x', 'y', 'z', "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_flex_joint",
            "wrist_flex_joint"]
    writerow(0, row0)
    t=0
    while t<65535:
        if rospy.is_shutdown():
            break
        pose = pose_group.get_random_joint_values()
        # 0, 1, 3, 5
        # pose = [0, 0.5, 0, -1, 0, 0.5, 0]  #z=0.8
        # pose = [0, 0.6, 0, -1.25, 0, 0.65, 0]  #z=0.8
        # pose = [0, 0, 0, 0, 0, 0, 0]

        # pose[0] = random.random()*0.5
        # pose[1] = 0.5
        pose[2] = 0
        pose[3] = -abs(pose[3])
        pose[4] = 0
        pose[5] = -pose[1]-pose[3]
        pose[6] = 0
        print("pose = ", pose)
        # Plans the joints in joint_names to angles in pose
        move_group.moveToJointPosition(joint_names, pose, wait=False)

        # Since we passed in wait=False above we need to wait here
        move_group.get_move_action().wait_for_result()
        result = move_group.get_move_action().get_result()

        if result:
            # Checking the MoveItErrorCode
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                # rospy.loginfo("Disco!")
                print(t)
                # print(pose_group.get_random_joint_values())
                # print(pose_group.get_joints())
                position = pose_group.get_current_pose("wrist_roll_link").pose.position
                pose1 = pose_group.get_current_joint_values()
                row = []
                row.append(position.x)
                row.append(position.y)
                row.append(position.z)
                row.append(pose1[0])
                row.append(pose1[1])
                row.append(pose1[3])
                row.append(pose1[5])
                writerow(t+1, row)
                t = t+1
                # print("cupo = ", pose_group.get_current_joint_values())
                # print(pose_group.get_current_pose("wrist_roll_link").pose)
                # time.sleep(2)


            else:
                # If you get to this point please search for:
                # moveit_msgs/MoveItErrorCodes.msg

                rospy.logerr("Arm goal in state: %s",
                             move_group.get_move_action().get_state())
        else:
            rospy.logerr("MoveIt! failure no result returned.")

    # This stops all arm movement goals
    # It should be called when a program is exiting so movement stops
    move_group.get_move_action().cancel_all_goals()

