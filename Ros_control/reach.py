#!/usr/bin/env python

import  os
import  sys
import  tty
import termios
import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

if __name__ == '__main__':
    rospy.init_node("hi")

    # Create move group interface for a fetch robot
    # planning_scene = PlanningSceneInterface("base_link")
    # planning_scene.removeCollisionObject("my_front_ground")
    # planning_scene.removeCollisionObject("my_back_ground")
    # planning_scene.removeCollisionObject("my_right_ground")
    # planning_scene.removeCollisionObject("my_left_ground")
    # planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
    # planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
    # planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
    # planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

    # This is the wrist link not the gripper itself
    # Position and rotation of two "wave end poses"
    # gripper_poses = [Pose(Point(0.042, 0.384, 1.826),
    #                       Quaternion(0.173, -0.693, -0.242, 0.657)),
    #                  Pose(Point(0.047, 0.545, 1.822),
    #                       Quaternion(-0.274, -0.701, 0.173, 0.635))]
    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    gripper_frame = 'wrist_roll_link'
    


    # Construct a "pose_stamped" message as required by moveToPose
    gripper_pose_stamped = PoseStamped()
    gripper_pose_stamped.header.frame_id = 'base_link'

    while not rospy.is_shutdown():
        # x, y, z =raw_input().split()
        # x = float(x)
        # y = float(y)
        # z = float(z)
        # pose =Pose(Point(0.5, 0.5, 0.8),Quaternion(0.2, 0.0, 0.0, 0.0))
        # x,y,z,w = -0.274, -0.701, 0.173, 0.635
        # std = sys.stdin.fileno()
        # settings = termios.tcgetattr(std)
        # try:
        #     tty.setraw(std)
        #     key = sys.stdin.read(1)
        # finally:
        #     termios.tcsetattr(std, termios.TCSADRAIN, settings)  
        # if key == 'a':
        #     x += 0.1
        # if key == 's':
        #     y += 0.1
        # if key == 'd':
        #     z += 0.1
        # if key == 'f':
        #     w += 0.1

        # if key == 'z':
        #     x -= 0.1
        # if key == 'x':
        #     y -= 0.1
        # if key == 'c':
        #     z -= 0.1 
        # if key == 'v':
        #     w -= 0.1

        # pose = Pose(Point(0.714735948298, 0.230212412285 ,0.854012569332),
        #             Quaternion(-0.151017997039, 0.351409601185, 0.367942540546, 0.847539458318))
        pose = Pose(Point(0.795787086791
  , 0.327617928505
  , 0.892385081917),
                    Quaternion(-0.506820478748
  , 0.492144314148
  , 0.511446524957
  , 0.489233624561))
        # print x, y, z, w
        # Finish building the Pose_stamped message
        # x, y, z, w =raw_input().split()
        # x = float(x)
        # y = float(y)
        # z = float(z)
        # w = float(w)
        
        # pose =Pose(Point(0.763, -0.139, 1.025),Quaternion(x, y, z, w))

        # If the message stamp is not current it could be ignored
        gripper_pose_stamped.header.stamp = rospy.Time.now()
        # Set the message pose
        gripper_pose_stamped.pose = pose

        # Move gripper frame to the pose specified
        move_group.moveToPose(gripper_pose_stamped, gripper_frame)
        result = move_group.get_move_action().get_result() 
        exit()
        if result:
            
            # Checking the MoveItErrorCode
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Hello there!")
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
    exit()
