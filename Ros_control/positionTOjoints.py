from keras.models import load_model
import numpy as np
import rospy
import moveit_commander
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface

if __name__ == '__main__':
    rospy.init_node("simple_disco")
    # Create move group interface for a fetch robot
    move_group = MoveGroupInterface("arm", "base_link")
    pose_group = moveit_commander.MoveGroupCommander("arm")
    # Define ground plane
    # This creates objects in the planning scene that mimic the ground
    # If these were not in place gripper could hit the ground
    planning_scene = PlanningSceneInterface("base_link")
    planning_scene.removeCollisionObject("my_front_ground")
    planning_scene.removeCollisionObject("my_back_ground")
    planning_scene.removeCollisionObject("my_right_ground")
    planning_scene.removeCollisionObject("my_left_ground")
    planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
    planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
    planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
    planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

    # TF joint names
    joint_names = ["shoulder_pan_joint",
                   "shoulder_lift_joint", "upperarm_roll_joint",
                   "elbow_flex_joint", "forearm_roll_joint",
                   "wrist_flex_joint", "wrist_roll_joint"]
    model = load_model('./model/0.962.h5')
    # model = load_model('model3output/0.981.h5')
    # x = np.array([[1.012846, 0, 0.723164+0.2]])
    x = np.array([[1.000000, -0.300000, 0.723224 + 0.2]])
    # x.reshape(1,3)
    # print(x.shape)
    joints = model.predict(x)[0]
    joint_3 = 0-joints[1]-joints[2]
    print(x)
    disco_poses = [
        [joints[0], joints[1], 0.0, joints[2], 0, joint_3, 0.0],
        # [joints[0], joints[1], 0.0, joints[2], 0, joint_3+0.2, 0.0],
        # [joints[0], joints[1], 0.0, joints[2], 0, joint_3, 0.0],
    ]

    for pose in disco_poses:
        if rospy.is_shutdown():
            break

        # Plans the joints in joint_names to angles in pose
        move_group.moveToJointPosition(joint_names, pose, wait=False)

        # Since we passed in wait=False above we need to wait here
        move_group.get_move_action().wait_for_result()
        result = move_group.get_move_action().get_result()

        if result:
            # Checking the MoveItErrorCode
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Hit!")
                position = pose_group.get_current_pose("wrist_roll_link").pose.position
                print(position)
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