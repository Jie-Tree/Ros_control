import rospy
import moveit
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface

rospy.init_node("simple_disco")
group = moveit.moveit_commander()
print(group)