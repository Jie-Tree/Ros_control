#!/usr/bin/env python

'''
get the position information(translation and rotation) of object from TFMessage
'''

import rospy
from std_msgs.msg import String 
from tf2_msgs.msg import TFMessage

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    id = data.transforms[0].child_frame_id
    if id == "object_16":
        print 'the position:'
        print data.transforms[0].transform.translation
        print 'the rotation:'
        print data.transforms[0].transform.rotation
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    #import ipdb; ipdb.set_trace()
    rospy.Subscriber("/tf", TFMessage, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()