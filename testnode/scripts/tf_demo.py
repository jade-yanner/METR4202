#!/usr/bin/env python3

from turtle import position
import rospy
import tf
from sensor_msgs.msg import JointState
import numpy as np

from modern_robotics import ( 
    MatrixExp6,
    VecTose3,
) 




class JointStateNode(object): 
    def __init__(self): 
        rospy.init_node("demo_two_link")

        self.joint_state_gui_sub = rospy.Subscriber("/joint_states", data_class=JointState, callback=self.joint_state_callback)
        self.joint_state_pub = rospy.Publisher("/tf_demo_joint_states", data_class=JointState, queue_size=10)

        self.tf_listener = tf.TransformListener()

        self.M = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0.2],
            [0, 0, 1, 2.85],
            [0, 0, 0, 1]        
            ])
        
        #w1 = [0, 1, 0]
        #q1= [0, 0, 1.95]
        #-w1 cross q2
        self.S1 = np.array(
            [0, 
             1, 
             0,
            -1.95, 
             0, 
             0]
        )
                
        #w2 = [0, 1, 0]
        #q2= [0, 0, 2.85]
        #-w2 cross q2
        self.S2 = np.array(
            [0, 
             1, 
             0,
            -2.85, 
             0, 
             0]
        )

    def joint_state_callback(self, msg:JointState):
        self.joint_state_pub.publish(msg)
        
        theta = msg.position
        Tspace_joint2 = MatrixExp6(VecTose3(self.S1)*theta[0]) @ MatrixExp6(VecTose3(self.S2)*theta[1])@self.M
        print(Tspace_joint2)

        trans, rot = self.tf_listener.lookupTransform("/base_link", "top_link", rospy.Time(0))
        print(f"Translation: {trans=}")
        print(f"Angles (rad): {rot=}")
        print("-------------------------")


    def run(self): 
        rospy.sleep(2)
        rospy.spin()

if __name__ == "__main__": 
    try: 
        joint_state_node = JointStateNode()
        joint_state_node.run()
    except rospy.ROSInterruptException as e: 
        pass
