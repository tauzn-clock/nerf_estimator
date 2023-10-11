import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math	

def set_goal(target_x,target_y,target_arg):
    client = actionlib.SimpleActionClient("move_base",MoveBaseAction)
    client.wait_for_server()

    custom_goal=MoveBaseGoal()
    custom_goal.target_pose.header.frame_id = "odom"
    custom_goal.target_pose.header.stamp = rospy.Time.now()
    custom_goal.target_pose.pose.position.x = target_x
    custom_goal.target_pose.pose.position.y = target_y
    custom_goal.target_pose.pose.orientation.w=math.cos(target_arg/2)
    custom_goal.target_pose.pose.orientation.z=math.sin(target_arg/2)

    client.send_goal(custom_goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        set_goal(2,-2,0)
        set_goal(2,2,math.pi/2)
        set_goal(-2,2,math.pi)
        set_goal(-2,-2,math.pi*1.5)
    except rospy.ROSInterruptException:
        pass