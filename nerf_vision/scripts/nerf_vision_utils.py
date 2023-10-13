import rospy
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from cv_bridge import CvBridge
import tf2_ros
import tf


def getMoveBaseGoal(move_base_frame,target_x,target_y,target_arg):
    """
    Creates a MoveBaseGoal object with the given target coordinates and orientation
    :param target_x: x coordinate of the target
    :param target_y: y coordinate of the target
    :param target_arg: orientation of the target
    :return: MoveBaseGoal object
    """
    custom_goal=MoveBaseGoal()
    custom_goal.target_pose.header.frame_id = move_base_frame
    custom_goal.target_pose.header.stamp = rospy.Time.now()
    custom_goal.target_pose.pose.position.x = target_x
    custom_goal.target_pose.pose.position.y = target_y
    custom_goal.target_pose.pose.orientation.w=math.cos(target_arg/2)
    custom_goal.target_pose.pose.orientation.z=math.sin(target_arg/2)
    return custom_goal

def getCameraTopicAsCvImage(data):
    """
    Subscribes to the camera topic and returns the image as a cv image
    :param data: ROS image
    :return: cv image
    """
    # Convert ROS image to OpenCV image
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

    return cv_image

def getRobotPose(camera_frame, robot_frame, buffer_length=1200.0, timeout=102):
    """
    Calculate the robot pose in the camera frame
    :param camera_frame: camera frame name
    :param robot_frame: robot frame name
    :param buffer_length: tf buffer length
    :param timeout: tf timeout
    :return: xyz, rpy
    """
    #Create a tf buffer and listener to get the transform
    tfBuffer = tf2_ros.Buffer(rospy.Duration(buffer_length)) #tf buffer length
    tf2_ros.TransformListener(tfBuffer)
    currentTf = tfBuffer.lookup_transform(camera_frame, robot_frame, rospy.Time(0), rospy.Duration(timeout)).transform

    #Transform from quaternion to euler
    xyz = [currentTf.translation.x, currentTf.translation.y, currentTf.translation.z]
    q = [currentTf.rotation.x, currentTf.rotation.y, currentTf.rotation.z, currentTf.rotation.w]
    rpy = tf.transformations.euler_from_quaternion(q)

    return xyz, rpy