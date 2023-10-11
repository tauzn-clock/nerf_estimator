import argparse
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def callback(data):
    # Convert ROS image to OpenCV image
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

    # Display the image
    cv2.imshow("Camera", cv_image)
    cv2.waitKey(1)

def main():
    # Get the camera topic name during runtime
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera_topic", type=str, help="Name of the camera topic")
    args = parser.parse_args()
    camera_topic = args.camera_topic

    # Initialize the ROS node
    rospy.init_node("camera_py")

    # Subscribe to the camera topic
    rospy.Subscriber(camera_topic, Image, callback)

    # Spin the node to keep it running
    rospy.spin()

if __name__ == "__main__":
    main()
