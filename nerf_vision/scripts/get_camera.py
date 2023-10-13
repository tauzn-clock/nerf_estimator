import argparse
import rospy
import json
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
    # Initialize the ROS node
    rospy.init_node("camera_py")

    # Subscribe to the camera topic
    rospy.Subscriber(camera_topic, Image, callback)

    # Spin the node to keep it running
    rospy.spin()

if __name__ == "__main__":
    # Get the camera topic name during runtime
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", type=str, help="Name of the config file")    
    args = parser.parse_args()
    config_file = args.c

    #Read the config file as a json
    camera_topic = ""

    with open(config_file) as f:
        config_dict = json.load(f)
        camera_topic = config_dict["camera_topic"]

    print("Camera Topic: ", camera_topic)
    print("--------------------")

    main()
