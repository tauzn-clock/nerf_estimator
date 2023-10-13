import sys
import os
sys.path.append(os.path.dirname(__file__))
from nerf_vision_utils import getRobotPose
import argparse
import rospy
import json


def main(camera_frame,robot_frame):
    rospy.init_node('tf2_client_listener')
    while True:
        xyz, rpy = getRobotPose(camera_frame, robot_frame)
        print("xyz: ", xyz)
        print("rpy: ", rpy)

if __name__ == "__main__":
        #Take a config file as parameter
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", type=str, help="Name of the config file")
    args = parser.parse_args()
    config_file = args.c

    #Read the config file as a json
    camera_frame = ""
    robot_frame = ""

    with open(config_file) as f:
        config_dict = json.load(f)
        camera_frame = config_dict["camera_frame"]
        robot_frame = config_dict["robot_frame"]
    
    print("Camera Frame: ", camera_frame)
    print("Robot Frame: ", robot_frame)
    print("--------------------")
    main(camera_frame,robot_frame)