import argparse
import tf2_ros
import rospy
import json
import tf2_geometry_msgs 

def main():
    #Take a config file as parameter
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str)
    args = parser.parse_args()
    config_file = args.config

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

    rospy.init_node('tf2_listener')
    while True:
        tfBuffer = tf2_ros.Buffer(rospy.Duration(2.0))  # tf buffer length
        tf2_ros.TransformListener(tfBuffer)
        print(tfBuffer.lookup_transform(camera_frame, robot_frame, rospy.Time(0), rospy.Duration(0.1)))
        rospy.sleep(0.2)

if __name__ == "__main__":
    main()