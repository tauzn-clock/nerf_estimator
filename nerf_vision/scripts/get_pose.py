import argparse
import tf2_ros
import rospy
import json

def get_transformation(source_frame, target_frame,
                       tf_cache_duration=2.0):
    tf_buffer = tf2_ros.Buffer(rospy.Duration(tf_cache_duration))
    tf2_ros.TransformListener(tf_buffer)

    # get the tf at first available time
    try:
        transformation = tf_buffer.lookup_transform(target_frame,
                source_frame, rospy.Time(0), rospy.Duration(0.1))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException):
        rospy.logerr('Unable to find the transformation from %s to %s'
                     % source_frame, target_frame)
    return transformation

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

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
if __name__ == "__main__":
    main()