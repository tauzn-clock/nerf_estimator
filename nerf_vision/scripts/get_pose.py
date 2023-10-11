import argparse
import tf2_ros
import rospy
import json
import tf2_geometry_msgs 
import tf

def main():
    #Take a config file as parameter
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, help="Name of the config file")
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
        #Create a tf buffer and listener to get the transform
        tfBuffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        tf2_ros.TransformListener(tfBuffer)
        currentTf = tfBuffer.lookup_transform(camera_frame, robot_frame, rospy.Time(0), rospy.Duration(102)).transform

        #Transform from quaternion to euler
        xyz = [currentTf.translation.x, currentTf.translation.y, currentTf.translation.z]
        q = [currentTf.rotation.x, currentTf.rotation.y, currentTf.rotation.z, currentTf.rotation.w]
        rpy = tf.transformations.euler_from_quaternion(q)
        
        print("xyz: ", xyz)
        print("rpy: ", rpy)

if __name__ == "__main__":
    main()