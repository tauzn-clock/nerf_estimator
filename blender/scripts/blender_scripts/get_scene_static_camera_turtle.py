import bpy
import os
import json
import numpy as np
from scipy.spatial.transform import Rotation

def set_object_pose(name, xyz, rpy):
    #Set camera position
    bpy.data.objects[name].location = xyz
    bpy.data.objects[name].rotation_euler = rpy

def get_pose(object):
    pose = np.eye(4)
    pose[:3,3] = np.array([object.location.x,object.location.y,object.location.z])
    pose[:3,:3] = Rotation.from_euler("xyz",(object.rotation_euler.x,object.rotation_euler.y,object.rotation_euler.z),degrees=True).as_matrix()
    return pose
def invert_pose(pose):
    inverse = np.eye(4)
    inverse[:3,:3] = pose[:3,:3].T
    inverse[:3,3] = - pose[:3,:3].T @ pose[:3,3]
    return inverse


# Set light
light_name = 'Light'
bpy.data.objects[light_name].location = (0,0,2.5)
bpy.data.objects[light_name].rotation_euler = (0,0,0)
bpy.data.objects[light_name].data.energy = 50

# Set data path
FILE_PATH = os.path.realpath(__file__)
FILE_PATH = FILE_PATH.split('/')
FILE_PATH = '/'.join(FILE_PATH[:-3])

DATA_PATH = os.path.join(FILE_PATH, 'data/turtle_floor_training_data_1')
print(DATA_PATH)

# Get transform
with open(os.path.join(DATA_PATH, 'ground_truth.json'), 'r') as f:
    transforms = json.load(f)


# Set render resolution
bpy.context.scene.render.resolution_x = transforms['w']
bpy.context.scene.render.resolution_y = transforms['h']
bpy.context.scene.render.image_settings.file_format = 'PNG'

# Set camera paramerters

camera_name = 'Camera'
bpy.data.objects[camera_name].data.lens = transforms['focal_length']
bpy.data.objects[camera_name].data.sensor_height = transforms["pixel_height"] * transforms["h"]
bpy.data.objects[camera_name].data.sensor_width = transforms["pixel_width"] * transforms["w"]

OUTPUT_FILE = os.path.join(DATA_PATH, "output.txt")
with open(OUTPUT_FILE,'w') as f:
    f.write("Angle:" + str(bpy.data.objects[camera_name].data.angle) + "\blender/test/jackal_floor_evaluation_datan")
    f.write("Angle X:" + str(bpy.data.objects[camera_name].data.angle_x) + "\n")
    f.write("Angle Y:" + str(bpy.data.objects[camera_name].data.angle_y) + "\n")
    f.write(str(bpy.data.collections.get("visual")))

init_pose = np.eye(4)
init_pose[:3,:3] = Rotation.from_euler("x", 90, degrees=True).as_matrix()

cnt = 0
for frame in transforms['frames']:
    file_path = os.path.join(DATA_PATH, frame['file_path'])
    transform_matrix = np.asarray(frame['transform_matrix'])
    camera_xyz = transform_matrix[:3, 3]
    camera_rpy = transform_matrix[:3, :3]
    camera_rpy = Rotation.from_matrix(camera_rpy).as_euler('xyz', degrees=False)
    
    #Translate to y = -2, align and pitch down
    final_pose = np.eye(4)
    final_pose[:3,:3] = Rotation.from_euler('xyz',(camera_rpy[0],0,0),degrees=False).as_matrix()
    final_pose[:3,3] = np.array([0,-2,camera_xyz[2]])
    
    camera_transform = final_pose
    camera_xyz = camera_transform[:3, 3]
    camera_rpy = camera_transform[:3, :3]
    camera_rpy = Rotation.from_matrix(camera_rpy).as_euler('xyz', degrees=False)
    set_object_pose(camera_name, camera_xyz, camera_rpy)
    
    object_transform = final_pose @ invert_pose(transform_matrix) @ init_pose
    object_xyz = object_transform[:3, 3]
    object_rpy = object_transform[:3, :3]
    object_rpy = Rotation.from_matrix(object_rpy).as_euler('xyz', degrees=False)
    
    set_object_pose("Turtlebot2i v11",object_xyz,object_rpy)
    
    # Render Path
    bpy.context.scene.render.filepath = os.path.join(DATA_PATH,file_path)

    # Render scene
    bpy.ops.render.render(write_still=True)