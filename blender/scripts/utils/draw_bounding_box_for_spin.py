import os
import json
import cv2
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
from PIL import Image

DATA_PATH = "../../test/jackal_floor_training_data_1"
SPIN = 50
#Open Transforms
with open(f"{DATA_PATH}/transforms.json", "r") as f:
    transforms = json.load(f)

drawing = False
ix,iy = -1,-1
start_x, start_y, end_x, end_y = -1, -1, -1, -1

def draw_rectangle(event, x, y, flags, param):
    global ix, iy, drawing, img
    global start_x, start_y, end_x, end_y

    if drawing:
        img = ori.copy()
        cv2.rectangle(img, (ix, iy), (x, y),(0, 0, 255),1)
        if event == cv2.EVENT_LBUTTONUP:
            drawing = False
            start_x = ix
            start_y = iy
            end_x = x
            end_y = y
    else:
        if event == cv2.EVENT_LBUTTONDOWN:
            drawing = True
            ix = x
            iy = y


rectangles = []
for i in range(0, len(transforms["frames"]), SPIN):
    img_path = os.path.join(DATA_PATH, transforms["frames"][i]['file_path'])
    print(img_path)
    
    #Open Image
    ori = cv2.imread(img_path)
    img = ori.copy()
    
    # Create a window and bind the function to window
    cv2.namedWindow("Rectangle Window")

    # Connect the mouse button to our callback function
    cv2.setMouseCallback("Rectangle Window", draw_rectangle)

    # display the window
    while True:
        cv2.imshow("Rectangle Window", img)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print(start_x, start_y, end_x, end_y)
            rectangles.append([[start_x, start_y, end_x, end_y]*SPIN])
            break
        
    cv2.destroyAllWindows()
        
        
#Save as bounding_box.csv
rectangles = np.array(rectangles).reshape(-1, 4)
np.savetxt(f"{DATA_PATH}/bounding_box.csv", rectangles, delimiter=",", fmt='%d')
