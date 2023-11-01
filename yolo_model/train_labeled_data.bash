#Get the directory the bash file is currently in
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
echo $DIR

yolo task=detect \
mode=train \
model=yolov8n.pt \
data= $DIR/jackal_detection-4/data.yaml \
epochs=100 \
imgsz=640