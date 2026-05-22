# When the YOLO model detects a branch, it subdivides the branch into smaller segments
# and outputs the cutting points coordinates:
#           -> size: size of the buffer;
#           -> points: array of points (x,y,z).

int32 size
geometry_msgs/Point[] points