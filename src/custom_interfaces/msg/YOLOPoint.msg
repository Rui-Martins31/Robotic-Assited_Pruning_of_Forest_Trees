# When the YOLO model detects a trunk or a branch, it outputs three values:
#           -> (x, y) in pixel coordinates. These are the center coordinates of the bounding box;
#           -> depth value of that pixel.

float64 x_pixel
float64 y_pixel
float64 depth