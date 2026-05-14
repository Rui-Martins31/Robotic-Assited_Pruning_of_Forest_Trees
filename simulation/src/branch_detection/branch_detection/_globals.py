import numpy as np

# Camera and Image
IMAGE_WIDTH: int  = 400
IMAGE_HEIGHT: int = 300

CAMERA_FOV_HORIZONTAL: float = 1.05  # rad
CAMERA_FX: float             = (IMAGE_WIDTH  / 2) / np.tan(CAMERA_FOV_HORIZONTAL / 2)
CAMERA_FY: float             = CAMERA_FX 
CAMERA_CU: float             = IMAGE_WIDTH  / 2
CAMERA_CV: float             = IMAGE_HEIGHT / 2
CAMERA_MAX_DIST: float       = 100.0
CAMERA_MIN_DIST: float       = 0.1