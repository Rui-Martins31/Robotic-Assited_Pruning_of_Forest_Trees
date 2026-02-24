## Packages

**List of Packages:**

    - MoveIt
    - Manipulator
    - Image Processing

---

### MoveIt

Package that is responsible for everything related to the trajectory planning and motion control of the manipulator.

The launch file (*gz_moveit.launch.py*) launches both the gazebo and the moveit nodes. 

The MoveIt nodes (MoveIt and Rviz) are launched with a 5.0 seconde delay to guarantee that all the controllers and the simulator are already up and running before it

### Manipulator

Package that contains meshes (visual and collision) of the manipulator and the world.

World description file is only used when gazebo is used.

### Image Processing

Package that processes images obtained from the camera attached to the manipulator.

**Nodes:**

- Images Saver: saves images to the directory ("output/image_processing/images_raw")