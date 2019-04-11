# CS603_RRT
RRT Planning for an ackerman-steered car.

## Dependencies
Install [ROS](http://wiki.ros.org/ROS/Installation)

And install additional dependencies
   ```
   sudo apt-get install cmake build-essential clang libgoogle-glog-dev  libgflags-dev libgtest-dev
   ```

## Clone This Repository
   ```
   git clone git@github.com:umass-amrl/CS603_RRT.git
   ```

## Build Instructions
1. Clone this repo, and enter the directory. 
   All subsequent commands must be run from within the directory.
1. Add the project to the ROS packages path:
   ```bash
   export ROS_PACKAGE_PATH=/PATH/TO/YOUR/REPO:$ROS_PACKAGE_PATH
   ```
   Pro-tip: add this line to your `.bashrc` file.
1. Run `make`.  
   DO NOT RUN `cmake` directly.  
   DO NOT RUN `catkin_build` or anything catkin-related.  
   
## Run 
1. Before starting programs that use ros, you will need to start a roscore instance in a terminal that you keep open:
     ```
   roscore
   ```

