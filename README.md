Terminal I/O
============

Terminal I/O enables ROS nodes to communicate with external terminal sessions.

Build & Install
---------------

Terminal I/O is built using [catkin](http://wiki.ros.org/catkin). Type the commands below on a terminal window to create a catkin workspace, clone the repository and build the sources:

    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/src
    $ catkin_init_workspace
    $ git clone https://github.com/xperroni/terminal_io.git
    $ cd ..
    $ catkin_make

Usage
-----

First, the `terminal_server` process must be started in a terminal session, for example:

    $ rosrun terminal_io terminal_server

It may also be possible to automatically create a new terminal window and start the server on it as part of a launch file. The details will vary depending on the environment, though; see the project's `launch` folder for an example on KDE.

Once the server is on, a ROS process can use a `terminal_io::Terminal` object to communicate to it, printing text or reading back user input. See the `include/terminal_io/terminal.h` header file for details.
