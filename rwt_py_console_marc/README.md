rwt_py_console_marc
===================

Installation
------------
1. Install TigerVNC Server and Websockify
    ```sh
    sudo apt install tigervnc-standalone-server websockify
    ```

2. Build ROS package
    ```
    cd <your_catkin_ws>/src

    git clone https://github.com/robo-marc/visualization_rwt.git

    rosdep install --ignore-src --from-paths visualization_rwt/rwt_py_console_marc -y

    catkin build rwt_py_console_marc

    source <your_catkin_ws>/devel/setup.bash
    ```

Usage
-----
```sh
roslaunch rwt_py_console_marc rwt_py_console.launch
```

and access to http://localhost:8000/rwt_py_console_marc/ using your browser

![rwt_py_console_marc.png](images/rwt_py_console_marc.png "rwt_py_console_marc.png")
