# To remap the topic with a different name 
ros2 run my_py_pkg robot_news_station --ros-args -r __node:=node_name_different -r robot_news:my_news

# To directly publish topics from the terminal
ros2 topic pub -r 10 /robot_news example_interfaces/msg/String "{data : 'hello'}"

# To find the average frequency of the messages published
ros2 topic hz /robot_news
# To get the details of the message interface
ros2 interface show example_interfaces/msg/String

# to print the msg published to this topic
ros2 topic echo /robot_news

ros2 topic info /robot_news
# to list all the topics
ros2 topic list

# We should not have 2 nodes with the same node, so to start a node with a different name
ros2 run my_cpp_pkg cpp_node --ros-args -r __node:=node_name_different
ros2 run my_cpp_pkg cpp_node --ros-args --remap __node:=node_name_different

# To get all the nodes running
ros2 node list

# use help to know the details
ros2 run -h

# To run the node py_node from the package my_py_pkg
ros2 run my_py_pkg py_node

# To make changes to python files on the fly, we don't need to build it everytime there is a change
colcon build --packages-select my_py_pkg --symlink-install

# To build seperate packages
colcon build --packages-select my_py_pkg

# To build the packages in the workspace
colcon build

# To create cpp package inside the src folder
ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp

# To create python package
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy



