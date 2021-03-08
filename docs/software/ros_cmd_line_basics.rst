.. _ros-cmd-line-basics:

ROS COMMAND LINE BASICS
=======================

The list below provides basic commands for performing tasks within the ROS (Robot Operating System) framework, but it is by no means exhaustive.
The `ROS Tutorials <http://wiki.ros.org/ROS/Tutorials>`_ are a great place to start for understanding these commands in more details, some highlights are:

* `Navigating the ROS Filesystem <http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem>`_
* `Creatinga ROS Package <http://wiki.ros.org/ROS/Tutorials/CreatingPackage>`_
* `Building a ROS Package <http://wiki.ros.org/ROS/Tutorials/BuildingPackages>`_
* `Understanding ROS Nodes <http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes>`_


Compiling
*********

* :code:`catkin_make` compiles the ROS package when called from the ROS workspace folder of the repository, i.e., from the folder named :code:`catkin_ws`

* See the `catkin_make documentation <http://wiki.ros.org/catkin/commands/catkin_make>`_ for more usages


Sourcing a ROS package
**********************

* :code:`<path_to_catkin_ws>/devel/setup.bash` sources the :code:`setup.bash` script that was automatically created when the :code:`catkin_make` command was successfully run

* :code:`~/asclinic-system/catkin_ws/devel/setup.bash` sources the :code:`setup.bash` script for when this repository was cloned to the default location for the logged in user, i.e., to :code:`~/asclinic-system/`


Launching nodes
***************

* :code:`roscore` launches the :code:`ROS Master` and the :code:`rosout` node

* :code:`rosrun [package_name] [node_name]` launches the node named :code:`[node_name]` from the package named :code:`[package_name]`

* :code:`rosrun asclinic template_cpp` launches the node named :code:`template_cpp` from the package named :code:`asclinic`

* :code:`roslaunch [package] [filename.launch]` starts nodes as defined in the launch file named :code:`[filename.launch]` that is located in the :code:`launch/` folder of :code:`[package]`

  * **Note** the :code:`roslaunch` command actually searches for launch files anywhere within the package, but keeping them in folder named :code:`launch/` is good practice

* :code:`roslaunch asclinic template.launch` starts nodes as defined in the file :code:`template.launch` that is located in the :code:`launch/` folder of the :code:`asclinic` package

* :code:`roslaunch asclinic template.launch alsopython:=true` as above but with the argument :code:`alsopython` set to :code:`true`, which is then then used inside the launch file to also launch the template python node


Displaying information about nodes
**********************************

* :code:`rosnode` displays the manual entry for the :code:`rosnode` command that is used to display information about ROS nodes that are currently running

* :code:`rosnode list -h` displays the manual entry for the :code:`list` command of :code:`rosnode`

* :code:`rosnode list` displays a list of the currently running nodes, also known as the active nodes

* :code:`rosnode info [node_name]` displays information about the node named :code:`[node_name]`, including any publishers, subscribers, and services of that nodes

* :code:`rosnode info template_cpp` displays information about the node named :code:`template_cpp` if it is currently running


Displaying information about topics
************************************

* :code:`rostopic` displays the manual entry for the :code:`rostopic` command that is used to display information about current ROS topics

* :code:`rostopic list -h` displays the manual entry for the :code:`list` command of :code:`rostopic`

* :code:`rostopic list` displays a list of all topics currently subscribed to and published

* :code:`rostopic list -v` displays a verbose list of topics subscribed to and published, including their message type and number of subscribers and publishers

* :code:`rostopic info [topic]` displays a information about :code:`[topic]`, including its message type, and the name of node subscribing and publishing to the topic

* :code:`rostopic info /asclinic/template_topic` displays a information about the topic :code:`/asclinic/template_topic`

* :code:`rostopic echo [topic]` displays the data published on :code:`[topic]`

* :code:`rostopic hz [topic]` displays the publishing rate of :code:`[topic]`


Miscellaneous
*************

* :code:`Ctrl-C` stop the currently active command, for example, to stop a :code:`roslaunch` command and hence kill the nodes launched by that terminal.
