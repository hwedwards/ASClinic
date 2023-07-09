.. _ros-run-and-launch:

Run, launch, and interrogate nodes
==================================

Requirements for this page:

  * ROS is installed on the computer you are working on (be that working "directly" on the computer or remotely connected to it).
  * A catkin workspace and ROS package initialised as per :ref:`ros-create-catkin-ws-and-package`
  * At a minimum, the simple nodes as per :ref:`ros-code-node-simple`

.. contents:: Contents of this page
   :local:
   :backlinks: none
   :depth: 2


TL;DR
*****

Of course you did already read everything below, and the following TL;DR summary is just for convenience when you return to remind yourself of a small detail:

**Use** :code:`rosrun` **with 4 separate terminals:**

.. code-block:: bash

  # TERMINAL 1
  roscore
  
  # TERMINAL 2
  rosrun my_robotics_pkg plain_cpp_node

  # TERMINAL 3
  rosrun my_robotics_pkg plain_py_node

  # TERMINAL 4
  rosnode list


**Use** :code:`roslaunch` **with 2 separate terminals:**

.. code-block:: bash

  # TERMINAL 1
  roslaunch my_robotics_pkg plain.launch

  # TERMINAL 2
  rosnode list

.. important::

  :code:`--help` is your friend.

  As for any command line tool, use the help display as the quickest reminder of usage: :code:`rosrun --help`, :code:`roslaunch --help`, :code:`rosnode --help`

.. important::

  The :code:`Tab` key is also your friend. The tools :code:`rosrun`, :code:`roslaunch`, and :code:`rosnode` all support tab completion. Save yourself some time and press :code:`Tab`.

Contents for a "plain.launch" launch file:

.. code-block:: html

  <launch>

      <!-- EXAMPLE OF DEFINING AN INPUT ARGUMENT -->
      <arg name="alsopython" default="false" />

      <!-- START A GROUP WITH A NAMESPACE -->
      <group ns="mrp">
          <!-- LAUNCH A "Plain C++" NODE -->
          <node
              pkg    = "my_robotics_pkg"
              name   = "plain_cpp_node"
              output = "screen"
              type   = "plain_cpp_node"
          />

          <!-- USE THE INPUT ARGUMENT IN AN IF-STATEMENT  -->
          <group if="$(arg alsopython)">
              <!-- LAUNCH A "Plain Python" NODE -->
              <node
                  pkg    = "my_robotics_pkg"
                  name   = "plain_py_node"
                  output = "screen"
                  type   = "plain_py_node.py"
              />
          </group>
      </group>

  </launch>


:code:`rosrun` - run a single node
**********************************

The syntax for using the :code:`rosrun` command is:

.. code-block:: bash

  rosrun <package> <executable>

where :code:`<package>` is replaced by the name of your package, and :code:`<executable>` is replaced by the name of the executable file (i.e., the node) that you want to run:

  * **For C++:** the executable is the name that you used in the :code:`add_executable` line of the :code:`CMakeLists.txt` (i.e., see :ref:`ros-code-node-simple-add-to-cmake`).
  * **For Python:** the executable is the name of the Python file including the extension.

Hence the commands to run the plain C++ and Python nodes are:

.. code-block:: bash

  # RUN THE PLAIN C++ NODE
  rosrun my_robotics_pkg plain_cpp_node

  # RUN THE PLAIN PYTHON NODE
  rosrun my_robotics_pkg plain_py_node.py


.. important::

  :code:`rosrun` can only start one node the terminal that you used cannot be used for anything else while the node is running.

  Hence, to run multiple nodes at the same time, you need to open multiple terminals (or multiple ssh connections).


.. important::

  :code:`rosrun` can only start running a node if the *ROS Master* is already running, otherwise, you will get a message similar to the following will be displayed:

  .. code-block:: console

    Unable to register with master node [http://localhost:11311]: master may not be running yet. Will keep trying.

  Hence, before using :code:`rosrun` you need to use a separate terminal to start the *ROS Master* node by using the command:

  .. code-block::bash

    roscore


To kill a node that is running, go to the terminal from which you started the node with :code:`rosrun`, and press :code:`ctrl+c` to cancel the command and hence kill the node.


:code:`roslaunch` - launch multiple nodes at once
*************************************************

Launch files allow multiple node to be run (i.e., launched) with a single command, and they allow specifying extra attributes for how the nodes should be launched, for example, namespace and parameters can be specified.

The syntax for using the :code:`roslaunch` command is:

.. code-block:: bash

  rosrun <package> <launch_file>

where :code:`<package>` is replaced by the name of your package, and :code:`<launch_file>` is replaced by the name of the launch file that you want to launch.

.. important::

  In contrast to :code:`rosrun`, the :code:`roslaunch` command can be used whether or not the *ROS Master* and the :code:`rosout` are already running:

    * If *ROS Master* is NOT already running, then :code:`roslaunch` will also launch the *ROS Master* and :code:`rosout` nodes.
    * If *ROS Master* is already running, then :code:`roslaunch` will only launch the nodes specified in the given launch file.

Launch files use a html-like syntax with tags that are bespoke to ROS. You should keep you launch files in a :code:`launch` folder of your package:

.. code-block::

  cd ~/my-robotics-system/catkin_ws/src/my_robotics_pkg
  mkdir launch
  cd launch

.. note::

  The :code:`roslaunch` command actually searches for launch files anywhere within the package, but keeping them in folder named :code:`launch/` is good practice

Start a launch file for editing
###############################

A launch file is simple a text file with the extension :code:`.launch`:

.. code-block:: bash

  nano plain.launch

Add the :code:`<launch>` tag
############################

.. code-block:: html
  :emphasize-lines: 1,3

  <launch>

  </launch>

All instructions inside the launch tag are executed when the launch file is :code:`roslaunch`-ed. A launch file:

  * MUST have only one launch tag.
  * MUST have nothing outside of the launch tag.


Add a :code:`<group>` with a namespace
######################################

.. code-block:: html
  :emphasize-lines: 2-3,5

  <launch>
      <!-- START A GROUP WITH A NAMESPACE -->
      <group ns="mrp">
          
      </group>
  </launch>

The group tag is a convenient way to group together the launching of multiple nodes that you want to put within the same namespace. The namespace property :code:`ns=` can be given any string, for example, :code:`"mrp"` is short for "my robotics package" and reflects an intention to launch all nodes from the package into the same namespace.

.. note::

  For an application where you have multiple robots launching and operating on the same ROS network, then this group namespace attribute can be parameterised with the ID of the robot. This can enable launching all robots in an automated fashion, for example, with :code:`ns="mrp_robot001"`, :code:`ns="mrp_robot002"`, ..., :code:`ns="mrp_robot042"`, ..., etc.


Add a :code:`<node>` or two
###########################

For adding a C++ node and a Python node:

.. code-block:: html
  :emphasize-lines: 4-10,12-18

  <launch>
      <!-- START A GROUP WITH A NAMESPACE -->
      <group ns="mrp">
          <!-- LAUNCH A "Plain C++" NODE -->
          <node
              pkg    = "my_robotics_pkg"
              name   = "plain_cpp_node"
              output = "screen"
              type   = "plain_cpp_node"
          />

          <!-- LAUNCH A "Plain Python" NODE -->
          <node
              pkg    = "my_robotics_pkg"
              name   = "plain_py_node"
              output = "screen"
              type   = "plain_py_node.py"
          />
      </group>
  </launch>



Add and use an input argument, :code:`<arg>` 
############################################


.. code-block:: html
  :emphasize-lines: 2-3,15-16,24

  <launch>
      <!-- EXAMPLE OF DEFINING AN INPUT ARGUMENT -->
      <arg name="alsopython" default="false" />

      <!-- START A GROUP WITH A NAMESPACE -->
      <group ns="mrp">
          <!-- LAUNCH A "Plain C++" NODE -->
          <node
              pkg    = "my_robotics_pkg"
              name   = "plain_cpp_node"
              output = "screen"
              type   = "plain_cpp_node"
          />

          <!-- USE THE INPUT ARGUMENT IN AN IF-STATEMENT  -->
          <group if="$(arg alsopython)">
              <!-- LAUNCH A "Plain Python" NODE -->
              <node
                  pkg    = "my_robotics_pkg"
                  name   = "plain_py_node"
                  output = "screen"
                  type   = "plain_py_node.py"
              />
          </group>
      </group>
  </launch>

The :code:`<arg>` tag is used to declare the input argument, providing the attributes:

  * :code:`name=` as any string that uniquely identifies the argument for this launch file.
  * :code:`default` as the default value of the input argument, which is used if the argument is not specified as part of the launch command.

The value of an input argument is accessed within the launch file using :code:`$(arg alsopython)`.

In the example, a group with an :code:`if=` attribute is used so that the nodes within that group are only launched if the input argument has the value true, i.e.:

.. code-block:: html

  <group if="$(arg alsopython)">
      <!-- THINGS HERE ONLY HAPPEN IF THE  -->
      <!-- ARGUMENT alsopython IS TRUE     -->
  </group>


The value of an input argument is set using :code:`:=` on the launch command, for example:

.. code-block:: bash

  roslaunch my_robotics_pkg plain.launch alsopython:=true


Use an environment variable for a default :code:`<arg>` value
#############################################################

In case it suits the architecture of you robotics project, the default value for a launch file input argument can be taken from an environment variable:

.. code-block:: html

  <!-- INPUT ARGUMENT OF THE ROBOT'S ID -->
  <arg name="robotID" default="$(optenv ROBOT_ID)" />

where the :code:`ROBOT_ID` environment variable would be set in some fashion prior to launching the launch file, for example using:

..code-block:: bash

  export ROBOT_ID=42

If you have one separate computer (e.g., Raspberry Pi) running on each separate robot, then this method can be used by setting a unique identifier into a :code:`ROBOT_ID` environment variable. Then the group namespace attribute can be using the same launch file for all robots as follows:

.. code-block:: html

  <group ns="$(eval 'mrp_robot' + str(robotID).zfill(3))">


:code:`rosnode` - command line tool for interrogating nodes
***********************************************************

Sometime the :code:`--help` display can seem because it seem "cluttered" by lots of options that you do not use. In the case of :code:`rosnode`, the help display is succinct and all you need.

The available commands for interrogating nodes with :code:`rosnode`:

.. code-block:: console

  # RUN THIS COMMAND
  rosnode --help
  
  # WHICH DISPLAYS THE FOLLOWING
  rosnode is a command-line tool for printing information about ROS Nodes.

  Commands:
    rosnode ping  test connectivity to node
    rosnode list  list active nodes
    rosnode info  print information about node
    rosnode machine list nodes running on a particular machine or list machines
    rosnode kill  kill a running node
    rosnode cleanup purge registration information of unreachable nodes

  Type rosnode <command> -h for more detailed usage, e.g. 'rosnode ping -h'

.. important::

  Note the last line in the console display above: Type :code:`rosnode <command> --help` for more detailed usage information about any of the :code:`rosnode` command, for example, :code:`rosnode ping --help`

Use :code:`list` to display all the nodes currently in existence and check that the namespaces and node names are as you expect:

.. code-block:: console

  # RUN THIS COMMAND
  rosnode list
  
  # WHICH MAY SOMETHING LIKE THE FOLLOWING
  /mrp/plain_cpp_node
  /mrp/plain_py_node

This simple listing allows you to see that when using a launch file, the node are indeed launched into the namespace :code:`mrp`. If you :code:`rosrun` the node, then you should see that :code:`mrp/` is no longer part of the nodes namespace, in fact, the nodes are run directly in the the root namespace :code:`/`


References
**********

The steps detailed on this page are mostly taken from:

  * `ROS documentation: rosbash (which includes rosrun) <https://wiki.ros.org/rosbash#rosrun>`_
  * `ROS documentation: roslaunch <https://wiki.ros.org/roslaunch>`_
  * `ROS documentation: rosnode <https://wiki.ros.org/rosnode>`_
  * `ROS tutorials: roslaunch tips for larger projects <https://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20tips%20for%20larger%20projects>`_



|

----

.. image:: https://i.creativecommons.org/l/by/4.0/88x31.png
  :alt: Creative Commons License
  :align: left
  :target: http://creativecommons.org/licenses/by/4.0/

| Paul N. Beuchat, 2023
| This page is licensed under a `Creative Commons Attribution 4.0 International License <http://creativecommons.org/licenses/by/4.0/>`_.

----

|
