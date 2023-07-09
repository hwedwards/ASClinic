.. _ros-code-cmakelists:

Make nodes executable (C++ and Python)
======================================

Requirements for this page:

* ROS is installed on the computer you are working on (be that working "directly" on the computer or remotely connected to it).
* A catkin workspace and ROS package initialised as per :ref:`ros-create-catkin-ws-and-package`
* Code for the simple nodes as per :ref:`ros-code-node-simple`

.. contents:: Contents of this page
   :local:
   :backlinks: none
   :depth: 2


TL;DR
*****

Of course you did already read everything below, and the following TL;DR summary is just for convenience when you return to remind yourself of a small detail:

.. code-block:: bash

  # EDIT THE C MAKE LIST TO ADD THE C++ FILE
  cd ~/my-robotics-system/catkin_ws/src/my_robotics_pkg/
  nano CMakeLists.txt

  # COMPILE THE C++ NODE
  cd ~/my-robotics-system/catkin_ws/
  catkin_make

  # ADD EXCUTION PERMISSION TO THE PYTHON FILE
  cd ~/my-robotics-system/catkin_ws/src/my_robotics_pkg/src
  chmod +x plain_py_node.py

The three not-a-comment lines in the following are all that needs to be added to the :code:`CMakeLists.txt`, these lines provide instructions for how to compile the C++ node:

.. code-block:: bash

  ## Declare a C++ executable
  ## With catkin_make all packages are built within a single CMake context
  ## The recommended prefix ensures that target names across packages don't collide
  # add_executable(${PROJECT_NAME}_node src/my_robotics_pkg_node.cpp)
  add_executable(plain_cpp_node src/plain_cpp_node.cpp)

  ## Rename C++ executable without prefix
  ## The above recommended prefix causes long target names, the following renames the
  ## target back to the shorter version for ease of user use
  ## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
  # set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

  ## Add cmake target dependencies of the executable
  ## same as for the library above
  # add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  add_dependencies(plain_cpp_node ${catkin_EXPORTED_TARGETS})

  ## Specify libraries to link a library or executable target against
  # target_link_libraries(${PROJECT_NAME}_node
  #   ${catkin_LIBRARIES}
  # )
  target_link_libraries(plain_cpp_node ${catkin_LIBRARIES})



.. _ros-code-node-simple-add-to-cmake:

Add the C++ node to the C Make List
***********************************

ROS does not automatically to try to compile new :code:`*.cpp` files that you add. You need to provide explicit instructions via the :code:`CMakeLists.txt` file of your ROS package. You provide the compilation instructions by adding the following three lines of code to the appropriate section of the :code:`CMakeLists.txt`:

0. Open the :code:`CMakeLists.txt` file for editing:

  .. code-block:: bash

    cd ~/my-robotics-system/catkin_ws/src/my_robotics_pkg/
    nano CMakeLists.txt

1. Search the :code:`CMakeLists.txt` for the following comments, which is in the section titled :code:`## BUILD ##`:

   .. code-block:: bash

     ## Declare a C++ executable
     ## With catkin_make all packages are built within a single CMake context
     ## The recommended prefix ensures that target names across packages don't collide
     # add_executable(${PROJECT_NAME}_node src/my_robotics_node.cpp)

   And add the following line below this comment:

   .. code-block:: bash

     add_executable(plain_cpp_node src/plain_cpp_node.cpp)

   This makes the node available to run / launch with the name :code:`plain_cpp_node`. A simple convention to start with is to use the same as the file name. If, and when, you start to have naming collision across pacakges, then you can follow the recommendation of the comment to put :code:`${PROJECT_NAME}` as the prefix. 

   We will return to this line when we need to add additional C++ class as executables for a node to access.

2. Find the following a few comments further down:

   .. code-block:: bash

     ## Add cmake target dependencies of the executable
     ## same as for the library above
     # add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

   And add the following line below this comment:

   .. code-block:: bash

     add_dependencies(plain_cpp_node ${catkin_EXPORTED_TARGETS})

   We will return to this line when we need to add messages types that the node depends on.

3. Find the following as the next comments:

   .. code-block:: bash

     ## Specify libraries to link a library or executable target against
     # target_link_libraries(${PROJECT_NAME}_node
     #   ${catkin_LIBRARIES}
     # )

   And add the following line below this comment:

   .. code-block:: bash

     target_link_libraries(plain_cpp_node ${catkin_LIBRARIES})

   We will return to this line when we need to add compiler flags for linking libraries to a node, for example, the :code:`gpiod` library.


Compile the C++ node
********************

Change directory to the catkin workspace and call :code:`catkin_make` to perform compilation:

.. code-block:: bash

  cd ~/my-robotics-system/catkin_ws/
  catkin_make

The executable files of your C++ nodes are stored in the :code:`build` and :code:`devel` folders as make of the :code:`catkin_make` process.

.. important::

  In order for any changes to your C++ node to take effect, you need to:

    1. Shutdown the node if it is currently running
    2. Ensure your changes are saves
    3. Compile the changes using :code:`catkin_make`
    4. Observe that :code:`catkin_make` completed without any errors
    5. Run / launch the node

.. note::

  If, and when, you encounter "strange" compilation or run time errors that have you completely stumped, one thing to try is deleting the :code:`build` and :code:`devel` folders and compiling again, i.e.,

  .. code-block:: bash

    cd ~/my-robotics-system/catkin_ws/
    rm -rf build/
    rm -rf devel/
    catkin_make


Make the Python file executable
*******************************

As Python is an interpreted language, ROS directly executes the Python code you write, hence the files need the neccessary permissions to execute.

At some stage when you are trying to run / launch a Python node, you are likely to get an error message similar to the following:

:code:`Couldn't find executable named plain_py_node.py below ...`

This error message is likely because the Python script you are trying to run / launch does not have execution rights. If you want to check this is the case, then list the details of the file using:

.. code-block:: bash

  ls -la ~/my-robotics-system/catkin_ws/src/my_robotics_pkg/src

Which should display the details of the Python file in question as:

.. code-block:: bash

  -rw-rw-r-- 1 asc asc  205 Jan  1 23:45 plain_py_node.py

Use the following command to add (:code:`+`) executable (:code:`x`) permissions to the file: 

.. code-block:: bash

  chmod +x ~/my-robotics-system/catkin_ws/src/my_robotics_pkg/src/plain_py_node.py

The listing of the file should now display the following details:

.. code-block:: bash

  -rwxrwxr-x 1 asc asc  205 Jan  1 23:45 plain_py_node.py

.. important::

  In order for any changes to your Python node to take effect, you need to:

    1. Shutdown the node if it is currently running
    2. Ensure your changes are saved
    3. Run / launch the node again



References
**********

The steps detailed on this page are mostly taken from:

  * `ROS wiki page: CMakeLists.txt <http://wiki.ros.org/catkin/CMakeLists.txt>`_



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
