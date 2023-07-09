.. _ros-create-catkin-ws-and-package:

Create a catkin workspace and a ROS package
===========================================

Requirements for this page:

* ROS is installed on the computer you are working on (be that working "directly" on the computer or remotely connected to it).
* A git account that you can push to and pull from on the computer with ROS installed.

.. contents:: Contents of this page
   :local:
   :backlinks: none
   :depth: 2


TL;DR
*****

Of course you did already read everything below, and the following TL;DR summary is just for convenience when you return to remind yourself of a small detail:

.. code-block:: bash

  # SOURCE THE ROS setup.bash FILE
  source /opt/ros/melodic/setup.bash

  # MAKE THE CATKIN WORKSPACE DIRECTORY
  cd ~
  mkdir my-robotics-system
  cd my-robotics-system
  mkdir catkin_ws

  # INITIALISE THE CATKIN WORKSPACE
  cd catkin_ws
  mkdir src
  catkin_make

  # SOURCE THE devel/setup.bash FILE FROM THE CATKIN WORKSPACE
  source devel/setup.bash

  # CREATE THE ROS PACKAGE
  cd src/
  catkin_create_pkg my_robotics_pkg std_msgs rospy roscpp

  # EDIT THE DETAILS IN THE PACKAGE XML FILE
  nano my_robotics_pkg/package.xml

  # ADD A GIT IGNORE FILE
  cd ~/my-robotics-system
  nano .gitignore

  # COMMIT AND PUSH TO YOUR BLANK GIT REPOSITORY
  cd ~/my-robotics-system
  git init
  git remote add origin
  https://gitlab.unimelb.edu.au:foobar/my-robotics-system.git
  git add .
  git commit -m "Initial commit"
  git push -u origin master


Suggested contents for a :code:`.gitignore` file:

.. code-block:: bash

  # Ignore the "build" and "devel" folder of the catkin_ws
  catkin_ws/build/
  catkin_ws/devel/

  # Ignore the ".catkin_workspace" file of the catkin_ws
  catkin_ws/.catkin_workspace

  # Ignore the auto-generated "CMakeLists.txt" of the catkin_ws
  catkin_ws/src/CMakeLists.txt

  # Ignore swp files and other typically unwanted files
  *.swp
  .DS_Store



Source the ROS :code:`setup.bash`
*********************************

In order to have the various ROS commands available from command line, you need to source the :code:`setup.bash` of your ROS installation. This file is located at the following location:

.. code-block:: bash

  /opt/ros/<ros-distro>/setup.bash

where :code:`<ros-distro>` is replaced by the name of the ROS distribution you are using. For example, if you are using ROS :code:`melodic`, then the source command to enter in terminal is:

.. code-block:: bash

  source /opt/ros/melodic/setup.bash

This step needs to be completed for each new terminal window or ssh connection that you open.


(Optional) add ROS :code:`setup.bash` to the :code:`.bashrc`
***************************************************************

You can save yourself entering the :code:`source` command every time by adding it to the :code:`.bashrc` file. The :code:`.bashrc` file is a bash script that runs every time you open a new terminal window or ssh connection. In general, there is a simple pro and con of adding command to the :

  * **Pro:** Saves you entering (or forgetting) to the command.
  * **Con:** Over time you forget what was added to the :code:`.bashrc` file, and perhaps in the future a command there cause errors that you cannot explain.

Adding the ROS :code:`setup.bash` to the :code:`.bashrc` file is a pretty safe bet for a computer that you regularly use to run ROS.

The :code:`.bashrc` file is located in the home directory of the logged in user. Hence:

  * Use :code:`cd ~` to change to the users home directory,
  * Then :code:`ls -la` to list all files and see that the :code:`.bashrc` file indeed exists
  * Then :code:`nano .bashrc` to edit the file (or use whichever editor you prefer, possibly :code:`vi .bashrc`)
  * Add the following lines of code to the end of the :code:`.bashrc` file (where the first line is a comment to remind you future-self what this command does):

  .. code-block:: bash

    # SOURCE THE ROS setup.bash FILE
    source /opt/ros/melodic/setup.bash

What does the :code:`rc` stand for anyway, you may ask. It is a relic from the 1960's and stands for `RUNCOM according to this Wikipedia <https://en.wikipedia.org/wiki/RUNCOM>`_ page, where they provide the quote:

  *There was a facility that would execute a bunch of commands stored in a file; it was called runcom for "run commands", and the file began to be called "a runcom". rc in Unix is a fossil from that usage.*


Make the :code:`catkin_ws` directory
************************************

Key to how ROS works is a prescribed directory structure for where you place the various pieces of code you write. As an additional layer to that, the goal of this page it to start a git repository with your ROS code so that you can write code from any computer, push it up to the repository, and then pull it down to your ROS computer for testing (i.e., pull the code to your robot).

Create a directory for the git repository:

.. code-block:: bash

  cd ~
  mkdir my-robotics-system

where you can replace :code:`my-robotics-system` with any directory name you wish.

Create a :code:`catkin_ws` directory:

.. code-block:: bash

  cd my-robotics-system
  mkdir catkin_ws

where :code:`ws` stands for work space and `catkin is the official build system of ROS <https://wiki.ros.org/catkin/conceptual_overview>`_, hence this directory is the space where you do all of your ROS work.

.. note::

  You can in fact use a directory name different from :code:`catkin_ws`, but using :code:`catkin_ws` is a good convention because then everyone knows to expect the sub-folder structure described on the rest of this page and on the `ROS Wiki catkin workspaces page <https://wiki.ros.org/catkin/workspaces>`_.


Use :code:`catkin_make` to initialise
*************************************

The :code:`catkin_make` command is the main tool for working with catkin workspaces. We use it now to initialise our catkin workspace. But first we need to have a :code:`src` directory:

.. code-block:: bash

  cd catkin_ws
  mkdir src
  catkin_make

After :code:`catkin_make` has finished, you can list the contents with :code:`ls -la` to see what was created:

.. code-block::

  catkin_ws/
    .catkin_workspace
    build/
    devel/
    src/
      CMakeLists.txt -> /opt/ros/melodic/share/catkin/

And you see that within the :code:`devel/` there is a :code:`setup.bash` file.


Source the :code:`setup.bash` of the catkin workspace
*****************************************************

In order to have the content of the catkin workspace available to work with from command line, you need to source the :code:`setup.bash` file from the :code:`devel` folder created in the previous step, i.e.,

.. code-block:: bash

  source devel/setup.bash

This step needs to be completed for each new terminal window or ssh connection that you open.


(Optional) add the catkin workspace :code:`setup.bash` to the :code:`.bashrc`
*****************************************************************************

Similar to above, edit the :code:`.bashrc` to add the following lines at the end:

.. code-block:: bash

  # SOURCE THE devel/setup.bash FILE FROM THE CATKIN WORKSPACE
  source ~/my-robotics-system/catkin_ws/devel/setup.bash


.. note::

  It is less clear whether adding the catkin workspace :code:`devel/setup.bash` to the :code:`.bashrc` file always make sense. For example, if you have multiple **copies of the same** :code:`catkin_ws`, each for testing a different feature (which may each be a separate branch of your git repository), then you only want to source the :code:`catkin_ws` relevant for the tests you are about to perform. Hence have one of then in the :code:`bashrc` (the one on the master branch for example) may cause more lost time in confusion that the time it saves in typing the source command every time.


Double check what is on the :code:`ROS_PACKAGE_PATH`
*****************************************************

If at any time you need to double check which :code:`setup.bash` files are already source for the current terminal session, then you simply need to echo the :code:`ROS_PACKAGE_PATH` environment variable:

.. code-block:: bash

  echo $ROS_PACKAGE_PATH

The following is an example results of the echo:

.. code-block::

  /home/asc/my-robotics-system/catkin_ws/src:/opt/ros/melodic/share

indicating that the catkin workspace and ROS melodic :code:`setup.bash` files are sourced.


What is a ROS Package
*********************

A ROS package is essentially a self-contained grouping of ROS code that serves a particular purpose. Self contained in the sense that everything needed to compile and run the ROS nodes contained in the package. The purpose of a package can range anywhere from being narrow and highly specialised to being broad and general. For example, you might create a package specifically for path planning, or you might create a package for generally doing every part of your robotics project.

The main benefit of compartmentalising functionality into packages is that you and others can easily add a package and its functionality to existing projects. To gain the benefits of adding a package, it should be well documented and provide sufficient abstraction of the implementation details.

As a general rule, start your robotics project with everything in one pacakge, and as things mature you can carve away certain functionalities into separate packages.


Create a ROS Package
*********************

A catkin workspace can have multiple packages, and the root of all packages must be in the :code:`catkin_ws/src/` directory. Hence change to that directory before using the create package command:

.. code-block:: bash

  cd src/

The command :code:`catkin_create_pkg` will create the skeleton of a ROS package, it uses the following syntax:

.. code-block:: bash

  catkin_create_pkg <package_name> <dependencies> 

where :code:`<package_name>` is replaced by the name you wish to give you package, and :code:`<dependencies>` is a list what standard ROS packages your package depends on. For example, create a package for your robotic system using the following:

.. code-block:: bash

  catkin_create_pkg my_robotics_pkg std_msgs rospy roscpp

The dependencies :code:`std_msgs`, :code:`rospy`, and :code:`roscpp` are the minimum you should include to be able to write ROS nodes in C++ or Python and communicate between the nodes with standard message types.

.. note::

  You can change the dependencies of your package at a later time.


Interrogating and customizing your package
******************************************

The create package command above create a folder with the name of your package, e.g., :code:`my_robotics_pkg`, and within that folder it creates the following four items:

  * :code:`package.xml` that provides all the high-level details of your package.
  * :code:`CMakeLists.txt` that provides the CMake instructions for how the contents of your package is to be compiled.
  * :code:`src` folder, where you can add and develop the source code for your robotics project.
  * :code:`include` folder, where you can add the header files you create and develop.

You should now edit the :code:`package.xml` file to include all the relevant details. A good guide for doing this is provided by the `ROS Tutorial: Customizing the package.xml <https://wiki.ros.org/catkin/Tutorials/CreatingPackage#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.Customizing_the_package.xml>`_

The :code:`CMakeLists.txt` is something we will return to multiple times when necessary. For now you can skim through the auto-generated comments in the :code:`CMakeLists.txt` to get a feeling for what goes in there.


Git ignore and git push
***********************

Having completed the steps above, now is a good time to commit and push changes to your git repository.

Most of the files automatically generated by the steps above do not need to be committed to your git repository because they would be automatically generated again. Hence add a :code:`.gitignore` file to the root of your git repository:

.. code-block:: bash

  cd ~/my-robotics-system
  nano .gitignore

and put the following as the contents of the :code:`.gitignore` file:

.. code-block:: bash

  # Ignore the "build" and "devel" folder of the catkin_ws
  catkin_ws/build/
  catkin_ws/devel/

  # Ignore the ".catkin_workspace" file of the catkin_ws
  catkin_ws/.catkin_workspace

  # Ignore the auto-generated "CMakeLists.txt" of the catkin_ws
  catkin_ws/src/CMakeLists.txt

  # Ignore swp files and other typically unwanted files
  *.swp
  .DS_Store

Now you can commit and push the changes to :code:`my-robotics-system` git repository. How exactly to do this depends on how or whether you have setup the existence of a :code:`my-robotics-system` git repository on your chosen git hosting hosting platform.

Assuming you are using a gitlab instance and you have created a blank project named :code:`my-robotics-system`, then use the following commands to commit and push:

.. code-block:: bash

  cd ~/my-robotics-system
  git init
  git remote add origin
  https://gitlab.unimelb.edu.au:foobar/my-robotics-system.git
  git add .
  git commit -m "Initial commit"
  git push -u origin master

* The :code:`foobar` on line 4 should be replaced by your username on the :code:`gitlab.unimelb.edu` gitlab instance, if that is indeed what you are using.
* The :code:`https://` on line 4 can be replaced by :code:`git@` if you prefer to use ssh authentication instead of password authentication.


References
**********

The steps detailed on this page are mostly taken from:

  * `ROS tutorial: Creating a workspace for catkin <https://wiki.ros.org/catkin/Tutorials/create_a_workspace>`_
  * `ROS tutorial: Creating a ROS package <https://wiki.ros.org/catkin/Tutorials/CreatingPackage>`_
  * `Atlassian explanation of git ignore patterns <https://www.atlassian.com/git/tutorials/saving-changes/gitignore#git-ignore-patterns>`_



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
