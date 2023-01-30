.. _software-installation-manual:

Manual Installation
===================


The :ref:`installation script <software-installation-script>` automates the steps described below.
It is recommended that you follow the manual installation steps described below the first time you are installing the :code:`asclinic-system` on a computer.

.. note::

  The installation steps on this page should work for any of the following:

  * Installation on the single board computer :ref:`single board computer <single-board-computers>` that is onboard the robot.
  * Installation on a partition of a laptop or desktop computer.
  * Installation on a virtual machine.

  Note that when peripherals such as GPIO and I2C pins are not accessible on a laptop, then those installation steps still work and should be performed, 

.. contents:: Contents of this page
   :local:
   :backlinks: none
   :depth: 2



Fresh install of Operating System
*********************************

Follow the instructions for flashing an SD card with the Ubuntu operating system for the :ref:`single board computer <single-board-computers>` you are using:

* Nvidia Jetson models: follow the links on the `Jetpack SDK page <https://developer.nvidia.com/embedded/jetpack>`_ for your respective Jetson model.
* Raspberry Pi: follow the links on the `Install Ubuntu on a Raspberry Pi page <https://ubuntu.com/download/raspberry-pi>`_ for your respective Raspberry Pi model.
* Laptop or Virtual machine: download the appropriate version of Ubuntu from the `Download Ubuntu Desktop page <https://ubuntu.com/download/desktop>`_ and use installation instructions appropriate for your physical machine or virtual machine software.
  * `Virtual Box <https://www.virtualbox.org>`_ is a freely available virtual machine software option that runs on Windows, Linux, and macOS.

.. important::

  It is recommended that you install the version of Ubuntu that is officially supported for the version of ROS:

  * For `ROS1 Noetic <http://wiki.ros.org/noetic/Installation/Ubuntu>`_ -> Install Ubuntu 20.04 (for Nvidia Jetson install Jetpack 5.X).





Respond to broadcast pings
**************************

Taken from `this broadcast icmp in linux post <https://www.theurbanpenguin.com/broadcast-icmp-in-linux-and-how-to-initiate-and-protect/>`_

This makes it significantly easier to discover the IP address of the SBC when it is connected to a network, especially so if the a dynamic IP address configuration is used.

Open the :code:`/etc/sysctl.conf` file for editing:

.. code-block::

  sudo nano /etc/sysctl.conf

Add the following lines at the end of the :code:`sysctl.conf` file:

.. code-block::

  net.ipv4.icmp_echo_ignore_all=0
  net.ipv4.icmp_echo_ignore_broadcasts=0

To make the change take effect, enter the command:

.. code-block::

  sysctl -p

Then, enter the following command to check the "ignore broadcast" status:

.. code-block::

  less /proc/sys/net/ipv4/icmp/echo_ignore_broadcasts

If the output is 1 then broadcast pings are ignored, otherwise, if the output 0 then broadcast pings will be responded to.

To again ignore broadcast pings, simply remove the above changes made to the :code:`sysctl.conf` file and then enter the :code:`sysctl -p` command.





.. _install_mosh:

Install mosh
************

Mosh, which stands for mobile shell, is an alternative to ssh (which stands for secure shell). As described on the `mosh website <https://mosh.org>`_, mosh is a "remote terminal application that allows roaming and supports intermittent connectivity". Hence mosh can avoid the annoyance of broken ssh pipelines in those areas of your campus with patchy WiFi connection.

Install :code:`mosh`:

.. code-block::

  sudo apt install mosh





.. _install_i2c:

Install I2C library
*******************

Install the :code:`libi2c-dev` and :code:`i2c-tools` libraries:

.. code-block::

  sudo apt install libi2c-dev i2c-tools

Afterwards, to test the successful installation, execute the following command in a terminal:

.. code-block::

  sudo i2cdetect -y -r 1

**Note:** for the :code:`i2cdetect` command, the :code:`1` argument indicate the I2C bus.

To allow the gpiod library to be used without requiring root priviliges, add the user you are logged in with to the :code:`i2c` group (and any other users that need such access):

.. code-block::

  sudo usermod -a -G i2c $(whoami)

where :code:`$(whoami)` simply provides the username of the user that is currently logged in.




Install GPIO library
********************

Install the :code:`gpiod`, :code:`libgpiod-dev`, and :code:`libgpiod-doc` libraries:

.. code-block::

  sudo apt install gpiod libgpiod-dev libgpiod-doc

Afterwards, to test the successful installation, execute the following command in a terminal:

.. code-block::

  sudo gpiodetect

To allow the gpiod library to be used without requiring root privileges, we now add a :code:`udev` rule to give the user access to a particular gpio chip.

Creating a new user group names :code:`gpiod`:

.. code-block::

  sudo groupadd gpiod

Now add a :code:`udev` rule to give the :code:`gpiod` group access to :code:`gpiochip0`. Create the following file with you preferred editor, for example:

.. code-block::

  sudo vi /etc/udev/rules.d/60-gpiod.rules

Add the following comments and rule to the file just opened:

.. code-block::

  # udev rules for giving gpio port access to the gpiod group
  # This allows use of certain libgpiod functions without sudo
  SUBSYSTEM==\"gpio\", KERNEL==\"gpiochip0\", GROUP=\"gpiod\", MODE=\"0660\"

The first two lines are comments for a reminder for when you look back at this file in the (distant) future. The third line specifies that any members of the :code:`gpiod` group are allowed to access the :code:`gpiochip0` kernel that is part of the :code:`gpio` subsystem.

Add the user you are logged in with to the :code:`gpiod` group (and any other users that need such access):

.. code-block::

  sudo usermod -a -G gpiod $(whoami)

where :code:`$(whoami)` simply provides the username of the user that is currently logged in.

.. important::

  New :code:`udev` rules only comes into effect after a restart of the computer, after which you can check that the rule is working correctly by using the following command:

  .. code-block::

    gpioinfo gpiochip0

The following commands may be useful to check various details about the groups.

* List of all groups that the :code:`$(whoami)` user currently belongs to:

  .. code-block::

    groups $(whoami)

* List of all the members of a particular group, for example the :code:`sudo` group:

  .. code-block::

    getent group sudo

* Look at the file that list all groups and their members:

  .. code-block::

    less /etc/group




.. _install_ros:

Install ROS
***********

Follow the `ROS installation instructions <http://wiki.ros.org/ROS/Installation>`_ recommended for the version of Ubuntu installed in the step above.

.. OLDER ROS VERSIONS
  * Ubuntu 18.04: install `ROS Melodic <http://wiki.ros.org/melodic/Installation/Ubuntu>`_

* Ubuntu 20.04: install `ROS Noetic <http://wiki.ros.org/noetic/Installation/Ubuntu>`_

.. note::

  Ensure the you complete the step to initialize :code:`rosdep`.





.. _install_clone_asclinic_system:

Clone the :code:`asclinic-system` repository
********************************************

Clone the :code:`asclinic-system` repository into the desired location on your computer, the recommended location is :code:`~`, hence clone with the following commands:

.. code-block::

  cd ~
  git clone https://gitlab.unimelb.edu.au/asclinic/asclinic-system.git



Compile the ASClinic ROS package
********************************

To compile the ASClinic ROS Package, first change directory to the :code:`catkin_ws` directory, where :code:`ws` stands for workspace:

.. code-block::

  cd ~/asclinic-system/catkin_ws

Then build the asclinic ROS Package using the :code:`catkin_make` command:

.. code-block::

  catkin_make



Add ROS setup scripts to bashrc
*******************************

Add the following :code:`source` commands to the bottom of the file :code:`~/.bashrc` (replace :code:`<ros version name>` and :code:`<catkin workspace>` accordingly)

.. code-block:: bash

  source /opt/ros/<ros version name>/setup.bash
  source <catkin workspace>/devel/setup.bash

If you followed the steps :ref:`install_ros` and :ref:`install_clone_asclinic_system` above, then:

* :code:`<ros version name>` should be :code:`noetic`
* :code:`<catkin workspace>` should be :code:`~/asclinic-system/catkin_ws`


.. note::

  The catkin workspace :code:`setup.bash` script will only appear after the first :code:`catkin_make` compilation of the catkin workspace.



.. _install_rplidar_for_ros:

Install the Slamtec RPLidar ROS package
***************************************

These instructions are based on the information provided by the `git repository for the Slamtec RPLidar ROS package <https://github.com/slamtec/rplidar_ros>`_.

Clone the RPLidar ROS package into the :code:`catkin_ws/src/` directory of your :code:`asclinic-system` git repository:

.. code-block:: bash

  cd ~/asclinic-system/catkin_ws/src/
  git clone https://github.com/Slamtec/rplidar_ros.git

Remove to the :code:`.git` directory that is created as part of cloning in order to avoid having this RPLidar git repository nested inside your git repository.

.. code-block:: bash

  rm -rf rplidar_ros/.git/

.. note::

  Removing the :code:`.git` directory means that you can no longer :code:`pull` updates that Slamtec makes to the :code:`rplidar_ros` repository. Instead you would need to remove the whole :code:`rplidar_ros` directory and clone the repository again.



Symbolic USB link for Slamtec RPLidar devices
*********************************************

Add the following :code:`udev` rule so that the RPLidar device is automatically recognised when it is plugged in to a USB port. First open the file for editing:

.. code-block:: bash

  sudo nano /etc/udev/rules.d/rplidar.rules

Then add the following contents to the file and save:

.. code-block:: bash

  # Configure the rplidar device port be a fixed symbolink link
  KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="rplidar"

If you want this :code:`udev` rule to take immediate effect, then you can :code:`reload` and :code:`restart` the service:

.. code-block:: bash

  sudo service udev reload
  sudo service udev restart

When the RPLidar device is plugged in, you check the symbolic link that this rule creates by the following listing:

.. code-block:: bash

  ls -l /dev/rplidar

This means that launch files for the :code:`rpliadrNode` should add the following parameter to the node for specifying the USB port of the RPLidar device:

.. code-block:: bash

  <param name="serial_port" type="string" value="/dev/rplidar"/>


.. important::

  This step of adding a :code:`udev` rule has not been tested when multiple RPLidar devices are connected to the same compute.

.. note::

  This step of adding a :code:`udev` rule is not necessary, but it does make using the RPLidar device much more convenient. Without this step, every time you plug in the RPLidar or boot the robot, you would need to manually the mode of the USB device handle that it is allocated to, for example:

  .. code-block:: bash

    sudo chmod 0777 /dev/ttyUSB0





.. _install_v4l_utilities:

Install the Video for Linux (v4l) utilities
*******************************************

The settings of a USB camera can be adjusted using the command line interface program :code:`v4l2-ctl`, which stands for video for linux controls. This program is installed as part of the following package:

.. code-block:: bash

  sudo apt install v4l-utils


The following are additional video for linux tools that can come in handy:

.. code-block:: bash

  sudo apt install libv4l-dev
  sudo apt install qv4l2
  sudo apt install uvcdynctrl



.. _install_opencv_python:

OpenCV Installation for Python3
*******************************

In order to use OpenCV and the contributed libraries for ArUco marker detection via Python3, the package `opencv-contrib-python <https://pypi.org/project/opencv-contrib-python/>`_ needs to be installed, which is achieved with the following steps.


Install :code:`pip3`
####################

Install the python3 package manager :code:`pip3` using the following:

.. code-block:: bash

  sudo apt install python3-pip

.. note::

  You can list the installed packages that are managed by :code:`pip3` with the following command:

  .. code-block:: bash

    pip3 list

  To see all the commands and option for :code:`pip3`, display the help information in the usual fashion:

  .. code-block:: bash

    pip3 --help

  You can display the pip version, package location, and Python version it relies on, with the following command:

  .. code-block:: bash

    pip3 --version  


Upgrade :code:`pip3`
####################

Ensure that pip3 is upgraded to the latest version with the following command:

.. code-block:: bash

  sudo pip3 install --upgrade pip

Upgrade also the :code:`setuptools` and :code:`wheel` packages to avoid some subsequent installation errors that may occur:

.. code-block:: bash

  sudo pip3 install --upgrade setuptools wheel


Upgrade :code:`numpy`
#####################

The OpenCV contributions package relies on the :code:`numpy` package and requires a certain minimum version, hence upgrade to the latest version of :code:`numpy` with the following command:

.. code-block:: bash

  sudo pip3 install --upgrade numpy


Install OpenCV Contributions for python
#######################################

Install the 

.. code-block:: bash

  sudo pip3 install opencv-contrib-python

Updates to the :code:`opencv-contrib-python` can cause warnings and/or errors to be display during the installation. Pay attention to these warnings/errors and action accordingly to upgrade/ammend the packages mentioned in the warnings/errors.


.. important::

  The **dangers** of using :code:`sudo pip3 install`!!

  A quick google uncovers many blog post about:

  * Avoiding using :code:`sudo` with :code:`pip3 install`, because it gives root privileges to potentially malicious pip installation scripts.
  * Versus using :code:`pip3 install --user` withOUT :code:`sudo`,  meaning that the package is installed only for the user running the command, i.e., it is installed in :code:`~/local/lib/python/` and you need to add this directory to your :code:`PATH` environment variable so that the installed package is available to execute.

  The instructions above use :code:`sudo` to install the :code:`opencv-contrib-python` packages because we trust developer and distributor of the package (though even a trusted source can be hacked), and it saves some extra configuration steps.

  If you are performing this installation on a computer where you wish to avoid the :code:`sudo pip3 install` risks, then the following should work:

  .. code-block:: bash

    pip3 install --user opencv-contrib-python
    echo "" >> ~/.bashrc
    echo "# Add my local python directory to the PATH environment variable" >> ~/.bashrc
    echo "export PATH=$PATH:~/.local/lib/python" >> ~/.bashrc
    source ~/.bashrc
    echo $PATH

  The more formal option it to use :code:`virtualenv`, for which there are plenty of good tutorials online.



Install ROS packages for running python3 scripts
################################################

.. important::

  This step is only required if you are running ROS with Ubuntu 18.04 or earlier, for example, ROS Melodic on Ubuntu 18.04.

  **Otherwise, you do NOT need to perform this step.**

In order to run python3 node in ROS, run the following installation commands in order:

.. code-block:: bash

  sudo apt install python3-pip python3-yaml

.. code-block:: bash

  pip3 install --user rospkg catkin_pkg


You can now run a python node in ROS as python3, simply adjust the very first line of the script to the following:

.. code-block:: python

  #!/usr/bin/env python3



Extra steps for some camera use cases
#####################################

After following the installation steps in the sections above, you should be able to run a python3 ROS node the call OpenCV and ArUco functions. However, certain errors may still occur when calling certain functions. **As always with programming, read the details of the error and attempt to determine whether a package is missing that needs to be installed.**

For example, the OpenCV function :code:`imshow()` may need the following package to be installed:

.. code-block::

  sudo apt install libcanberra-gtk0 libcanberra-gtk-module





.. EXTRA COMMANDS THAT WERE TRIED BUT ARE POSSIBLY NOT NEEDED
  # Uninstall opencv-python
  #pip3 uninstall opencv-python
  # This step is probably not needed because this returned the message:
  #   Cannot uninstall requirement opencv-python, not installed

  # Add the boost library as required in the CMakeList.txt
  #find_package(Boost REQUIRED python3)
  # Though it is not clear if this really needs to be added

  # Set the following environment variable (and add to .bashrc)
  #export  OPENBLAS_CORETYPE=ARMV8
  # Though it all seemed to work without setting this



.. OUTDATED INSTRUCTIONS
  Disable IPv6
  ************

  Taken from `this disable ipv6 post <https://www.configserverfirewall.com/ubuntu-linux/ubuntu-disable-ipv6/>`_

  It is not necessary to disable IPv6, but some forums mention that having IPv6 can cause unexpected network behaviour under certain circumstances.

  Open the :code:`/etc/sysctl.conf` file for editing:

  .. code-block::

    sudo nano /etc/sysctl.conf

  Add the following lines at the end of the :code:`sysctl.conf` file:

  .. code-block::

    net.ipv6.conf.all.disable_ipv6 = 1
    net.ipv6.conf.default.disable_ipv6 = 1
    net.ipv6.conf.lo.disable_ipv6 = 1

  To make the change take effect, enter the command:

  .. code-block::

    sysctl -p

  Then, enter the following command to check the IPv6 status:

  .. code-block::

    less /proc/sys/net/ipv6/conf/all/disable_ipv6

  If the output is 1 then IPv6 is disabled, otherwise, if the output 0 then IPv6 is enabled.

  To re enable IPv6 addresses, simply remove the above changes made to the :code:`sysctl.conf` file and then enter the :code:`sysctl -p` command.



.. OUTDATED INSTRUCTIONS
  OpenCV Installation
  *******************

  In order to use OpenCV and the included libraries for ArUco marker detection, certain packages need to be installed.


  Install pip3
  ############

  The installation steps in this workflow are for using OpenCV and the ArUco library via pyhton3. Hence install the python3 package manager :code:`pip3` using the following:

  .. code-block:: bash

    sudo apt install python3-pip


  Install OpenCV Contributions for python
  #######################################

  The OpenCV contributions package relies on a number of other python3 packages that need to be installed first. Run the following installation commands in order:

  .. code-block:: bash

    pip3 install scikit-build

  .. code-block:: bash

    pip3 install Cython

  .. code-block:: bash

    pip3 install numpy

  .. code-block:: bash

    pip3 install opencv-contrib-python



  When each of the above installation steps is complete, it should return something similar to the following:

  .. code-block:: bash

    Successfully installed distro-1.5.0 packaging-20.9 pyparsing-2.4.7 scikit-build-0.11.1 setuptools-56.0.0 wheel-0.36.2

  .. code-block:: bash

    Successfully installed Cython-0.29.23

  .. code-block:: bash

    Successfully installed numpy-1.19.5

  .. code-block:: bash

    Successfully installed numpy-1.19.5 opencv-contrib-python-4.5.1.48
