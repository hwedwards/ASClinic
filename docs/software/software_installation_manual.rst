.. _software-installation-manual:

Manual Installation
===================


The :ref:`installation script <software-installation-script>` automates the steps described below.
It is recommended that you follow the manual installation steps described below the first time you are installing the :code:`asclinic-system` on a :ref:`single board computer <single-board-computers>`.



Flash SD Card
*************

Follow the instructions for flashing an SD card with the Ubuntu operating system for the :ref:`single board computer <single-board-computers>` you are using:

* Jetson Xavier NX: follow the `getting started <https://developer.nvidia.com/embedded/learn/get-started-jetson-xavier-nx-devkit>`_ guide.




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

If the output is 1 then broadcast pings are ignored, otherwise, if the output 0 then broadcast pings will be reponded to.

To again ignore broadcast pings, simply remove the above changes made to the :code:`sysctl.conf` file and then enter the :code:`sysctl -p` command.



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



Install ROS
***********

Follow the `ROS installation instructions <http://wiki.ros.org/ROS/Installation>`_ recommended for the version of Ubuntu installed in the step above.

* Ubuntu 18.04: install `ROS Melodic <http://wiki.ros.org/melodic/Installation/Ubuntu>`_
* Ubuntu 20.04: install `ROS Noetic <http://wiki.ros.org/noetic/Installation/Ubuntu>`_

**Note:** ensure the you complete the step to initialize :code:`rosdep`.



Install I2C library
*******************

Install the :code:`libi2c-dev` and :code:`i2c-tools` libraries:

.. code-block::

  sudo apt-get install libi2c-dev i2c-tools

Afterwards, to test the successful installation, execute the following command in a terminal:

.. code-block::

  sudo i2cdetect -y -r 1

**Note:** for the :code:`i2cdetect` command, the :code:`1` argument indicate the I2C bus.



Install GPIO library
********************

Install the :code:`gpiod`, :code:`libgpiod-dev`, and :code:`libgpiod-doc` libraries:

.. code-block::

  sudo apt-get install gpiod

Afterwards, to test the successful installation, execute the following command in a terminal:

.. code-block::

  sudo gpiodetect



Clone this repository
*********************

Clone the :code:`asclinic-system` repository into the desired location on your SBC, the recommended location is :code:`~`:

.. code-block::

  cd ~
  git clone https://gitlab.unimelb.edu.au/asclinic/asclinic-system.git



Compile the ROS package
***********************

To compile the asclinic ROS Package, first change directory to the :code:`catkin_ws` directory, where :code:`ws` stands for workspace:

.. code-block::

  cd ~/asclinic-system/catkin_ws

Then build the asclinic ROS Package using the :code:`catkin_make` command:

.. code-block::

  catkin_make



Add ROS setup scripts to bashrc
*******************************

Add the following :code:`source` commands to the bottom of the file :code:`~/.bashrc` (replace :code:`<ros version name>` and :code`<catkin workspace>` accordingly)

.. code-block::

  source /opt/ros/<ros version name>/setup.bash
  source <catkin workspace>/devel/setup.bash

If you followed the steps above, then:

* :code:`<ros version name>` should be either :code:`melodic` or :code:`noetic`
* :code:`<catkin workspace>` should be :code:`~/asclinic-system/catkin_ws`

**Note:** the workspace setup script will only appear after the first compilation of the catkin workspace.


OpenCV Installation
*******************

In order to use OpenCV and the included libraries for ArUco marker detection, certain packages need to be installed. These installation steps are currently **NOT** included in the :ref:`software-installation-manual` nor in the :ref:`software-installation-script`.


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




Install ROS packages for running python3 scripts
################################################

In order to run python3 node in ROS, run the following installation commands in order:

.. code-block:: bash

  sudo apt-get install python3-pip python3-yaml

.. code-block:: bash

  pip3 install rospkg catkin_pkg --user


You can now run a python node in ROS as python3, simply adjust the very first line of the script to the following:

.. code-block:: python

  #!/usr/bin/env python3



Extra steps
###########

After following the installation steps in the sections above, you should be able to run a python3 ROS node the call OpenCV and ArUco functions. However, certain errors may still occur when calling certain functions. As always with programming, read the details of the error and attempt to determine whether a package is missing that needs to be installed.

For example, the OpenCV function :code:`imshow()` needs the following package to be installed:

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

