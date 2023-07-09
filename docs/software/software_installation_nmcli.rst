.. _software-installation-nmcli:

CLI for Managing Network Connections
====================================

The Network Manager Command Line Interface (:code:`nmcli`) is the default tool installed on Ubuntu for managing the computer's network connection from the command line.

An example situation that provides context for what we wish to achieve with :code:`nmcli` is the following:

* The robot's computer is connected via the Ethernet port to your router at home.
* Your personal laptop is connected to the WiFi network of your home router.
* From your personal laptop you are logged-in to the robot's computer via :code:`ssh`.
* You wish to connect the robot's computer to the hotspot of your smartphone.
* Once the robot's computer is connected to the hotspot of your smartphone, then you can connect your personal laptop to the smartphone hotspot, and then you can run the robot from any location where you have the robot and your personal laptop connected to the hotspot of your smartphone.


Available devices and status
****************************

A good starting point is to always list the available networking devices and their status:

.. code-block::

  nmcli device status


Display available WiFi networks
*******************************

.. note::

  If any of the following commands do not work as expected, then try again with :code:`sudo`

.. note::

  In the following commands :code:`device` can be replaced by :code:`dev` if you want to save typing a few characters, the functionality is identical.

The simplest command to display available WiFi networks:

.. code-block::

  nmcli device wifi

If a network you are expecting is not visible, then you can force a rescan of the available wifi networks:

.. code-block::

  nmcli device wifi rescan

To display additional information about the scanned networks:

.. code-block::

  nmcli --fields ALL device wifi


Connecting to a new WiFi network
********************************

Use the network SSID listed using the command above, i.e., the network name, to connect to that WiFi network:

.. code-block::

  sudo nmcli device wifi connect <network-ssid> password "<network-password>"

where :code:`<network-ssid>` and :code:`<network-password>` are replaced accordingly. You can also leave off the password part of the command and you will be prompted to provide the password.

.. note::

  If you are connected to the computer via :code:`ssh` (or similar), then when you give the command to connect to a different (WiFi) network, your :code:`ssh` connection will break.

Working with saved connections
******************************

.. note::

  In the following commands :code:`connection` can be replaced by :code:`con` if you want to save typing a few characters, the functionality is identical.

Show saved connections:

.. code-block::

  nmcli connection show
    
Disconnect from a current connection:

.. code-block::

  nmcli connection down <network-ssid>

Connect to, or change the connection to, a saved network:

.. code-block::

  nmcli connection up <network-ssid>.

For these previous two commands, :code:`<network-ssid>` should replaced by the SSID of the network, i.e., the network name.



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
