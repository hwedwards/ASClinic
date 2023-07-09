.. _building-block-usb-camera-settings:

USB Camera Settings
===================

.. important::

  Adjusting the settings of a USB camera as described on this page may not persist when running code that accesses the camera, hence:
  
  * Use this page to get familiar with the options available for the camera you are using.
  
  * Explicitly adjust the desired settings within any code you write that accesses the camera.


Install the v4l2-ctl command line tool
**************************************

Mostly likely this is already installed because it is included in both the :ref:`software-installation-manual` and :ref:`software-installation-script`. If not already installed, follow the :ref:`install_v4l_utilities` instructions.

Find the device
***************

An important part of adjusting the USB camera settings via command line is that you must always specify which :code:`device` the settings are for. Regardless of whether one or multiple USB cameras are plugged in to the computer, it is good practice to list all of the detected devices to be sure that you are adjusting the settings of the USB camera that you intend. List all of the detected USB camera devices using the following:

.. code-block:: bash

  v4l2-ctl --list-devices

The output display of this command may look something like this:

.. code-block:: bash

  C922 Pro Stream Webcam usb-0000:00:14.0-1):
    /dev/video2
    /dev/video3

  Integrated_Webcam_HD: Integrate (usb-0000:00:14.0-12):
    /dev/video0
    /dev/video1


In which case device :code:`/dev/video2` is used for adjusting the settings of the Logitech C922 Pro Stream Webcam.


.. note::

  The examples given on the remainder of this page use :code:`/dev/video0`; **remember to change this as required if you copy-paste any commands**.


Display the manual
******************

As with most linux command line tools, you can display the its manual using the following:

.. code-block:: bash

  v4l2-ctl --help


You are encouraged to read through the manual and explore the controls that :code:`v4l2-ctl` offers. The following sections describe how to use a selection of the available options of :code:`v4l2-ctl` command line tool.


Get and set the controls
************************

This heading raises the question: what is a "control"? "Control" is the terminology used in the manual of the :code:`v4l2-ctl` command, and hence the quickest way to find out the controls for a particular USB camera is to list all of the available controls using either of the following commands:

.. code-block:: bash

  v4l2-ctl --device=<device> --list-ctrls
  v4l2-ctl --device=<device> --list-ctrls-menus

Where :code:`<device>` is replaced by the path for the device that you wish to list all the controls for. The :code:`--list-ctrls-menus` enumerates the possible options for those controls that have multiple options indexed by an integer.

For example, list the controls for USB camera device :code:`/dev/video0` using the following:

.. code-block:: bash

  v4l2-ctl --device=/dev/video0 --list-ctrls


Get the value currently set for a particular control using the following:

.. code-block:: bash

  v4l2-ctl --device=<device> --get-ctrl=<ctrl>

Where :code:`<device>` is replaced as usual, and :code:`<ctrl>` is replaced by the name of the control as listed by the :code:`--list-ctrls` command. Note that the :code:`--get-ctrl` option can be passed a comma separated list of control names to retrieve the values of.


Set the value for a particular control using the following:

.. code-block:: bash

  v4l2-ctl --device=<device> --set-ctrl=<ctrl>=<val>

Where :code:`<device>` is replaced as usual, and :code:`<ctrl>` is replaced by the name of the control as listed by the :code:`--list-ctrls` command, and :code:`<val>` is the value to be set for that cotnrol.


Check the available video formats
*********************************

TO BE COMPLETED

# v4l2-ctl --list-formats
# v4l2-ctl --list-formats-ext


Get and set the focus
*********************

To display the current value of the boolean auto focus control and of the focus level of :code:`/dev/video0`, use the following:

.. code-block:: bash

  v4l2-ctl --device=/dev/video0 --get-ctrl=focus_auto,focus_absolute

The output display of this command may look something like this:

.. code-block:: bash

  focus_auto: 1
  focus_absolute: 100

Which means the auto focus is turned on, and the focus level is set to 100, which is between the minimum of 0 (focus at infinity) and the maximum of 250 (the minimum and maximum are displayed as part of the :code:`--list-ctrls` output).


To set the current value of the boolean auto focus control and the focus level of :code:`/dev/video0`, use the following:

.. code-block:: bash

  v4l2-ctl --device=/dev/video0 --set-ctrl=focus_auto=0
  v4l2-ctl --device=/dev/video0 --set-ctrl=focus_absolute=0

.. note::

  Setting :code:`focus_absolute` is only possible when the current value of :code:`focus_auto` is 0, otherwise attempting to set :code:`focus_absolute` will return an error. In other words, if the USB camera is operating in auto focus mode, then you are not allowed to set the focus level.


Get and set the resolution
**************************

The options for the camera resolution are grouped under what is termed "video capture format options". To display the manual entry for these options, use the following command:

.. code-block:: bash

  v4l2-ctl --help-vidcap

To get and display the current video capture format options for a particular device, use the following:

.. code-block:: bash

  v4l2-ctl --device=<device> --get-fmt-video

Where :code:`<device>` is replaced as usual. The output display of this command may look something like this:

.. code-block:: bash

  Format Video Capture:
    Width/Height      : 1920/1080
    Pixel Format      : 'MJPG'
    Field             : None
    Bytes per Line    : 0
    Size Image        : 4147200
    Colorspace        : sRGB
    Transfer Function : Default (maps to sRGB)
    YCbCr/HSV Encoding: Default (maps to ITU-R 601)
    Quantization      : Default (maps to Full Range)
    Flags


To set the video capture settings format options for a particular device, use the following:

.. code-block:: bash

  v4l2-ctl --device=<device> --set-fmt-video=width=<w>,height=<h>,pixelformat=<pf>,field=<f>,colorspace=<c>,xfer=<xf>,ycbcr=<y>,quantization=<q>,premul-alpha,bytesperline=<bpl>

Where :code:`<device>` is replaced as usual, and the remainder of the :code:`<...>` parameters are best explained by reading the manual entry (i.e., :code:`v4l2-ctl --help-vidcap`).

The :code:`--set-fmt-video` command can be used with a sub-set of the options shown above. To set the resolution (width and height) of the camera images captured by :code:`/dev/video0`, use the following:

.. code-block:: bash

  v4l2-ctl --device=/dev/video0 --set-fmt-video=width=640,height=480

This sets the resolution to 640x480, and you can check it was set correctly by using :code:`--get-fmt-video` to display the current settings. To set the resolution of :code:`/dev/video0` to be HD, use the following:

.. code-block:: bash

  v4l2-ctl --device=/dev/video0 --set-fmt-video=width=1920,height=1080



Display everything about a device
*********************************

To display all the specifications and current settings for a device, use the following:

.. code-block:: bash

  v4l2-ctl --device=<device> --all

Where :code:`<device>` is replaced as usual. For example, to display all the detail for :code:`/dev/video0`, use the following:

.. code-block:: bash

  v4l2-ctl --device=/dev/video0 --all

The output display of this command may look something like this:


.. code-block:: bash

  Driver Info (not using libv4l2):
  Driver name   : uvcvideo
  Card type     : C922 Pro Stream Webcam
  Bus info      : usb-0000:00:14.0-1
  Driver version: 5.4.94
  Capabilities  : 0x84A00001
    Video Capture
    Metadata Capture
    Streaming
    Extended Pix Format
    Device Capabilities
  Device Caps   : 0x04200001
    Video Capture
    Streaming
    Extended Pix Format
  Priority: 2
  Video input : 0 (Camera 1: ok)
  Format Video Capture:
    Width/Height      : 1920/1080
    Pixel Format      : 'MJPG'
    Field             : None
    Bytes per Line    : 0
    Size Image        : 4147200
    Colorspace        : sRGB
    Transfer Function : Default (maps to sRGB)
    YCbCr/HSV Encoding: Default (maps to ITU-R 601)
    Quantization      : Default (maps to Full Range)
    Flags             :
  Crop Capability Video Capture:
    Bounds      : Left 0, Top 0, Width 1920, Height 1080
    Default     : Left 0, Top 0, Width 1920, Height 1080
    Pixel Aspect: 1/1
  Selection: crop_default, Left 0, Top 0, Width 1920, Height 1080
  Selection: crop_bounds, Left 0, Top 0, Width 1920, Height 1080
  Streaming Parameters Video Capture:
    Capabilities     : timeperframe
    Frames per second: 30.000 (30/1)
    Read buffers     : 0
                        brightness 0x00980900 (int)    : min=0 max=255 step=1 default=128 value=128
                          contrast 0x00980901 (int)    : min=0 max=255 step=1 default=128 value=128
                        saturation 0x00980902 (int)    : min=0 max=255 step=1 default=128 value=128
    white_balance_temperature_auto 0x0098090c (bool)   : default=1 value=1
                              gain 0x00980913 (int)    : min=0 max=255 step=1 default=0 value=0
              power_line_frequency 0x00980918 (menu)   : min=0 max=2 default=2 value=2
         white_balance_temperature 0x0098091a (int)    : min=2000 max=6500 step=1 default=4000 value=4000 flags=inactive
                         sharpness 0x0098091b (int)    : min=0 max=255 step=1 default=128 value=128
            backlight_compensation 0x0098091c (int)    : min=0 max=1 step=1 default=0 value=0
                     exposure_auto 0x009a0901 (menu)   : min=0 max=3 default=3 value=3
                 exposure_absolute 0x009a0902 (int)    : min=3 max=2047 step=1 default=250 value=250 flags=inactive
            exposure_auto_priority 0x009a0903 (bool)   : default=0 value=1
                      pan_absolute 0x009a0908 (int)    : min=-36000 max=36000 step=3600 default=0 value=0
                     tilt_absolute 0x009a0909 (int)    : min=-36000 max=36000 step=3600 default=0 value=0
                    focus_absolute 0x009a090a (int)    : min=0 max=250 step=5 default=0 value=0
                        focus_auto 0x009a090c (bool)   : default=1 value=0
                     zoom_absolute 0x009a090d (int)    : min=100 max=500 step=1 default=100 value=100
                         led1_mode 0x0a046d05 (menu)   : min=0 max=3 default=0 value=3
                    led1_frequency 0x0a046d06 (int)    : min=0 max=255 step=1 default=0 value=24



Quick Setup
***********

Every time that the USB camera is plugged in, or the computer reboots, then the settings may revert to default. Hence it is worthwhile to create a simple shell for adjusting the setting that are relevant to your use case.

For example, if you wish to disable auto focus, adjust the focus level to 0, and capture HD images, then create a shell script named :code:`my_camera_setup.sh` with the following contents:

.. code-block:: bash

  DEVICE=/dev/video0
  echo "Now adjusting the camera settings for $DEVICE"
  v4l2-ctl --device=$DEVICE --set-ctrl=focus_auto=0
  v4l2-ctl --device=$DEVICE --set-ctrl=focus_absolute=0
  v4l2-ctl --device=$DEVICE --set-fmt-video=width=1920,height=1080
  echo " "
  echo "Camera settings were adjusted to:"
  v4l2-ctl --device=$DEVICE --get-ctrl=focus_auto,focus_absolute
  v4l2-ctl --device=$DEVICE --get-fmt-video


**Remember:** adjust the line :code:`DEVICE=/dev/video0` to correctly specify the USB camera device for your use case.

Adjust this script to be executable using the following:

.. code-block:: bash

  chmod +x my_camera_setup.sh

Run the script using: :code:`./my_camera_setup.sh`


External links
**************

`This website <https://www.kurokesu.com/main/2016/01/16/manual-usb-camera-settings-in-linux/>`_ provides explanations for a similar set of commands.



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
