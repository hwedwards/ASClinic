#!/bin/bash

# ============================================ #
# USER SPECIFICATIONS

# NOTE THE FOLLOWING
# > A true/false flag will ONLY be processed as
#   true if it is set to exactly the string
#   "true".



# USEFUL CONFIGURATIONS
SHOULD_SET_RESPOND_TO_BROADCAST_PINGS="true"



# I2C AND GPIO SETUP
SHOULD_INSTALL_AND_SETUP_I2C="true"
SHOULD_INSTALL_AND_SETUP_GPIO="true"



# ROS INSTALLATION:
SHOULD_INSTALL_ROS="true"
ROS_CONFIGURATION="desktop-full"
# > Options for config:
# "desktop-full"  (Recommended for a GUI-based operating system on a "normal" machine)
# "desktop"       (Recommended for a GUI-based operating system on a "low-power" machine))
# "ros-base"      (Recommended for a "headless" operating system, i.e., when running ubuntu server, also referred to as "ROS-Base: (Bare Bones)")



# ASCLINIC SYSTEM SETUP:
SHOULD_SETUP_ASCLINIC_SYSTEM_LOCALLY="true"
PATH_FOR_ASCLINIC_SYSTEM_LOCALLY="/home/$(whoami)/"
# > Note for the "asclinic system locally":
#   This is intended for the logged in user
#   that runs this installation script.



# CAMERA UTILITIES
SHOULD_INSTALL_VIDEO_FOR_LINUX_UTILITIES="true"



# OPEN CV CONTRIBUTED MODULES IN PYTHON
SHOULD_INSTALL_OPENCV_CONTRIB_PYTHON="true"



# END OF: USER SPECIFICATIONS
# ============================================ #




# ============================================ #
# SPECIFICATIONS WAITING TO BE IMPLEMENTED

# ASCLINIC SYSTEM WEB INTERFACE SETUP:
SHOULD_SETUP_ASCLINIC_WEB_INTERFACE="false"
# > Note for the "asclinic web interface":
#   This would perform the following.
#   BUT it is not currently setup:
#   - installs an apache web server
#   - puts a separate copy of the asclinic-system
#     in a shared location
#   - gives the "www-data" web server user
#     full access rights to this shared copy
#     of the dfall system
#   - makes the asclinic web interface the home
#     page of the apache web server

# QT INSTALLATION:
SHOULD_INSTALL_QT="false"
SHOULD_INSTALL_QT_CREATOR="false"

# BITCRAZE SOFTWARE INSTALLATION:
SHOULD_SETUP_PYUSB_FOR_CRAZYRADIO="false"
SHOULD_INSTALL_BITCRAZE_CFCLIENT="false"

# END OF: SPECIFICATIONS WAITING TO BE IMPLEMENTED
# ============================================ #





# ============================================ #
# SOME USEFUL NOTES FOR READING THIS SCRIPT
#
# > For the "apt" command, the -y option means automatic yes to prompts
#
# > For the "mkdir" command, the -p option means no error if existing, make parent directories as needed
#
# > For the "sh" command, the -c option specifies to "Read commands from the command_string operand instead of from the standard input."
#
# > For the "tee" command, the -a option specifies to "append to the given FILEs, do not overwrite"
#
# END OF: SOME USEFUL NOTES FOR READING THIS SCRIPT
# ============================================ #





# ============================================ #
# EXIT IF SCRIPT CALLED USING SUDO
if [[ $(whoami) = "root" ]]
then
	echo ""
	echo "ERROR: this installation script is NOT allowed to be"
	echo "       run by the root user. Either login as a different"
	echo "       user, or do not use \"sudo\" to call this"
	echo "       installation script."
	echo ""
	exit 1
fi
# END OF: EXIT IF SCRIPT CALLED USING SUDO
# ============================================ #


# ============================================ #
# CONFIRM THE INSTALLATION ACTIONS

# DISPLAY A STARTING MESSAGE
echo ""
echo "NOW BEGINNNING THE INSTALLATION SCRIPT"
echo ">> the user running the script is: $(whoami)"
echo ""



# USE REGULAR EXPRESSIONS TO EXTRACT UBUNTU 
# VERSION DETAILS FROM THE FILE /etc/os-release
# > See the following website for details and
#   testing of regular expressions:
#   https://regex101.com/

# > Get the Ubuntu major version number
UBUNTU_VERSION_MAJOR="$(grep -o --perl-regexp '(?<=VERSION_ID=")(\d{2,2})(?=\.\d{2,2}")' /etc/os-release)"

# > Get the Ubuntu minor version number
UBUNTU_VERSION_MINOR="$(grep -o --perl-regexp '(?<=VERSION_ID="\d{2,2}\.)(\d{2,2})(?=")' /etc/os-release)"

# > Get the Ubuntu codename
UBUNTU_VERSION_CODENAME="$(grep -o --perl-regexp '(?<=VERSION_CODENAME=)(\w{0,})' /etc/os-release)"

# > Display the details about the Ubuntu version
echo ""
echo "According to the file /etc/os-release,"
echo "the Ubuntu operating system version is:"
echo "${UBUNTU_VERSION_MAJOR}.${UBUNTU_VERSION_MINOR} (${UBUNTU_VERSION_CODENAME})"



# DISPLAY THE USER SPECIFICATIONS
echo ""
echo "The following installation actions will"
echo "be performed:"

echo ""

if [[ ${SHOULD_SET_RESPOND_TO_BROADCAST_PINGS} = "true" ]]
then
	echo ">> The \"/etc/sysctl.conf\" will be adjusted so that this computer responds to broadcast pings"
fi

if [[ ${SHOULD_INSTALL_AND_SETUP_I2C} = "true" ]]
then
	echo ">> I2C will be installed and configured"
fi

if [[ ${SHOULD_INSTALL_AND_SETUP_GPIO} = "true" ]]
then
	echo ">> GPIO will be installed and configured"
fi

if [[ ${SHOULD_INSTALL_ROS} = "true" ]]
then
	echo ">> ROS will be installed with the configuration:"
	echo "   ${ROS_CONFIGURATION}"
fi

if [[ ${SHOULD_SETUP_ASCLINIC_SYSTEM_LOCALLY} = "true" ]]
then
	echo ">> The \"asclinic-system\" will be setup locally at the path:"
	echo "   ${PATH_FOR_ASCLINIC_SYSTEM_LOCALLY}"
fi

if [[ ${SHOULD_INSTALL_VIDEO_FOR_LINUX_UTILITIES} = "true" ]]
then
	echo ">> The video for linux (v4l) camera utilities will be installed."
fi

if [[ ${SHOULD_INSTALL_OPENCV_CONTRIB_PYTHON} = "true" ]]
then
	echo ">> The OpenCV contributed modules for Python will be installed."
fi

if [[ ${SHOULD_SETUP_ASCLINIC_WEB_INTERFACE} = "true" ]]
then
	echo ">> The asclinic web interface will be setup locally at the path:"
	echo "   /home/shared/asclinic/"
fi

if [[ ${SHOULD_INSTALL_QT} = "true" ]]
then
	echo ">> The necessary QT packages will be installed"
fi

if [[ ${SHOULD_INSTALL_QT_CREATOR} = "true" ]]
then
	echo ">> QT creator will be installed"
fi

if [[ ${SHOULD_SETUP_PYUSB_FOR_CRAZYRADIO} = "true" ]]
then
	echo ">> The \"pyusb\" python pacakge will be installed,"
	echo "   and rules will be set for the crazyradio and crazyflie"
fi

if [[ ${SHOULD_INSTALL_BITCRAZE_CFCLIENT} = "true" ]]
then
	echo ">> The Bitcraze software \"cfclient\" will be installed"
fi



# GET USER CONFIRMATION TO PERFORM INSTALLATION
# > Initialise a varaible for whether to continue
SHOULD_PERFORM_INSTALLATION="false"
# > Get the user's response
echo ""
read -p "Do you wish to perform the installation? [y,n] " -n 1 -r REPLY_CONTINUE
# > Move to a new line after the users response is given
echo ""
# > Check the user's response
if [[ $REPLY_CONTINUE =~ ^[Yy]$ ]]
then
	# Update the variable
	SHOULD_PERFORM_INSTALLATION="true"
fi

if [[ ${SHOULD_PERFORM_INSTALLATION} = "false" ]]
then
	echo ""
	echo ""
	echo "INSTLLATION EXITED WITHOUT PERFORMING ANY ACTIONS"
	echo ""
	exit 1
else
	echo ""
	echo ""
	echo "NOW BEGINNING THE INSTALLATION"
fi
# END OF: CONFIRM THE INSTALLATION ACTIONS
# ============================================ #





# ============================================ #
# CONFIGURE TO RESPOND TO ALL PINGS

if [[ ${SHOULD_SET_RESPOND_TO_BROADCAST_PINGS} = "true" ]]
then

	DID_CHANGE_RESPOND_TO_ALL_PINGS_CONFIGURATION="false"

	# Inform the user
	echo ""
	echo ""
	echo "NOW CONFIGURING TO RESPOND TO ALL PINGS"

	# Check if already configured
	ICMP_ECHO_IGNORE_ALL="$(grep -o --perl-regexp '(?<=net.ipv4.icmp_echo_ignore_all=)(\d{1,1})' /etc/sysctl.conf)"

	if [[ ${ICMP_ECHO_IGNORE_ALL} = "0" ]]
	then
		# Check if the line is commented-out or not
		ICMP_ECHO_IGNORE_ALL_COMMENT_CHARACTER="$(grep -o --perl-regexp '(#)(?= *net.ipv4.icmp_echo_ignore_all=)' /etc/sysctl.conf)"
		if [[ ${ICMP_ECHO_IGNORE_ALL_COMMENT_CHARACTER} = "#" ]]
		then
			# Uncomment the line
			sed -i '/net.ipv4.icmp_echo_ignore_all=/s/^#//g' /etc/sysctl.conf
			# Inform the user
			echo ""
			echo ">> \"/etc/sysctl.conf\" already contained the configuration to respond to all pings,"
			echo "   but it needed to be uncommended, i.e., the relevant line is now configured to:"
			echo "   $(grep -w net.ipv4.icmp_echo_ignore_all /etc/sysctl.conf)"
			# Set the flag
			DID_CHANGE_RESPOND_TO_ALL_PINGS_CONFIGURATION="true"
		else
			# Inform the user
			echo ""
			echo ">> \"/etc/sysctl.conf\" already configured to respond to all pings,"
			echo "   i.e., the relevant line is:"
			echo "   $(grep -w net.ipv4.icmp_echo_ignore_all /etc/sysctl.conf)"
		fi

	else
		# Change if the line already exists
		if [[ ${ICMP_ECHO_IGNORE_ALL} = "1" ]]
		then
			# Check if the line is commented-out or not
			ICMP_ECHO_IGNORE_ALL_COMMENT_CHARACTER="$(grep -o --perl-regexp '(#)(?= *net.ipv4.icmp_echo_ignore_all=)' /etc/sysctl.conf)"
			if [[ ${ICMP_ECHO_IGNORE_ALL_COMMENT_CHARACTER} = "#" ]]
			then
				# Uncomment the line
				sed -i '/net.ipv4.icmp_echo_ignore_all=/s/^#//g' /etc/sysctl.conf
				# Change configuration from "1" to "0"
				sed -i 's/net.ipv4.icmp_echo_ignore_all=1/net.ipv4.icmp_echo_ignore_all=0/g' /etc/sysctl.conf
				# Inform the user
				echo ""
				echo ">> \"/etc/sysctl.conf\" already contained the configuration to ignore all pings,"
				echo "   but it needed to be uncommented and adjusted, i.e., the relevant line is now configured to:"
				echo "   $(grep -w net.ipv4.icmp_echo_ignore_all /etc/sysctl.conf)"
				# Set the flag
				DID_CHANGE_RESPOND_TO_ALL_PINGS_CONFIGURATION="true"
			else
				# Inform the user
				echo ""
				echo ">> \"/etc/sysctl.conf\" currently configured to ignore all pings,"
				echo "   i.e., the relevant line is currently:"
				echo "   $(grep -w net.ipv4.icmp_echo_ignore_all /etc/sysctl.conf)"
				# Change configuration from "1" to "0"
				sed -i 's/net.ipv4.icmp_echo_ignore_all=1/net.ipv4.icmp_echo_ignore_all=0/g' /etc/sysctl.conf
				# Inform the user
				echo ""
				echo ">> \"/etc/sysctl.conf\" adjusted to repond to all pings,"
				echo "   i.e., the relevant line is now:"
				echo "   $(grep -w net.ipv4.icmp_echo_ignore_all /etc/sysctl.conf)"
				# Set the flag
				DID_CHANGE_RESPOND_TO_ALL_PINGS_CONFIGURATION="true"
			fi
			
		else
			# Append the neccessary line to the file
			echo "" | sudo tee -a /etc/sysctl.conf
			echo "###################################################################" | sudo tee -a /etc/sysctl.conf
			echo "# Respond to all pings" | sudo tee -a /etc/sysctl.conf
			echo "net.ipv4.icmp_echo_ignore_all=0" | sudo tee -a /etc/sysctl.conf
			# Inform the user
			echo ""
			echo ">> \"/etc/sysctl.conf\" appended with the configuration to repond to all pings,"
			echo "   i.e., the relevant line added is:"
			echo "   $(grep -w net.ipv4.icmp_echo_ignore_all /etc/sysctl.conf)"
			# Set the flag
			DID_CHANGE_RESPOND_TO_ALL_PINGS_CONFIGURATION="true"
		fi
	fi

	# Reload the "sysctl.conf"
	if [[ ${DID_CHANGE_RESPOND_TO_ALL_PINGS_CONFIGURATION} = "true" ]]
	then
		echo ""
		echo ">> Now reloading configuration using command \"sudo sysctl -p\""
		sudo sysctl -p
		echo ""
	fi
fi
# END OF: CONFIGURE TO RESPOND TO ALL PINGS
# ============================================ #





# ============================================ #
# CONFIGURE TO RESPOND TO BROADCAST PINGS

if [[ ${SHOULD_SET_RESPOND_TO_BROADCAST_PINGS} = "true" ]]
then

	DID_CHANGE_RESPOND_TO_BROADCAST_PINGS_CONFIGURATION="false"

	# Inform the user
	echo ""
	echo ""
	echo "NOW CONFIGURING TO RESPOND TO BROADCAST PINGS"

	# Check if already configured
	ICMP_ECHO_IGNORE_BROADCASTS="$(grep -o --perl-regexp '(?<=net.ipv4.icmp_echo_ignore_broadcasts=)(\d{1,1})' /etc/sysctl.conf)"

	if [[ ${ICMP_ECHO_IGNORE_BROADCASTS} = "0" ]]
	then
		# Check if the line is commented-out or not
		ICMP_ECHO_IGNORE_BROADCASTS_COMMENT_CHARACTER="$(grep -o --perl-regexp '(#)(?= *net.ipv4.icmp_echo_ignore_broadcasts=)' /etc/sysctl.conf)"
		if [[ ${ICMP_ECHO_IGNORE_BROADCASTS_COMMENT_CHARACTER} = "#" ]]
		then
			# Uncomment the line
			sed -i '/net.ipv4.icmp_echo_ignore_broadcasts=/s/^#//g' /etc/sysctl.conf
			# Inform the user
			echo ""
			echo ">> \"/etc/sysctl.conf\" already contained the configuration to respond to broadcast pings,"
			echo "   but it needed to be uncommended, i.e., the relevant line is now configured to:"
			echo "   $(grep -w net.ipv4.icmp_echo_ignore_broadcasts /etc/sysctl.conf)"
			# Set the flag
			DID_CHANGE_RESPOND_TO_BROADCAST_PINGS_CONFIGURATION="true"
		else
			# Inform the user
			echo ""
			echo ">> \"/etc/sysctl.conf\" already configured to respond to broadcast pings,"
			echo "   i.e., the relevant line is:"
			echo "   $(grep -w net.ipv4.icmp_echo_ignore_broadcasts /etc/sysctl.conf)"
		fi

	else
		# Change if the line already exists
		if [[ ${ICMP_ECHO_IGNORE_BROADCASTS} = "1" ]]
		then
			# Check if the line is commented-out or not
			ICMP_ECHO_IGNORE_BROADCASTS_COMMENT_CHARACTER="$(grep -o --perl-regexp '(#)(?= *net.ipv4.icmp_echo_ignore_broadcasts=)' /etc/sysctl.conf)"
			if [[ ${ICMP_ECHO_IGNORE_BROADCASTS_COMMENT_CHARACTER} = "#" ]]
			then
				# Uncomment the line
				sudo sed -i '/net.ipv4.icmp_echo_ignore_broadcasts=/s/^#//g' /etc/sysctl.conf
				# Change configuration from "1" to "0"
				sudo sed -i 's/net.ipv4.icmp_echo_ignore_broadcasts=1/net.ipv4.icmp_echo_ignore_broadcasts=0/g' /etc/sysctl.conf
				# Inform the user
				echo ""
				echo ">> \"/etc/sysctl.conf\" already contained the configuration to ignore broadcast pings,"
				echo "   but it needed to be uncommented and adjusted, i.e., the relevant line is now configured to:"
				echo "   $(grep -w net.ipv4.icmp_echo_ignore_broadcasts /etc/sysctl.conf)"
				# Set the flag
				DID_CHANGE_RESPOND_TO_BROADCAST_PINGS_CONFIGURATION="true"
			else
				# Inform the user
				echo ""
				echo ">> \"/etc/sysctl.conf\" currently configured to ignore broadcast pings,"
				echo "   i.e., the relevant line is currently:"
				echo "   $(grep -w net.ipv4.icmp_echo_ignore_broadcasts /etc/sysctl.conf)"
				# Change configuration from "1" to "0"
				sed -i 's/net.ipv4.icmp_echo_ignore_broadcasts=1/net.ipv4.icmp_echo_ignore_broadcasts=0/g' /etc/sysctl.conf
				# Inform the user
				echo ""
				echo ">> \"/etc/sysctl.conf\" adjusted to repond to broadcast pings,"
				echo "   i.e., the relevant line is now:"
				echo "   $(grep -w net.ipv4.icmp_echo_ignore_broadcasts /etc/sysctl.conf)"
				# Set the flag
				DID_CHANGE_RESPOND_TO_BROADCAST_PINGS_CONFIGURATION="true"
			fi
			
		else
			# Append the neccessary line to the file
			echo "" | sudo tee -a /etc/sysctl.conf
			echo "###################################################################" | sudo tee -a /etc/sysctl.conf
			echo "# Respond to broadcast pings" | sudo tee -a /etc/sysctl.conf
			echo "net.ipv4.icmp_echo_ignore_broadcasts=0" | sudo tee -a /etc/sysctl.conf
			# Inform the user
			echo ""
			echo ">> \"/etc/sysctl.conf\" appended with the configuration to repond to broadcast pings,"
			echo "   i.e., the relevant line added is:"
			echo "   $(grep -w net.ipv4.icmp_echo_ignore_broadcasts /etc/sysctl.conf)"
			# Set the flag
			DID_CHANGE_RESPOND_TO_BROADCAST_PINGS_CONFIGURATION="true"
		fi
	fi

	# Reload the "sysctl.conf"
	if [[ ${DID_CHANGE_RESPOND_TO_BROADCAST_PINGS_CONFIGURATION} = "true" ]]
	then
		echo ""
		echo ">> Now reloading configuration using command \"sudo sysctl -p\""
		sudo sysctl -p
		echo ""
	fi
fi
# END OF: CONFIGURE TO RESPOND TO BROADCAST PINGS
# ============================================ #





# ============================================ #
# INSTALL I2C

if [[ ${SHOULD_INSTALL_AND_SETUP_I2C} = "true" ]]
then
	# Inform the user
	echo ""
	echo ""
	echo "NOW INSTALLING AND CONFIGURING I2C"

	# Install the "i2c-tools" pacakge
	echo ""
	echo ">> Now installing \"i2c-tools\" package:"
	echo ""
	sudo apt -y install i2c-tools
	
	# Install the "libi2c-dev" pacakge
	echo ""
	echo ">> Now installing \"libi2c-dev\" package:"
	echo ""
	sudo apt -y install libi2c-dev

	# Inform the user, and check that the installation worked
	echo ""
	echo ">> Nothing further to configure."
	echo ">> Now checking the installation was successful by running the command:"
	echo ">> sudo i2cdetect -y -r 1"
	echo ""
	sudo i2cdetect -y -r 1
fi
# END OF: INSTALL I2C
# ============================================ #





# ============================================ #
# INSTALL GPIO

if [[ ${SHOULD_INSTALL_AND_SETUP_GPIO} = "true" ]]
then
	# Inform the user
	echo ""
	echo ""
	echo "NOW INSTALLING AND CONFIGURING GPIO"

	# Install the "gpiod" pacakge
	echo ""
	echo ">> Now installing \"gpiod\" package:"
	echo ""
	sudo apt -y install gpiod
	echo ""
	
	# Install the "libgpiod-dev" pacakge
	echo ""
	echo ">> Now installing \"libgpiod-dev\" package:"
	echo ""
	sudo apt -y install libgpiod-dev
	echo ""

	# Install the "libgpiod-doc" pacakge
	echo ""
	echo ">> Now installing \"libgpiod-doc\" package:"
	echo ""
	sudo apt -y install libgpiod-doc
	echo ""

	# Create the user group "gpiod"
	echo ""
	echo ">> Now creating a new user group names \"gpiod\"."
	sudo groupadd gpiod
	echo ""

	# Add a "udev" rule to give the "gpiod" group access
	# to "gpiochip0"
	# > First check if there is already such a rule
	if [[ -f "/etc/udev/rules.d/60-gpiod.rules" ]]
	then
		# > Inform the user
		echo ""
		echo ">> A \"udev\" rule named \"60-gpiod.rules\" already exists and has the contents:"
		echo ""
		cat /etc/udev/rules.d/60-gpiod.rules

		echo ""
		echo ">> This file will be removed"
		sudo rm /etc/udev/rules.d/60-gpiod.rules
	fi
	# > Inform the user
	echo ""
	echo ">> Now adding a \"udev\" rule named \"60-gpiod.rules\" with the following content:"
	# > Add the "udev" rule
	echo "# udev rules for giving gpio port access to the gpiod group" | sudo tee -a /etc/udev/rules.d/60-gpiod.rules
	echo "# This allows use of certain libgpiod functions without sudo" | sudo tee -a /etc/udev/rules.d/60-gpiod.rules
	echo "SUBSYSTEM==\"gpio\", KERNEL==\"gpiochip0\", GROUP=\"gpiod\", MODE=\"0660\"" | sudo tee -a /etc/udev/rules.d/60-gpiod.rules
	# > Display the "udev" rule that was added
	echo ""
	echo ">> As a double check, the full contents of \"60-gpiod.rules\" is now:"
	cat /etc/udev/rules.d/60-gpiod.rules
	
	# Add the logged in user to the "gpiod" group
	echo ""
	echo ">> Now adding you (i.e., the $(whoami) user) to the \"gpiod\" group."
	sudo usermod -a -G gpiod $(whoami)

	# Inform the user
	echo ""
	echo ">> The following is a list of all groups that the $(whoami) user currently belongs to:"
	groups $(whoami)

	# Inform the user, and check that the installation worked
	echo ""
	echo ">> The GPIO congiruation is now complete."
	echo ">> Now checking the installation was successful by running the command:"
	echo ">> sudo gpiodetect"
	echo ""
	sudo gpiodetect

	echo ""
	echo ">> The \"udev\" rules comes into effect after a restart,"
	echo "   you can then check that the rule is working correctly"
	echo "   by using the following command:"
	echo ">> gpioinfo gpiochip0"
	echo ""
fi
# END OF: INSTALL GPIO
# ============================================ #





# ============================================ #
# INSTALL ROS

if [[ ${SHOULD_INSTALL_ROS} = "true" ]]
then
	# Convert the Major Ubunter Version to the ROS version
	if [[ ${UBUNTU_VERSION_MAJOR} = "16" ]]
	then
		ROS_VERSION_CODENAME="kinetic"
	else
		if [[ ${UBUNTU_VERSION_MAJOR} = "18" ]]
		then
			ROS_VERSION_CODENAME="melodic"
		else
			if [[ ${UBUNTU_VERSION_MAJOR} = "20" ]]
			then
				ROS_VERSION_CODENAME="noetic"
			else
				# Inform the user
				echo ""
				echo ""
				echo "[ERROR] The ROS installation cannot be completed because"
				echo "        Ubuntu ${UBUNTU_VERSION_MAJOR}.${UBUNTU_VERSION_MINOR} is NOT supported by this"
				echo "         installation script."
				echo ""
				echo ""
				# Set the flag to false
				SHOULD_INSTALL_ROS="false"
			fi
		fi
	fi

	if [[ ${SHOULD_INSTALL_ROS} = "true" ]]
	then
		# Inform the user
		echo ""
		echo ""
		echo "NOW INSTALLING ROS"


		# INSTALL ROS AS PER:
		# http://wiki.ros.org/melodic/Installation/Ubuntu

		# Setup the computer to accept software from packages.ros.org
		echo ""
		echo ">> Now adding packages.ros.org to the software sources list"
		sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

		# Setup the keys
		echo ""
		echo ">> Now setting up the keys"
		#sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
		#sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
		#sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
		sudo apt -y install curl
		curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

		# Update the package index, i.e., the list of programs
		echo ""
		echo ">> Now updating the Debian package index"
		sudo apt update

		# Install all available upgrades
		# > Note: the -y option means Automatic yes to prompts
		#echo ">> Now installing all available upgrade for installed packages"
		#sudo apt -y upgrade

		# Install ROS Melodic
		echo ""
		echo ">> Now installating the package ros-${ROS_VERSION_CODENAME}-${ROS_CONFIGURATION}"
		sudo apt -y install ros-${ROS_VERSION_CODENAME}-${ROS_CONFIGURATION}

		# Add the ROS environment setup to the .bashrc
		# > Note: added together with a description in comments
		echo ""
		echo ">> Now adding the sourcing of the ROS environment variables to the ~/.bashrc file"
		echo "# SOURCE THE ROS setup.bash FILE" >> ~/.bashrc
		echo "# (Note: this was added as part of the asclinic-system installation)" >> ~/.bashrc
		echo "source /opt/ros/${ROS_VERSION_CODENAME}/setup.bash" >> ~/.bashrc
		# Source the file also for the current shell
		source /opt/ros/${ROS_VERSION_CODENAME}/setup.bash

		# Install rosdep and various other tool and dependencies for building ROS packages
		# > Note: need to install python3 pacakges for Ubuntu 20 and later
		if [[ ${UBUNTU_VERSION_MAJOR} -le "19" ]]
		then
			sudo apt -y install python-rosdep
			sudo apt -y install python-rosinstall python-rosinstall-generator python-wstool build-essential
			# Install a few extra things to allow running python3 nodes in ROS
			sudo apt -y install python3-pip python3-yaml
			pip3 install rospkg catkin_pkg --user
		else
			sudo apt -y install python3-rosdep
			sudo apt -y install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
			sudo apt -y python3-yaml
		fi

		# Install, initialise and update rosdep
		# > Note: rosdep enables you to easily install system
		#   dependencies for source you want to compile and is
		#   required to run some core components in ROS
		echo ""
		echo ">> Now installing, initialising, and updating rosdep"
		sudo rosdep init
		rosdep update
	fi
fi
# END OF: INSTALL ROS
# ============================================ #





# ============================================ #
# SETUP THE asclinic-system LOCALLY

if [[ ${SHOULD_SETUP_ASCLINIC_SYSTEM_LOCALLY} = "true" ]]
then
	# Inform the user
	echo ""
	echo ""
	echo "NOW SETTING UP THE \"asclinic-system\" LOCALLY"

	# Make the "dfall" directory under the users root
	# > Note: the -p option means: no error if existing, make parent directories as needed
	echo ""
	echo ">> Now creating, if necessary, the directory: ${PATH_FOR_ASCLINIC_SYSTEM_LOCALLY}"
	mkdir -p ${PATH_FOR_ASCLINIC_SYSTEM_LOCALLY}

	# Change directory to this folder
	cd ${PATH_FOR_ASCLINIC_SYSTEM_LOCALLY}

	# Install git
	echo ""
	echo ">> Now installing \"git\" package:"
	echo ""
	sudo apt -y install git

	# Clone the asclinic-system git repository
	echo ""
	echo ">> Now cloning the \"asclinic-system\" repository"
	echo ""
	git clone https://gitlab.unimelb.edu.au/asclinic/asclinic-system.git

	# Add the ROS packge devel setup to the .bashrc
	# > Note: added together with a description in comments
	echo ""
	echo ">> Now adding the sourcing of the asclinic ROS package"
	echo "   environment variables to the ~/.bashrc file"
	echo "# SOURCE THE asclinic ROS PACKAGE DEVEL setup.bash FILE" >> ~/.bashrc
	echo "# (Note: this was added as part of the asclinic-system installation)" >> ~/.bashrc
	echo "source ${PATH_FOR_ASCLINIC_SYSTEM_LOCALLY}asclinic-system/catkin_ws/devel/setup.bash" >> ~/.bashrc
fi
# END OF: SETUP THE asclinic-system LOCALLY
# ============================================ #



# ============================================ #
# INSTALL CAMERA UTILITIES

if [[ ${SHOULD_INSTALL_VIDEO_FOR_LINUX_UTILITIES} = "true" ]]
then
	# Inform the user
	echo ""
	echo ""
	echo "NOW INSTALLING THE VIDEO FOR LINUX (V4L) UTILITIES"

	# Install the "v4l-utils" pacakge
	echo ""
	echo ">> Now installing \"v4l-utils\" package:"
	echo ""
	sudo apt -y install v4l-utils

	# Inform the user, and check that the installation worked
	echo ""
	echo ">> Nothing further to configure for v4l-utils."
	#echo ">> Now checking the installation was successful by running the command:"
	#echo ">> v4l2-ctl --list-devices"
	echo ""
	#v4l2-ctl --list-devices
fi
# END OF: INSTALL CAMERA UTILITIES
# ============================================ #





# ============================================ #
# INSTALL OPEN CV CONTRINUTED MODULES FOR PYTHON

if [[ ${SHOULD_INSTALL_OPENCV_CONTRIB_PYTHON} = "true" ]]
then
	# Inform the user
	echo ""
	echo ""
	echo "NOW INSTALLING THE OPEN CV CONTRIBUTED MODULES FOR PYTHON"

	# Install the "python3-pip" pacakge
	echo ""
	echo ">> Now installing \"python3-pip\" package:"
	echo ""
	sudo apt -y install python3-pip

	# Install the "scikit-build" pacakge
	echo ""
	echo ">> Now installing \"scikit-build\" package using pip3:"
	echo ""
	pip3 install scikit-build

	# Install the "Cython" pacakge
	echo ""
	echo ">> Now installing \"Cython\" package using pip3:"
	echo ""
	pip3 install Cython

	# Install the "numpy" pacakge
	echo ""
	echo ">> Now installing \"numpy\" package using pip3:"
	echo ""
	pip3 install numpy

	# Install the "opencv-contrib-python" pacakge
	echo ""
	echo ">> Now installing \"opencv-contrib-python\" package using pip3:"
	echo ""
	pip3 install opencv-contrib-python
fi
# END OF: INSTALL OPEN CV CONTRINUTED MODULES FOR PYTHON
# ============================================ #





# Command to check if a pacakge is already installed:
#if dpkg-query -s package-name 1>/dev/null 2>&1; then echo "temp"; fi