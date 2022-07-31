#!/bin/bash
#
# Call the motion service with sudo
# > The "sudoers rule" should allow this without needing a password
 sudo systemctl stop motion.service 2>&1
