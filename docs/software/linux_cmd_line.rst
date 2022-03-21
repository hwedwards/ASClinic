.. _linux-cmd-line:

LINUX COMMAND LINE
==================

The list below provides basic commands for performing tasks from a Linux terminal, but it is by no means exhaustive.
You can find many great command line tutorials on the web, for example:

* `Ubuntu command line for beginners <https://tutorials.ubuntu.com/tutorial/command-line-for-beginners>`_
* `Linux commands cheat sheet <https://www.linuxtrainingacademy.com/linux-commands-cheat-sheet/>`_
* `linuxcommand.org <https://linuxcommand.org/index.php>`_
* `Software carpentry shell tutorial <https://swcarpentry.github.io/shell-novice/>`_


Navigating directories
**********************

* :code:`pwd` print working directory, i.e., the current directory

* :code:`ls` list files in the working directory.

* :code:`ls -l` list files in long format, i.e., with extra information.

* :code:`ls -a` list all files, i.e., including hidden files whose filenames being with :code:`.`

* :code:`ls -la` list all files in long format

* :code:`ls ~/Documents/` list files in the :code:`~/Documents/` directory

  * **Note** that :code:`~` is the home directory of the logged in user
  * For example, if logged in as :code:`myrobot`, then :code:`~` is :code:`/home/myrobot`

* :code:`man ls` display the the manual entry for the :code:`ls` command.

* :code:`cd ~/Documents` change directory to :code:`~/Documents`

* :code:`cd ..` change directory up one level


Editing filenames and directories
*********************************

* :code:`cp myscript.py myscript_new.py` make a copy of the file :code:`myscript.py` named :code:`myscript_new.py` in the current directory

* :code:`mkdir oldscripts` make a new directory named :code:`oldscripts`

* :code:`mv myscript.py oldscripts/myscript_old.py` move the file :code:`myscript.py` into the directory :code:`oldscripts` and at the same time rename it to :code:`myscript_old.py`

* :code:`cp hello.py oldscripts/hello_old.py` copy the file :code:`hello.py` into the directory :code:`oldscripts` and at the same time rename it to :code:`hello_old.py`

* :code:`cp -r oldscripts/ unused` copy the directory :code:`oldscripts` and all its contents recursively into a new directory named :code:`unused`, the directory :code:`unused` is created if it does not already exist

* :code:`rm hello.py` remove the file :code:`hello.py`

  * **Note** that :code:`rm` is permanent and cannot be undone, less the file is tracked using a git repository

* :code:`rm -r oldscripts` remove recursively the directory :code:`oldscripts` and all its contents

* :code:`rmdir oldscripts` remove :code:`oldscripts` only if it is an empty directory


Displaying files
****************

* :code:`cat .bashrc` prints the whole contents of the file named :code:`.bashrc` to the screen

* :code:`less .bashrc` an interactive way to view files, especially useful for long filenames

  * **Note** the manual entry, i.e., :code:`man less`, details the navigation shortcuts, some of the most useful are as follows.
  * :code:`d` and :code:`u` to scroll down and up by half a window respectively
  * :code:`f` and :code:`b` to scroll forwards and backwards by a one window respectively
  * :code:`g` and :code:`G` scroll to the beginning and end of the file respectively
  * :code:`/pattern` then press :code:`ENTER` to search forward in the file for occurrences of :code:`pattern`

    * :code:`n` and :code:`N` to repeat the search in the forward and backward direction respectively
    * All matches of :code:`pattern` are highlighted

  * :code:`q` to quit and return to the command line prompt


Editing files
*************

* :code:`nano myscript.py` edit the file :code:`myscript.py` using the :code:`nano` program

* :code:`vi myscript.py` edit the file :code:`myscript.py` using the :code:`vi` program

* **Note** the :code:`nano` editor is more similar to editing with a GUI based text editor and it displays the key commands on the screen. The :code:`vi` editor is more powerful but has a steeper learning curve for internalising its commands.


Executing files
***************

* :code:`python myscript.py` executes the python file :code:`myscript.py`

* :code:`chmod +x myscript.py` make the file :code:`myscript.py` executable

  * **Note** the command :code:`chmod` is used for modifying file permissions, see :code:`man chmod`
  * **Note** see `this tutorial on using chmod <https://linuxize.com/post/chmod-command-in-linux/>`_ for more details

* :code:`./hello.py` run the executable file named :code:`myscript.py`



Installing programs
*******************

* :code:`sudo apt install figlet` install the :code:`figlet` program

* :code:`figlet ASClinic` run the :code:`figlet` program with the argument :code:`ASClinc`, which prints out the following:

.. code-block::

      _    ____   ____ _ _       _
     / \  / ___| / ___| (_)____ (_) ___
    / _ \ \___ \| |   | | |  _ \| |/ __|
   / ___ \ ___) | |___| | | | | | | (__
  /_/   \_\____/ \____|_|_|_| |_|_|\___|


* :code:`sudo apt install` install the :code:`tree` program

* :code:`tree` run the :code:`tree` program to display the file structure of the working directory


Secure shell and secure copying
*******************************

* :code:`ssh username@ip_address` log in as :code:`username` to a terminal on the remote machine at :code:`ip_address`, .i.e., log in to a secure shell

* :code:`scp myscript.py username@ip_address:Documents` copy the file :code:`myscript.py` from your local machine to the remote machine at :code:`ip_address` into the :code:`Documents` sub-folder of :code:`username`'s home directory

  * **Note** see `this tutorial on using scp <https://linuxize.com/post/how-to-use-scp-command-to-securely-transfer-files/>`_ for more details


Miscellaneous
*************

* :code:`Ctrl-C` stop the currently active command, useful for when you get stuck
