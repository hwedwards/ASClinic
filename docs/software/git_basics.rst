.. _git-basics:

Git Basics
==========

What is git?
************

Git is a version control software, and as such it is used to track modifications to source code of your project.
The tutorials linked below provide details about the inner workings and advantages of doing version control with git.

Links to git tutorials
**********************

You can find many great git tutorials on the web, for example:

* `Atlassian Bit Bucket tutorials for beginners <https://www.atlassian.com/git/tutorials/what-is-version-control>`_
* `Git Handbook on Github Guides <https://guides.github.com/introduction/git-handbook/>`_
* `Git Cheat Sheet by Github <https://training.github.com>`_
* `MUEEC Git Workshop recording on YouTube <https://youtu.be/vUgsyfbOUl4>`_


Where to host your git
**********************

Although you can host a git repository on your PC, it is typical to host it on a server with a browser-based interface.
Popular websites for hosting git repositories are:

* `Github <https://github.com>`_
* `BitBucket <https://bitbucket.org>`_
* `GitLab <https://about.gitlab.com>`_

The University of Melbourne also has instances of GitLab running on its servers so that you can rest assured that you data is local and safe:

* UoM-wide: `gitlab.unimelb.edu.au <http://gitlab.unimelb.edu.au/>`_
* FEIT: `gitlab.eng.unimelb.edu.au <http://gitlab.eng.unimelb.edu.au/>`_

..
  The remaining details on this page assume that you are using the UoM-wide GitLab instance.

Clone your git
**************

Once you have created a git repository on a server and initialized it, you can clone the repository to your local machine using

.. code-block::

  git clone <url>

where :code:`<url>` is replaced with the URL of you repository, for example:

.. code-block::

  git clone https://gitlab.unimelb.edu.au/asclinic/asclinic-system.git

If you have setup your account on the server to work with ssh keys from your PC (see `these GitLab instructions <https://gitlab.unimelb.edu.au/help/ssh/README>`_ for example)
then the :code:`<url>` uses the format :code:`git@`, for example:

.. code-block::

  git@gitlab.unimelb.edu.au:asclinic/asclinic-system.git


Typical git workflow
********************

* Add, edit, delete files on your PC using your favourite text editor and file manager (i.e., :code:`cp`, :code:`vi`, :code:`rm`)

* :code:`git status` to display the files you have changed.

* :code:`git diff` to display the content of the files you have changed, highlighting only the changes you made, i.e., the :code:`diff` erences.

* :code:`git pull` to fetch and integrate changes from the server and provide you with an initial indication of potential merge conflicts.

  * **Note:** :code:`git pull` does not delete any of the changes your made, it only integrates changes from the server if they cause no conflects.

* :code:`git add .` to add all changes you made to the staging index.

  * **Note:** to add individual files, use `git add <file_path_and_name>`

* :code:`git status` to display the files are staged and ready to be committed.

* :code:`git commit -m "<Write a descriptive commit message here>"` to commit the stage files locally.

  * **Note:** be sure to write a descriptive commit message as you do not know what future reason will cause you to be looking back through your commits.
  * **Note:** committed changes are only local to your PC.

* :code:`git status` to double check that the commit was successful.

* :code:`git push` to upload your commits to the repository on the server.


View previous commits
*********************

* :code:`git log` to display the version history of the current branch.

* :code:`git log --stat` to display also some statistics about the version history.

* :code:`git log --help` to display the manual entry for :code:`git log`

* :code:`git diff <commit1> <commit2>` to display the differences between :code:`<commit1>` and :code`<commit2>`, which are the hash keys displayed by :code:`git log`

* :code:`git diff --help` to display the manual entry for :code:`git diff`
