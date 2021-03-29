.. _workflow-git-merge-from-upstream-fork:

Git Merge from Upstream Fork
============================

If you created your :code:`asclinic-system` repository as a fork of:

https://gitlab.eng.unimelb.edu.au/asclinic/asclinic-system

then it will be beneficial to occasionally synchronise, i.e., merge, changes from that "original" repository into your fork.
To understand the descriptions below, the key terminology is that the "original" :code:`asclinic-system` repository that you forked is referred to as the "upstream" repository.

This `repository mirroring entry <https://docs.gitlab.com/ee/user/project/repository/forking_workflow.html#repository-mirroring>`_ in the GitLab documentation, the the links therein, provide a description for how to synchronise changes via the website.

Alternatively, you can synchronise changes by the following command line commands.


Add the upstream remote
***********************

1. Your repository needs to be configured to have the upstream repository as a so-called :code:`remote`. Add the upstream repository using the command:

.. code-block:: bash

	git remote add upstream https://gitlab.eng.unimelb.edu.au/asclinic/asclinic-system.github

2. Verify that step 1 worked by observing the output displayed by the command:

.. code-block:: bash

	git remote -v

**Note** that these two steps above only need to be performed once for the repository.


Merge in changes from the upstream remote
*****************************************

1. Fetch changes from the upstream repository using the command:

.. code-block:: bash

	git fetch upstream

2. Checkout the, i.e., change to, the master branch of your local repository:

.. code-block:: bash

	git checkout master

3. Merge the changes from the master branch of the upstream repository, i.e., from :code:`upstream/master`, into the master branch of your local repository:

.. code-block:: bash

	git merge upstream/master

If there are merge conflicts, these will need to be handled in the usual fashion.
