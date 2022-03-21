.. _ros-what:

What is ROS?
============

The sensible starting point to answer to this is given on the `ROS wiki page <https://wiki.ros.org/ROS/Introduction>`_ where it states the following:

  *ROS is an open-source, meta-operating system for your robot. It provides the services you would expect from an operating system, including hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management. It also provides tools and libraries for obtaining, building, writing, and running code across multiple computers*

As you become familiar with ROS and develop expertise as using various of the ROS features, you will be able to give a variety of answers to this question.

As ROS requires an operating system to run, it is sometimes refered to as a middleware. Middleware is software that aims to facilitate easier interaction between applications, data sources accessed by the applications, users of the applications, and hardware the applications interact with. Many of these interaction would be possible using just the features offered by the operating system, but it would require significant software development. Middleware abstracts the complexity of these interactions and hence reduces development times required to implement a particular project where applications, data, users, and hardware interact.

In this context of middleware, and drawing on some key words from the ROS wiki, the following in a alterantive description of what ROS is:

  ROS is a middleware where the stand-alone C++ or Python pieces of software, which is written by you as the developer, are run as separate processes (called ROS nodes). For example, you can think of each piece of C++ or Python software as a separate app running independently on your smartphone. ROS provides the framework for sending messages between each of these processes that are running separately. This message-passing framework includes an API (Application Programming Interface) so that the sending of messages (called publishing) and the recieving of messages (called subscribing) is achieved with only a few lines of C++ or Python code. ROS also keep track of all processes that are running of any computer on the network so that separate processes can be run on the same or on different computers and there is no change needed to the API code for passing messages between those processes.


.. important::
  Each time you feel that you have gained new skills and mastery with ROS, take some time to write down another alternative answer to the question: what is ROS?






