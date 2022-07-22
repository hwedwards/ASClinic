.. _ros-key-elements:

The key elements of ROS
=======================

This pages is aim at providing intuitive description of the key elements of ROS. Ideally, the descriptions here would be enough for you to draw a system architecture diagram for how these elements will be used together to achieve the desired functionality of your robotics project. In other words, the descriptions are ideally enough for you to describe how you would use ROS even if you have zero knowledge of the programming required to implement your specified usage of ROS.

For each of the key elements below, the following is provided:

  * Quotes taken directly from the ROS Wiki and ROS Tutorial pages.
  * Links to the relevant `ROS Wiki <https://wiki.ros.org>`_ and `ROS Tutorial <https://wiki.ros.org/ROS/Tutorials>`_ pages.
  * An intuitive explanation of the key elements.

.. contents:: Contents of this page
   :local:
   :backlinks: none
   :depth: 2


.. _ros-key-elements-nodes:

Nodes
*****

* "A process that performs computation"

* See also the `ROS Wiki Nodes page <https://wiki.ros.org/Nodes>`_ and the `ROS Tutorials Nodes page <https://wiki.ros.org/ROS/Tutorials/UnderstandingNodes>`_.

* A single node can be thought of as a single program on your computer, or a single app on your smartphone, or a single C++ code that you compile, or a single Python script that you run. Each one of these examples run stand alone and does its thing. For example, the weather app on your smartphone retrieves the weather data from the internet and displays it in a meaningful fashion, call it the "weather node" if you will.


.. _ros-key-elements-topics:

Topics
******

* "Topics are named buses over which nodes exchange messages."
* "Topics have anonymous publish/subscribe semantics."
* "Many-to-many one-way transport" of data."
* "Decouples the production of information from its consumption."

* See also the `ROS Wiki Topics page <https://wiki.ros.org/Topics>`_ and the `ROS Tutorials Topics page <https://wiki.ros.org/ROS/Tutorials/UnderstandingTopics>`_.

* A single topic is can be thought of as a single option for email / notification preferences for any website account you have. For example, if you have a linkedin account, then you can choose whether or not to receive emails / notifications for: conversations, enterprise products, jobs, network events, news, profile info, etc. Note that each of these options for emails / notifications has a name that indicates what it is about, in the same way each topic in ROS has a name and you should choose these name to indicated what the topic is about. Each of these can be thought of as a separate topics with a publish/subscribe semantics:

  * The person (or team, or bot) sending out (i.e., publishing) linkedin "*daily news summary*" doesn't know who will be receiving the "*daily news summary*" based on their account preferences (i.e., doesn't need to know who is subscribed to the "*daily news summary*").
  * After you turn on your account preference to receive the linkedin "*daily news summary*", then you receive all subsequent news emails / notifications until you turn off the preference. The person (or team) who sent out the "*daily news summary*" doesn't know when you will read the news or what you will do with it.
  * In other words, the person (or team) producing and sending out the linkedin "*daily news summary*" is decoupled from you as the consumer of that "*daily news summary*".

* To connect the linkedin example to nodes. The whole linkedin company can be thought of as a node (i.e., the server-side), and they offer the existence on the topic named "*daily news summary*". The linkedin app on your smartphone is a separate node (i.e., the client-side), and that is where you receive a notification when the is news. Hence the "*daily news summary*" is the name of a particular channel that allows **exchanges of a particular type** from the linkedin team to you (i.e., from the server-side to the client-side).


.. _ros-key-elements-messages:

Messages
********

* "A message is a simple data structure, comprising typed fields"

* See also the `ROS Wiki Messages page <https://wiki.ros.org/Messages>`_.

* A single message is a single "thing" that is sent along a particular topic, and the contents of the message need to meet a specified convention.

  * Continuing the linkedin example from above, a single message would be the contents of the news email that is sent out. Moreover, the contents of the news email will adhere to as particular format, for example: news heading, followed by an image, followed by text describing that news item, followed by the next news heading, and so on.
  * More specifically for ROS, the convention for a message is exactly what is be called a :code:`struct` in C++.


.. _ros-key-elements-pub-and-sub:

Publishing and Subscribing
**************************

* The act of sending and receiving a messages on a topic.


.. _ros-key-elements-advert-vs-pub:

Advertising versus publishing
*****************************

* To advertise a topic is to simply state the existence of the topic and give it a name.

  * For example, by linkedin having the "*daily news summary*" topic available when you go to your account preferences, they are hence advertising the existence of the "*daily news summary*" topic. But it is possible (though unlikely) that they never send a message on this topic.

* To publish is to send a message on the topic. It is not possible to send a message on a topic that have not been advertised.


.. _ros-key-elements-services:

Services
********

* A request / reply interaction
* "Defined by a pair of messages: one for the request and one for the reply."

* See also the `ROS Wiki Services page <https://wiki.ros.org/Services>`_ and the `ROS Tutorials Services page <https://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams>`_.

* A service can be thought of as any two-way communication, as opposed to the one-way communication of topics (i.e., publishing and subscribing).

* A service, as per its name, is a service that a particular node offers. Think of anything that might come under the umbrella term of customer service, which means any interaction where the customers requests something and the service-provider responds with the something that was requested. For example:
  * A hairdresser offers various services, each with its own name, such as: "hair cut", "hair colour", "hair extension", "perm", "straighten", etc.
  * These services are only available when the hairdresser is open, i.e., they are only available after the hairdresser open (or launches in ROS terminology) in the morning and continued to be available until the hairdresser close (or shuts down in ROS terminology) in the evening.
  * You go to the hairdresser during opening hours as a customer and request a particular service by its name, e.g., you ask for a "hair colour".
  * The hairdresser provides you with the "hair colour" service there and then.
  * During the execution of the "hair cut" service, both you and the hairdresser are occupied and cannot perform other tasks.

* The request and reply of a service is ROS also used messages to describe the details of the each. For the example of the "hair colour" service:

  * You are the customer node use a message to specify the colour you want, e.g., green.
  * The service is then carried out and hence you and the hairdresser are occupied during the execution of the hair colour service.
  * Once the hairdresser has finished colouring your hair, then the hairdresser responds to you with a message specifying for example: whether the hair colouring was successful or not, the price you need to pay, the date by which you need to pay, and a list of possible payment methods.


.. _ros-key-elements-namespaces:

Namespaces
**********

* "A hierarchical naming structure that is used for all resources in a ROS Computation Graph, such as Nodes, Parameters, Topics, and Services."
* "These names are very powerful in ROS and central to how larger and more complicated systems are composed in ROS."

* See also the `ROS Wiki Concepts page <https://wiki.ros.org/ROS/Concepts#Names.Names>`_.

* Namespaces can be thought of a simply unique strings of text that group things together. For the example above of a "*daily news summary*" topic offered by the linkedin node, how do we distinguish that from the "*daily news summary*" topic offered by a node that is the whole BBC company?

  * The answer is that purely from the topic name we cannot distinguish.
  * Hence we need to group the topic names into different spaces, i.e., we need to group them into namespaces.
  * In this way, namespaces create a hierarchical structure for things that are named, i.e., for topics, for services, for parameters, for nodes.
  * The way the hierarchical structure is represented in ROS is exactly like a folder structure.
  * Hence we would put the "*daily news summary*" topic from linkedin into a namespace that we call "*linkedin*", and we would the "*daily news summary*" topic from BBC into a namespace that we call "*BBC*".
  * Hence the full path to each of these "*daily news summary*" would be:

    * :code:`/linkedin/daily news summary`
    * :code:`/BBC/daily news summary`
    * Where spaces are generally frowned upon when naming things in ROS, and the :code:`/` character is exactly what is used by ROS to separate namespaces from names.

  * As with folders that you use on your personal computer to arrange your data, namespaces can be nested to whatever level you need to distinguish between things. And as with folders on your personal computer, things can quickly get out-of-hand:

    * :code:`/study/eng_masters/year1/sem1/asclinic/self_study/ros/my_note.txt`

  .. hint::
    Keep your namespace nesting as simple as possible, and only add layers to the hierarchy when they become necessary.

  * A natural use of namespaces is multi-agent robotics, i.e., where multiple robots are all connected to the same network and are all running ROS. In this setting, robots of the same type will most like be running the same set of nodes, i.e., the sensor, controller, and actuator nodes. Hence, it is natural to put the nodes for one robot into a namespace that is a unique identifier for that robot. For example:

    * :code:`/robot01/motion_and_motor_controller`
    * :code:`/robot02/motion_and_motor_controller`
  
    where :code:`motion_and_motor_controller` is the name of the ROS node that is running for both robots, and :code:`/robot01` & :code:`/robot02` are the namespaces for the respective robots.


.. _ros-key-elements-parameters:

Parameters and the Parameter Server
***********************************

* "The parameter server is a shared, multi-variate dictionary".
* "The parameter server allows data to be stored by key in a central location."
* "It is currently part of the Master."
* "Nodes use this server to store and retrieve parameters at runtime"
* "As the parameter server is not designed for high-performance, it is best used for static, non-binary data such as configuration parameters."

* See also the `ROS Wiki Parameter Server page <https://wiki.ros.org/Parameter%20Server>`_ and the `ROS Tutorials Parameters page <https://wiki.ros.org/roscpp_tutorials/Tutorials/Parameters>`_.

* Parameters can be thought of as defining the specific numbers of our engineering design, i.e., the numbers appearing on the data-sheet/documentation that you **should** produce for your robotics project. Some examples of specification that are appropriate as parameters are:

  * Wheel diameter
  * Wheel base
  * Location of robot's centre of gravity relative to the robot's centre of rotation.
  * Location of the camera relative to the robot's centre of rotation.
  * Proportional, integral, and derivative gain for any such controllers.
  * Limits for the motor controllers.
  * Many, many more.

* Parameters are those numbers that are hazardous to hard-code into the code you write for your nodes because:

  * It make it difficult to get an overview for the specifications that are actually running.
  * It becomes very tedious to changes specifications.

* Two main benefits of using parameters are:

  * You can gather together many parameters in one place, thus providing a good overview and quick changes.
  * You can change the parameter values without needing to re-compile.
  * It is possible to add code for changing the parameter values while a node is running, which can be quite time-efficient for online tuning of parameter values. 


.. _ros-key-elements-ros-master:

ROS Master
**********

* "The ROS Master provides name registration and lookup to the rest of the Computation Graph."
* "Without the Master, nodes would not be able to find each other, exchange messages, or invoke services."

* See also the `ROS Wiki Master page <https://wiki.ros.org/Master>`_.

* The ROS master can be thought of as a single unique entity that keep track of everything. Nothing in ROS can run with a ROS master, and there can only be one ROS master for any collection of nodes that need to communicate with each other.

* The ROS master is like a *gate keeper* than keeps a *master list* of everything currently in existence in ROS, i.e., things coming into existence are added to the *master list*, and things going out of existence are removed from the *master list*. For example:

  * A node can only start (i.e., launch or run in ROS terminology) by first finding the *gate keeper* (at a particular IP address), at which time the *gate keeper* writes down on the *master list* the node's name and its namespace.

    * Additionally for nodes, if the *gate keeper* has an already existing node with the same name and namespace on the *master list*, then that existing node is force to shutdown and it is kicked out.

  * When a node shut downs, then it must go past and inform the *gate keeper* on its way to going out of existence. Hence the *gate keeper* removes that node from the *master list*.

  * Services and parameters also need to have a can only have one in existence at any one time on the *master list* with the same name and namespace.

  * By contrast, a topic with the same name and namespace can be advertise or subscribed to by multiple nodes, and the ROS master as the *gate keeper* just keeps one entry for this topic on the *master list*, and under that entry it keeps a sub-list of all the nodes that subscribe to the topic and all the nodes that advertise to the topic. Once there are no more subscribers or advertisers, them the topic is removed from the *master list*.

    * Another part of an advertiser or subscriber registering with the ROS master is that the communication layer is configured so that published messages are transferred from the advertiser to the subscriber(s) via a UDP or TCP/IP connection.


.. _ros-key-elements-all-together:

Put it all together
*******************


.. graphviz::
  :layout: dot
  :name: ex_ros_architecture
  :caption: Example ROS Architecture (figure layout generated by graphviz)
  :alt: Example ROS Architecture
  :align: center

  digraph "sphinx-ext-graphviz"
  {
    #size="6,4";
    splines=true;
    rankdir="TB";
    #rank="same";
    graph [fontname="Helvetica Neue", fontsize="12"];
    node [fontname="Helvetica Neue", fontsize="12"];
    edge [fontname="Helvetica Neue", fontsize="9"]; 

    # Define the nodes
    wheels [
      shape=plain,
      label = <
        <TABLE BORDER="0" CELLBORDER="1" CELLSPACING="0">
        <TR><TD PORT="title" BGCOLOR="darkolivegreen1" COLSPAN="2">Physical System</TD></TR>
        <TR>
          <TD PORT="motors" BGCOLOR="darkolivegreen1">motors</TD>
          <TD PORT="encoders" BGCOLOR="darkolivegreen1">encoders</TD>
        </TR>
        </TABLE>
      >,
    ];

    i2c [
      shape=plain,
      label = <
        <TABLE BORDER="0" CELLBORDER="1" CELLSPACING="0">
        <TR><TD PORT="title" BGCOLOR="deepskyblue">i2c</TD></TR>
        </TABLE>
      >,
    ];

    controller [
      shape=plain,
      label = <
        <TABLE BORDER="0" CELLBORDER="1" CELLSPACING="0">
        <TR><TD PORT="title" BGCOLOR="deepskyblue">motion_and_motor_controller</TD></TR>
        <TR><TD PORT="duty_cycle" BGCOLOR="coral">duty_cycle<BR/>{left,right}</TD></TR>
        </TABLE>
      >,
    ];

    encoder [
      shape=plain,
      label = <
        <TABLE BORDER="0" CELLBORDER="1" CELLSPACING="0">
        <TR><TD PORT="title" BGCOLOR="deepskyblue">encoder_counter</TD></TR>
        <TR><TD PORT="counts" BGCOLOR="coral">counts<BR/>{left,right}</TD></TR>
        </TABLE>
      >,
    ];

    odometry [
      shape=plain,
      label = <
        <TABLE BORDER="0" CELLBORDER="1" CELLSPACING="0">
        <TR><TD PORT="title" BGCOLOR="deepskyblue">wheel_odometry</TD></TR>
        <TR><TD PORT="pose" BGCOLOR="coral">pose<BR/>{x,y,heading}</TD></TR>
        </TABLE>
      >,
    ];

    

    
    
    # Define the connections
    i2c:title -> wheels:motors [
      label="I2C bus"
      style=dashed,
      shape=vee,
      constraint=true,
    ];

    controller:duty_cycle -> i2c:title [
      style=solid,
      shape=vee,
      constraint=true,
    ];

    encoder:counts -> odometry:title [
      style=solid,
      shape=vee,
      constraint=true,
    ];

    odometry:pose -> controller:title [
      style=solid,
      shape=vee,
      constraint=true,
    ];

    wheels:encoders -> encoder:title [
      label="GPIO pins"
      style=dashed,
      shape=vee,
      constraint=true,
    ];

    controller:duty_cycle -> encoder:title [
      style=solid,
      shape=vee,
      constraint=false,
    ];
  }


What does it all mean for you?
******************************

.. important::
  Each time you feel that you have gained new skills and mastery with ROS, take some time to write down your most concise description to each of the key elements of ROS:

    * :ref:`ros-key-elements-nodes`
    * :ref:`ros-key-elements-topics`
    * :ref:`ros-key-elements-messages`
    * :ref:`ros-key-elements-pub-and-sub`
    * :ref:`ros-key-elements-advert-vs-pub`
    * :ref:`ros-key-elements-services`
    * :ref:`ros-key-elements-namespaces`
    * :ref:`ros-key-elements-parameters`
    * :ref:`ros-key-elements-ros-master`