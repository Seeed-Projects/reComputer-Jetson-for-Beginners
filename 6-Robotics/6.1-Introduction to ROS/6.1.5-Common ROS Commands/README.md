# Common ROS Commands

In a robot system, there may be a few to dozens of nodes running simultaneously. Each node has a unique name, and they communicate using topics, services, messages, parameters, and more. A common challenge arises: when you need to customize a node to communicate with another existing node, how do you retrieve the topic and the message format being used by the other node?

ROS provides a set of useful command-line tools to obtain various information about different nodes. The commonly used commands are as follows:

- **rosnode**: Manage nodes
- **rostopic**: Manage topics
- **rosservice**: Manage services
- **rosmsg**: Manage message types (msg)
- **rossrv**: Manage service message types (srv)
- **rosparam**: Manage parameters

These commands are dynamic, unlike the static file system commands. After the ROS program starts, these commands allow you to dynamically retrieve information about running nodes or parameters.

## rosnode
`rosnode` is used to retrieve information about ROS nodes.

- `rosnode ping`: Test the connectivity status to a node
- `rosnode list`: List all active nodes
- `rosnode info`: Print information about a node
- `rosnode machine`: List nodes running on a specific machine
- `rosnode kill`: Terminate a node
- `rosnode cleanup`: Clean up unreachable nodes

## rostopic
`rostopic` includes command-line tools to display debugging information about ROS topics, such as publishers, subscribers, publishing frequency, and ROS messages. It also includes an experimental Python library for dynamically retrieving information about topics and interacting with them.

- `rostopic bw`: Display bandwidth usage of a topic
- `rostopic delay`: Display delay of a topic with a header
- `rostopic echo`: Print messages to the screen
- `rostopic find`: Find topics by type
- `rostopic hz`: Display publishing frequency of a topic
- `rostopic info`: Display information about a topic
- `rostopic list`: List all active topics
- `rostopic pub`: Publish data to a topic
- `rostopic type`: Print the type of a topic

Examples:

- `rostopic list(-v)`: Print the names of topics currently running (with `-v` for detailed information like the number of publishers and subscribers).
- `rostopic pub /topic_name msg_type "msg_content"`: Publish a message to a topic.
- `rostopic echo /topic_name`: Retrieve and print the current message being published on a topic.
- `rostopic info /topic_name`: Get detailed information about a topic, including message type, publisher, and subscriber information.
- `rostopic hz /topic_name`: Display the publishing frequency of a topic.
- `rostopic bw /topic_name`: Display the bandwidth usage of a topic.

## 2.4.3 rosmsg
`rosmsg` is a command-line tool for displaying information about ROS message types.

- `rosmsg show`: Display the description of a message
- `rosmsg info`: Display detailed information about a message
- `rosmsg list`: List all message types
- `rosmsg md5`: Display the MD5 checksum of a message
- `rosmsg package`: List all messages in a package
- `rosmsg packages`: List all packages that contain messages

Examples:

- `rosmsg list`: List all message types in the current ROS environment.
- `rosmsg packages`: List all packages containing message types.
- `rosmsg package <package_name>`: List all messages in a specific package.
- `rosmsg show <msg_name>`: Display the description of a specific message.
- `rosmsg info <msg_name>`: Similar to `rosmsg show`, it provides information about a message type.
- `rosmsg md5 <msg_name>`: Generate the MD5 checksum of a message for data integrity checks.

## 2.4.4 rosservice
`rosservice` includes command-line tools to list and query ROS services.

- `rosservice args`: Print the arguments required by a service
- `rosservice call`: Call a service with the provided arguments
- `rosservice find`: Find services by type
- `rosservice info`: Print information about a service
- `rosservice list`: List all active services
- `rosservice type`: Print the type of a service
- `rosservice uri`: Print the ROSRPC URI of a service

Examples:

- `rosservice list`: List all active services.
- `rosservice args /service_name`: Print the arguments required by a specific service.
- `rosservice call /service_name "args"`: Call a service with the provided arguments.
- `rosservice find <service_type>`: Find services by their message type.
- `rosservice info /service_name`: Get detailed information about a service.
- `rosservice type /service_name`: Get the type of a service.
- `rosservice uri /service_name`: Get the URI of a service.

## 2.4.5 rossrv
`rossrv` is a command-line tool for displaying information about ROS service types. It is very similar to `rosmsg` in syntax.

- `rossrv show`: Display the description of a service message
- `rossrv info`: Display detailed information about a service message
- `rossrv list`: List all service message types
- `rossrv md5`: Display the MD5 checksum of a service message
- `rossrv package`: List all service messages in a package
- `rossrv packages`: List all packages that contain service messages

Examples:

- `rossrv list`: List all service message types in the current ROS environment.
- `rossrv packages`: List all packages containing service messages.
- `rossrv package <package_name>`: List all service messages in a specific package.
- `rossrv show <srv_name>`: Display the description of a specific service message.
- `rossrv info <srv_name>`: Similar to `rossrv show`, it provides information about a service message type.
- `rossrv md5 <srv_name>`: Generate the MD5 checksum of a service message for data integrity checks.

## 2.4.6 rosparam
`rosparam` includes command-line tools for getting and setting ROS parameters on the parameter server, using YAML-encoded files.

- `rosparam set`: Set a parameter
- `rosparam get`: Get a parameter
- `rosparam load`: Load parameters from an external file
- `rosparam dump`: Dump parameters to an external file
- `rosparam delete`: Delete a parameter
- `rosparam list`: List all parameters

Examples:

- `rosparam list`: List all parameters on the parameter server.
- `rosparam set <param_name> <value>`: Set a parameter with a specific value.
- `rosparam get <param_name>`: Get the value of a specific parameter.
- `rosparam delete <param_name>`: Delete a specific parameter.
- `rosparam load <file_name.yaml>`: Load parameters from a YAML file.
- `rosparam dump <file_name.yaml>`: Dump the current parameters to a YAML file.

These commands allow you to interact with different components of the ROS ecosystem dynamically, providing a powerful way to manage and monitor your ROS-based robot system.