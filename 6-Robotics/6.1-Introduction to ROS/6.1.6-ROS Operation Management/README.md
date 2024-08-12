# 6.1.6-ROS Operation Management

## Managing ROS Nodes with Launch Files

Launch files in ROS are XML-formatted files used to start and manage multiple ROS nodes efficiently. This section covers the various tags available in launch files, including their attributes and use cases.

### The `<launch>` Tag

The `<launch>` tag is the root of every launch file and acts as a container for all other tags.

#### 1. Attributes
- **deprecated="deprecation statement"**  
  Indicates to the user that the current launch file has been deprecated.

#### 2. Child Tags
- All other tags in a launch file are child elements of the `<launch>` tag.

#### Example:
```xml
<launch>
    <!-- Other tags go here -->
</launch>
```

### The `<node>` Tag

The `<node>` tag is used to specify a ROS node to be launched. It's one of the most commonly used tags in a launch file. Note that the `roslaunch` command does not guarantee that nodes will start in the order they are declared, as the node startup process is multi-threaded.

#### 1. Attributes
- **pkg="package_name"**  
  Specifies the package to which the node belongs.

- **type="nodeType"**  
  The type of the node, which corresponds to the executable file name.

- **name="nodeName"**  
  The name of the node within the ROS network topology.

- **args="xxx xxx xxx"** (optional)  
  Passes arguments to the node.

- **machine="machine_name"**  
  Specifies the machine on which the node should be launched.

- **respawn="true | false"** (optional)  
  Determines whether the node should automatically restart if it exits.

- **respawn_delay="N"** (optional)  
  If `respawn` is set to true, this defines a delay of N seconds before the node is restarted.

- **required="true | false"** (optional)  
  Indicates whether this node is critical. If set to true, the entire `roslaunch` process will be terminated if the node exits.

- **ns="namespace"** (optional)  
  Launches the node within the specified namespace.

- **clear_params="true | false"** (optional)  
  Clears all parameters in the node's private namespace before the node starts.

- **output="log | screen"** (optional)  
  Determines where the log output should be sent: either to a log file or the screen. The default is `log`.

#### 2. Child Tags
- **env**: Used for setting environment variables.
- **remap**: Used for remapping topic or service names.
- **rosparam**: Used for setting parameters.
- **param**: Used for setting parameters.

#### Example:
```xml
<launch>
    <node name="node1" pkg="my_package" type="node_executable" output="screen" respawn="true" respawn_delay="5">
        <param name="param_name" value="param_value"/>
        <remap from="/old_topic" to="/new_topic"/>
    </node>
</launch>
```

### The `<include>` Tag

The `<include>` tag is used to include another XML-formatted launch file into the current launch file. This allows for modular and reusable configurations.

#### 1. Attributes
- **file="$(find package_name)/path/to/file.launch"**  
  Specifies the path to the launch file to be included.

- **ns="namespace"** (optional)  
  Includes the file under the specified namespace.

#### 2. Child Tags
- **env**: Used for setting environment variables.
- **arg**: Used to pass arguments to the included launch file.

#### Example:
```xml
<launch>
    <include file="$(find my_package)/launch/another_launch_file.launch" ns="my_namespace"/>
</launch>
```

### The `<remap>` Tag

The `<remap>` tag is used to remap ROS topic or service names. This is useful for avoiding name conflicts or standardizing names across different nodes.

#### 1. Attributes
- **from="xxx"**  
  The original topic or service name.

- **to="yyy"**  
  The new name for the topic or service.

#### 2. Child Tags
- None

#### Example:
```xml
<launch>
    <node name="node1" pkg="my_package" type="node_executable">
        <remap from="/old_topic" to="/new_topic"/>
    </node>
</launch>
```

### The `<param>` Tag

The `<param>` tag is used to set parameters on the ROS parameter server. The source of the parameter can be specified directly in the tag or loaded from an external file. When used inside a `<node>` tag, the parameter is set within the node's private namespace.

#### 1. Attributes
- **name="namespace/parameter_name"**  
  The name of the parameter, which can include a namespace.

- **value="xxx"** (optional)  
  Defines the value of the parameter. If omitted, an external file must be specified as the parameter source.

- **type="str | int | double | bool | yaml"** (optional)  
  Specifies the type of the parameter. If not specified, `roslaunch` will attempt to infer the type based on the value:
  - Numbers with a `.` are parsed as floating-point (double).
  - The strings "true" and "false" are parsed as boolean values (case-insensitive).
  - Everything else is parsed as a string.

#### 2. Child Tags
- None

#### Example:
```xml
<launch>
    <node name="node1" pkg="my_package" type="node_executable">
        <param name="namespace/param_name" value="param_value" type="str"/>
    </node>
</launch>
```

### The `<rosparam>` Tag

The `<rosparam>` tag allows parameters to be loaded from a YAML file, exported to a YAML file, or deleted. When used inside a `<node>` tag, the parameters are considered private.

#### 1. Attributes
- **command="load | dump | delete"** (optional, default is `load`)  
  Specifies the operation to perform: load parameters from a file, export them to a file, or delete them.

- **file="$(find package_name)/path/to/file.yaml"**  
  Specifies the YAML file to load or export parameters.

- **param="parameter_name"**  
  The name of the parameter.

- **ns="namespace"** (optional)  
  Specifies the namespace for the parameters.

#### 2. Child Tags
- None

#### Example:
```xml
<launch>
    <rosparam file="$(find my_package)/config/params.yaml" command="load" ns="my_namespace"/>
</launch>
```

## The `<group>` Tag

The `<group>` tag is used to group nodes and other tags, and it allows for applying a namespace or other settings to the group as a whole.

#### 1. Attributes
- **ns="namespace"** (optional)  
  Applies a namespace to all nodes and parameters within the group.

- **clear_params="true | false"** (optional)  
  Clears all parameters in the group's namespace before the group is launched. Use with caution as this can remove critical parameters.

#### 2. Child Tags
- Any tags except the `<launch>` tag can be children of `<group>`.

#### Example:
```xml
<launch>
    <group ns="my_namespace" clear_params="true">
        <node name="node1" pkg="my_package" type="node_executable"/>
        <node name="node2" pkg="my_package" type="node_executable"/>
    </group>
</launch>
```

### The `<arg>` Tag

The `<arg>` tag is used to define dynamic arguments that can be passed to the launch file at runtime, similar to function parameters. This increases the flexibility of launch files.

#### 1. Attributes
- **name="argument_name"**  
  The name of the argument.

- **default="default_value"** (optional)  
  Specifies the default value for the argument.

- **value="value"** (optional)  
  Specifies the value for the argument. Cannot be used simultaneously with `default`.

- **doc="description"**  
  Provides a description of the argument.

#### 2. Child Tags
- None

#### 3. Example
Launch file with argument syntax, `hello.launch`:

```xml
<launch>
    <arg name="robot_name" default="my_robot"/>
    <param name="robot_name" value="$(arg robot_name)"/>
</launch>
```

Command-line invocation with argument passing:

```bash
roslaunch hello.launch robot_name:=robot_value
```
## ROS Workspace Overlay

Imagine you have two custom workspaces, Workspace A and Workspace B, both containing a package named `turtlesim`. Additionally, the system's built-in workspace also has a package named `turtlesim`. When you invoke the `turtlesim` package, which one will be used?

### Implementation Steps

#### Step 0: Create Workspaces A and B
First, create two separate workspaces, A and B. Within each workspace, create a package named `turtlesim`.

#### Step 1: Modify the `~/.bashrc` File
Add the following lines to your `~/.bashrc` file to source the setup files for both workspaces:

```bash
source /home/user/path/to/workspaceA/devel/setup.bash
source /home/user/path/to/workspaceB/devel/setup.bash
```

Replace `/home/user/path/to/` with the actual paths to your workspaces.

#### Step 2: Load Environment Variables
Open a new terminal and run the following command to load the updated environment variables:

```bash
source ~/.bashrc
```

#### Step 3: Check ROS Environment Variables
To verify the ROS package paths, run:

```bash
echo $ROS_PACKAGE_PATH
```

**Result:** The output will show the paths in the following order: Workspace B → Workspace A → System Built-in Workspace.

#### Step 4: Invoke the `turtlesim` Package
Now, run the following command to navigate to the `turtlesim` package:

```bash
roscd turtlesim
```

**Result:** You will be directed to the `turtlesim` package within Workspace B.


## Handling ROS Node Name Conflicts

### Scenario
In ROS, each node has a name, which is defined during node initialization. In C++, this is done using the `ros::init(argc, argv, "node_name");` API, while in Python, it's done with `rospy.init_node("node_name")`. In a ROS network topology, nodes must have unique names because if multiple nodes share the same name, it can cause confusion during invocation. Specifically, if a node with a duplicate name is started, the existing node with that name will be shut down automatically. But what if you need to run multiple instances of the same node or deal with name conflicts?

ROS provides two strategies to handle such situations: **namespaces** and **name remapping**. 

- **Namespaces** add a prefix to node names.
- **Name remapping** assigns an alias to a node name.

Both strategies can resolve node name conflicts, and they can be implemented in several ways:

1. Using the `rosrun` command.
2. Through launch files.
3. In the node's code.

This section will demonstrate how to use these three methods to avoid node name conflicts.

### Example Scenario
Let's start two `turtlesim_node` nodes. If you open two terminals and start the nodes directly without any changes, the first node will be shut down when you start the second one. You'll see a warning message:

```plaintext
[ WARN] [1578812836.351049332]: Shutdown request received.
[ WARN] [1578812836.351207362]: Reason given for shutdown: [new node registered with same name]
```

Since nodes cannot share the same name, we'll explore several strategies to address this issue.

## Using `rosrun` for Namespaces and Remapping

### 1. Setting a Namespace with `rosrun`

You can set a namespace for a node using the following syntax:

```bash
rosrun package_name node_name __ns:=/new_namespace
```

#### Example:
```bash
rosrun turtlesim turtlesim_node __ns:=/xxx
rosrun turtlesim turtlesim_node __ns:=/yyy
```

With these commands, both nodes will run without issues.

#### Results:
Use `rosnode list` to check the nodes:

```plaintext
/xxx/turtlesim
/yyy/turtlesim
```

### 2. Remapping Node Names with `rosrun`

You can also remap a node's name, effectively giving it an alias, using the following syntax:

```bash
rosrun package_name node_name __name:=new_name
```

#### Example:
```bash
rosrun turtlesim turtlesim_node __name:=t1
rosrun turtlesim turtlesim_node __name:=t2
```

With these commands, both nodes will run with their new names.

#### Results:
Use `rosnode list` to check the nodes:

```plaintext
/t1
/t2
```

### 3. Combining Namespace and Name Remapping with `rosrun`

You can combine both techniques, setting a namespace and remapping the node name simultaneously:

```bash
rosrun package_name node_name __ns:=/new_namespace __name:=new_name
```

#### Example:
```bash
rosrun turtlesim turtlesim_node __ns:=/xxx __name:=tn
```

#### Results:
Use `rosnode list` to check the node:

```plaintext
/xxx/tn
```

Alternatively, you can set the namespace using an environment variable before starting the node:

```bash
export ROS_NAMESPACE=xxxx
```

## Using Launch Files for Namespaces and Remapping

In launch files, the `<node>` tag includes two important attributes: `name` and `ns`. These are used for name remapping and setting namespaces, respectively. Using a launch file to handle namespaces and name remapping is straightforward.

### 1. Launch File Example

Here's how you can set namespaces and name remapping in a launch file:

```xml
<launch>
    <node pkg="turtlesim" type="turtlesim_node" name="t1" />
    <node pkg="turtlesim" type="turtlesim_node" name="t2" />
    <node pkg="turtlesim" type="turtlesim_node" name="t1" ns="hello"/>
</launch>
```

In this example, the `name` attribute is mandatory, while `ns` is optional.

### 2. Running the Launch File

Run the launch file and then use `rosnode list` to see the results:

```plaintext
/t1
/t2
/hello/t1
```

## Setting Namespaces and Remapping in Code

If you're implementing custom nodes, you have more flexibility in setting namespaces and name remapping directly in your code.

### 1. C++ Implementation: Name Remapping

You can set a name alias using the following code:

```cpp
ros::init(argc, argv, "zhangsan", ros::init_options::AnonymousName);
```

#### Execution:
This will append a timestamp to the node's name, ensuring it's unique.

### 2. C++ Implementation: Setting a Namespace

You can set a namespace directly in the code like this:

```cpp
std::map<std::string, std::string> map;
map["__ns"] = "xxxx";
ros::init(map, "wangqiang");
```

#### Execution:
This sets a namespace for the node, allowing it to run without conflicts.

### 3. Python Implementation: Name Remapping

In Python, you can achieve similar functionality by using the following code:

```python
rospy.init_node("lisi", anonymous=True)
```
---
## Topic Name Remapping in ROS

In ROS, topic name remapping allows you to change the name of a topic that a node subscribes to or publishes to without modifying the node's code. This is particularly useful when integrating multiple nodes that need to communicate over different topic names. There are three primary methods to remap topic names in ROS:

1. Using the `rosrun` command.
2. Through launch files.
3. By directly modifying the code in C++ or Python.

### Using `rosrun` to Remap Topics

The syntax for remapping a topic name with `rosrun` is:

```bash
rosrun package_name node_name old_topic_name:=new_topic_name
```

### Example: Integrating `teleop_twist_keyboard` with `turtlesim`

There are two ways to set up communication between the `teleop_twist_keyboard` node and the `turtlesim` display node:

#### 1. Solution 1: Remap `teleop_twist_keyboard` Topic

In this approach, we remap the `teleop_twist_keyboard` node's topic to `/turtle1/cmd_vel`.

- **Start the keyboard control node:**

  ```bash
  rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/turtle1/cmd_vel
  ```

- **Start the turtlesim display node:**

  ```bash
  rosrun turtlesim turtlesim_node
  ```

Both nodes will communicate correctly using the `/turtle1/cmd_vel` topic.

#### 2. Solution 2: Remap `turtlesim` Topic

Alternatively, we can remap the `turtlesim` node's topic to `/cmd_vel`.

- **Start the keyboard control node:**

  ```bash
  rosrun teleop_twist_keyboard teleop_twist_keyboard.py
  ```

- **Start the turtlesim display node:**

  ```bash
  rosrun turtlesim turtlesim_node /turtle1/cmd_vel:=/cmd_vel
  ```

Both nodes will communicate correctly using the `/cmd_vel` topic.

### Using Launch Files to Remap Topics

You can also remap topics in a launch file. The syntax for remapping a topic in a launch file is:

```xml
<node pkg="package_name" type="node_type" name="node_name">
    <remap from="original_topic" to="new_topic" />
</node>
```

### Example: Integrating `teleop_twist_keyboard` with `turtlesim` Using Launch Files

Again, there are two solutions:

#### 1. Solution 1: Remap `teleop_twist_keyboard` Topic

In this approach, we remap the `teleop_twist_keyboard` node's topic to `/turtle1/cmd_vel`.

```xml
<launch>
    <node pkg="turtlesim" type="turtlesim_node" name="t1" />
    <node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" name="key">
        <remap from="/cmd_vel" to="/turtle1/cmd_vel" />
    </node>
</launch>
```

Both nodes will communicate correctly.

#### 2. Solution 2: Remap `turtlesim` Topic

In this approach, we remap the `turtlesim` node's topic to `/cmd_vel`.

```xml
<launch>
    <node pkg="turtlesim" type="turtlesim_node" name="t1">
        <remap from="/turtle1/cmd_vel" to="/cmd_vel" />
    </node>
    <node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" name="key" />
</launch>
```

Both nodes will communicate correctly.

### Remapping Topics in Code

The topic name in ROS is influenced by the node's namespace, the node's name, and the topic's own name. Topic names can generally be categorized into three types:

1. **Global:** The topic name is absolute and starts with a `/`, making it independent of the node's namespace.
2. **Relative:** The topic name is relative and does not start with `/`, meaning it is interpreted within the node's namespace.
3. **Private:** The topic name is private and starts with `~`, meaning it is resolved relative to the node's private namespace.

Let's explore these concepts through examples in C++ and Python.

### 1. C++ Implementation

#### Example Preparation:

1. **Initialize the node with a name:**

   ```cpp
   ros::init(argc, argv, "hello");
   ```

2. **Set different types of topic names.**
3. **Pass a `__ns:=xxx` argument when launching the node.**
4. **After the node starts, use `rostopic` to check the topic information.**

#### Global Topic Name

Global topic names start with a `/` and are independent of the node's name or namespace.

- **Example 1:**

  ```cpp
  ros::Publisher pub = nh.advertise<std_msgs::String>("/chatter", 1000);
  ```

  **Result:** `/chatter`

- **Example 2:**

  ```cpp
  ros::Publisher pub = nh.advertise<std_msgs::String>("/chatter/money", 1000);
  ```

  **Result:** `/chatter/money`

#### Relative Topic Name

Relative topic names do not start with a `/` and are resolved relative to the node's namespace.

- **Example 1:**

  ```cpp
  ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 1000);
  ```

  **Result:** `xxx/chatter`

- **Example 2:**

  ```cpp
  ros::Publisher pub = nh.advertise<std_msgs::String>("chatter/money", 1000);
  ```

  **Result:** `xxx/chatter/money`

#### Private Topic Name

Private topic names start with `~` and are resolved relative to the node's private namespace.

- **Example 1:**

  ```cpp
  ros::NodeHandle nh("~");
  ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 1000);
  ```

  **Result:** `/xxx/hello/chatter`

- **Example 2:**

  ```cpp
  ros::NodeHandle nh("~");
  ros::Publisher pub = nh.advertise<std_msgs::String>("chatter/money", 1000);
  ```

  **Result:** `/xxx/hello/chatter/money`

- **Special Case:** When using `~`, if the topic name starts with `/`, the topic name is treated as absolute.

  ```cpp
  ros::NodeHandle nh("~");
  ros::Publisher pub = nh.advertise<std_msgs::String>("/chatter/money", 1000);
  ```

  **Result:** `/chatter/money`

###  Python Implementation

#### Example Preparation:

1. **Initialize the node with a name:**

   ```python
   rospy.init_node("hello")
   ```

2. **Set different types of topic names.**
3. **Pass a `__ns:=xxx` argument when launching the node.**
4. **After the node starts, use `rostopic` to check the topic information.**

#### Global Topic Name

Global topic names start with a `/` and are independent of the node's name or namespace.

- **Example 1:**

  ```python
  pub = rospy.Publisher("/chatter", String, queue_size=1000)
  ```

  **Result:** `/chatter`

- **Example 2:**

  ```python
  pub = rospy.Publisher("/chatter/money", String, queue_size=1000)
  ```

  **Result:** `/chatter/money`

###  Relative Topic Name

Relative topic names do not start with a `/` and are resolved relative to the node's namespace.

- **Example 1:**

  ```python
  pub = rospy.Publisher("chatter", String, queue_size=1000)
  ```

  **Result:** `xxx/chatter`

- **Example 2:**

  ```python
  pub = rospy.Publisher("chatter/money", String, queue_size=1000)
  ```

  **Result:** `xxx/chatter/money`

#### Private Topic Name

Private topic names start with `~` and are resolved relative to the node's private namespace.

- **Example 1:**

  ```python
  pub = rospy.Publisher("~chatter", String, queue_size=1000)
  ```

  **Result:** `/xxx/hello/chatter`

- **Example 2:**

  ```python
  pub = rospy.Publisher("~chatter/money", String, queue_size=1000)
  ```

  **Result:** `/xxx/hello/chatter/money`

---
## Setting Parameters in ROS

In ROS, parameters are used to configure nodes at runtime. They can be set in various ways: using the `rosrun` command, within launch files, or directly in the code. Parameters can be global, relative, or private, depending on how they are defined.

### Setting Parameters with `rosrun`

You can set parameters when launching a node with the `rosrun` command. The syntax for setting parameters is:

```bash
rosrun package_name node_name _parameter_name:=parameter_value
```

#### Example: Setting a Parameter for the Turtlesim Node

Let's start the `turtlesim_node` and set a parameter `A = 100`.

```bash
rosrun turtlesim turtlesim_node _A:=100
```

#### Check the Parameters

You can use the following command to list all parameters and check the results:

```bash
rosparam list
```

**Output:**

```plaintext
/turtlesim/A
/turtlesim/background_b
/turtlesim/background_g
/turtlesim/background_r
```

**Explanation:** The parameter `A` is prefixed with the node name (`/turtlesim/`), indicating that when `rosrun` is used to set a parameter, it does so in the private namespace mode.

## Setting Parameters in Launch Files

As previously discussed, parameters can be set in launch files using either the `<param>` or `<rosparam>` tags. Parameters set outside the `<node>` tag are global, while those set within the `<node>` tag are private and relative to the node's namespace.

### Example: Setting Parameters with the `<param>` Tag

Here’s an example where we set a global parameter and a private parameter:

```xml
<launch>
    <param name="p1" value="100" />
    <node pkg="turtlesim" type="turtlesim_node" name="t1">
        <param name="p2" value="100" />
    </node>
</launch>
```

### Check the Parameters

After running the launch file, you can check the parameters with:

```bash
rosparam list
```

**Output:**

```plaintext
/p1
/t1/p2
```

**Explanation:** The parameter `p1` is global, while `p2` is private to the `t1` node, as indicated by the namespace.

## Setting Parameters in Code

Setting parameters in code provides greater flexibility, allowing you to define global, relative, and private parameters programmatically.

### 1. C++ Implementation

In C++, parameters can be set using the `ros::param` API or through a `ros::NodeHandle` object.

#### 1.1 Using `ros::param` to Set Parameters

The `ros::param::set` function is used to set parameters. The function’s first argument is the parameter name, and the second is the parameter value. If the parameter name starts with `/`, it's a global parameter. If it starts with `~`, it's a private parameter. Otherwise, it’s a relative parameter.

**Example:**

```cpp
ros::param::set("/set_A", 100); // Global, independent of namespace and node name
ros::param::set("set_B", 100);  // Relative, dependent on namespace
ros::param::set("~set_C", 100); // Private, dependent on namespace and node name
```

Assuming the namespace is `xxx` and the node name is `yyy`, checking the parameters with `rosparam list` would show:

```plaintext
/set_A
/xxx/set_B
/xxx/yyy/set_C
```

#### Using `ros::NodeHandle` to Set Parameters

To set parameters using `ros::NodeHandle`, first create a `NodeHandle` object, then call the `setParam` method. If the parameter name starts with `/`, it's global. If it doesn’t start with `/`, whether it’s relative or private depends on how the `NodeHandle` object was created.

**Example:**

```cpp
ros::NodeHandle nh;
nh.setParam("/nh_A", 100); // Global, independent of namespace and node name

nh.setParam("nh_B", 100);  // Relative, dependent on namespace

ros::NodeHandle nh_private("~");
nh_private.setParam("nh_C", 100); // Private, dependent on namespace and node name
```

Assuming the namespace is `xxx` and the node name is `yyy`, checking the parameters with `rosparam list` would show:

```plaintext
/nh_A
/xxx/nh_B
/xxx/yyy/nh_C
```

### 2. Python Implementation

In Python, setting parameters is slightly simpler than in C++. The `rospy.set_param` function is used to set parameters. The first argument is the parameter name, and the second is the parameter value. As with C++, if the parameter name starts with `/`, it’s global. If it starts with `~`, it’s private. Otherwise, it’s relative.

**Example:**

```python
rospy.set_param("/py_A", 100)  # Global, independent of namespace and node name
rospy.set_param("py_B", 100)   # Relative, dependent on namespace
rospy.set_param("~py_C", 100)  # Private, dependent on namespace and node name
```

Assuming the namespace is `xxx` and the node name is `yyy`, checking the parameters with `rosparam list` would show:

```plaintext
/py_A
/xxx/py_B
/xxx/yyy/py_C
```



