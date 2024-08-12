# 6.1.7-Common Components and Features of ROS

## Distributed Communication in ROS

ROS (Robot Operating System) is designed as a distributed computing environment. This means that a running ROS system can consist of multiple nodes spread across multiple machines. Depending on the configuration, any node may need to communicate with any other node at any time.

To facilitate this, ROS has specific networking requirements:

1. There must be full bidirectional connectivity between all machines on all ports.
2. Each machine must announce itself using a name that can be resolved by all other machines.

### Implementation Steps

#### 1. Preparation

Before configuring ROS for distributed communication, ensure that the different computers are on the same network. Ideally, each computer should be assigned a static IP address. If you are using virtual machines, you need to change the network adapter setting to "Bridged Mode" to allow them to interact on the same network.

#### 2. Modify Configuration Files

On each computer, you need to modify the `/etc/hosts` file to include the IP addresses and hostnames of the other machines.

- **On the Host Machine:**

  Add the IP address and hostname of the slave machine.

  ```plaintext
  <Slave_IP_Address>    <Slave_Hostname>
  ```

- **On the Slave Machine:**

  Add the IP address and hostname of the host machine.

  ```plaintext
  <Host_IP_Address>    <Host_Hostname>
  ```

After updating the `/etc/hosts` files, use the `ping` command to test if the machines can communicate with each other:

- **Check IP Address:** Use `ifconfig` or `ip addr show`.
- **Check Hostname:** Use `hostname`.

#### 3. Configure the Host Machine's IP Address

On the host machine, you need to configure the IP address by adding the following lines to the `~/.bashrc` file:

```bash
export ROS_MASTER_URI=http://<Host_IP_Address>:11311
export ROS_HOSTNAME=<Host_IP_Address>
```

These lines set the `ROS_MASTER_URI` to the host machine's IP address, which tells ROS where the master node (`roscore`) is running. The `ROS_HOSTNAME` sets the hostname for the machine that ROS will use.

#### 4. Configure the Slave Machine's IP Address

On each slave machine (you can have multiple slaves), you also need to modify the `~/.bashrc` file by adding:

```bash
export ROS_MASTER_URI=http://<Host_IP_Address>:11311
export ROS_HOSTNAME=<Slave_IP_Address>
```

This configuration points each slave machine to the host's ROS master, enabling them to join the ROS network.

### Testing the Setup

#### 1. Start `roscore` on the Host Machine

On the host machine, start the ROS master node by running:

```bash
roscore
```

This step is crucial because the ROS master node manages the communication between different nodes in the ROS network.

#### 2. Test Communication from the Host to the Slave

- **On the Host Machine:** Start a subscriber node.
- **On the Slave Machine:** Start a publisher node.

Check if the nodes can communicate as expected.

#### 3. Test Communication from the Slave to the Host

- **On the Slave Machine:** Start a subscriber node.
- **On the Host Machine:** Start a publisher node.

Again, verify that communication between the nodes is functioning correctly.
