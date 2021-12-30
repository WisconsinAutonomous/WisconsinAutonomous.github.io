---
title: ROS Overview
author: Aaron Young
date: 2021-12-16 16:53:00 -0600
categories: [ROS]
tags: [ros, technical]
---

This guide outlines how Wisconsin Autonomous structures their ROS respositories. 

## Purpose

A major difference between Wisconsin Autonomous and industry is that in WA, we have much higher turnover. This is because students graduate or move on. In addition, industry has employees working 40hrs a week and WA has students working 5-10hrs a week. As a result, planning is very important to ensure the time we do spend working on WA is not wasted.

This guide goes into detail about how Wisconsin Autonomous structures their control stacks. We may have multiple control stacks for different applications; for example, one for the evGrand Prix, one for AutoDrive, and one for the rc car.

It is very important the ideas presented on this page are followed. 

## TL;DR

### Required Packages

- [Docker](https://www.docker.com/)
- [Docker Compose](https://docs.docker.com/compose/)
- [Python3](https://www.python.org/)
- [Git](https://git-scm.com/)

### 1. Clone the Repository

```bash
git clone --recursive https://github.com/WisconsinAutonomous/REPO.git && cd REPO 
```

### 2. Install Host Python Dependencies

```bash
pip install -r host-requirements.txt
```

```{note}
A python environment is recommended (Anaconda, venv, etc.)
```

### 3. Create the Docker Network

```bash
wa docker network
```

### 4. Use the Docker Development Environment

**Building:**

```bash
docker-compose build wagrandprix-dev
```

**Starting:**

```bash
docker-compose up -d wagrandprix-dev
```

**Entering:**

```bash
docker-compose exec wagrandprix-dev bash
```

**Tearing Down:**

```bash
docker-compose down wagrandprix-dev
```

### 5. Test the Stack with wa_simulator

```bash
wa docker run --wasim --data wasim/data wasim/baseline_track.py
```

### 6. Visualize GUI Applications

```bash
wa docker novnc
```

Navigate to [https://localhost:8080/vnc\_auto.html](https://localhost:8080/vnc_auto.html).

## Prerequisites

We'll refer to the control stack repository as `REPO` from now on.

### ROS

The [Robot Operating System (ROS)](https://www.ros.org/) is a major tool in the area of robotics. We use [ROS 2](https://docs.ros.org/en/foxy/) and for the remainder of this document, it is assumed you have a solid background with ROS related topics. This includes, but is not limited to, topics, nodes, messages, publishers, and subscribers. If any of this doesn't make sense, plesae refer to the [ROS tutorials](https://docs.ros.org/en/foxy/Tutorials.html).

We have also created an easy to setup ROS environment using [Docker](#docker) called [wa_ros_tutorial](https://github.com/WisconsinAutonomous/wa_ros_tutorial). There are instructions there on how to set it up.

### Docker

Additionally, [Docker](https://www.docker.com/) is used to mitigate setup headaches. Please research Docker to understand what it is exactly. Detailed instructions will be provided on how to use it.

## Repository Structure

Each control stack used by Wisocnsin Autonomous should be structured as follows.

```
REPO
├── docker-compose.yml
├── docker/             # Dockerfiles
├── docs/               # Documentation/tutorials specific to this repo
├── misc/               # Miscellaneous scripts and resources
├── wasim/          		# wa_simulator scenarios that can be used to test the control stack
├── workspace/          # ROS 2 workspace
├── _templates/         # Templates for generating ROS nodes
└── README.md
```

A template has been made with the aforementioned structure to streamline the creation process for this type of repo. The template can be found [here](https://github.com/WisconsinAutonomous/wa-control-stack).

### `docker-compose.yml`

[Docker Compose](https://docs.docker.com/compose/) is a tool for running multi-container Docker applications. For the most part, it is used in this way; however, it comes with an entrypoint called `docker-compose` that simplifies building/running/executing on containers that is cross platform.

The `docker-compose.yml` file defines configuration variables for the `docker` containers within it. 

All `docker-compose.yml` files will have two `services`: `REPO-dev` and `REPO-prod`. `REPO-dev` is meant for local development. `REPO-prod` is meant for production and will most likely be run on the actual vehicle.

### `docker/`

This folder holds the Dockerfiles that are used to generate the images that we'll run the control stack in. For the most part, these don't need to be changed. 

### `docs/`

This folder holds general documentation or tutorials specific to this repository. Anything that may be similar across each repos should just go in this guide.

### `misc/`

This should house miscellaneous resources and/or scripts. This should again be specific to the repository. Anything common across repos should go in a shared repository.

### `wasim/`

This folder contains `wa_simulator` scripts that implement various simulation scenarios. This folder, in conjunction with the `wa_cli`, is simply a way of providing simulation files without the need to create them each time or clone `wa_simulator` locally.

### `workspace/`

This is where we put the ROS 2 workspace. This is what is specific to each repository. Most of the development will take place here, and we'll go into more detail in future sections of this guide.

### `_templates/`

This folder holds the metadata in order for [Hygen](https://www.hygen.io) to generate templated ROS nodes. This will go into more detail in future sections.

## Setup and Installation

**DISCLAIMER**: For developing the ROS 2 workspace, you _must_ use the created docker images. This is a requirement because we don't want to have to support multiple operating systems and have to deal with individual people's systems. The only place docker will not be used with this repository is on the actual vehicle (and with our own hardware).

Docker is a powerful application that is out of the scope of this README. Beyond Wisconsin Autonomous, knowing what and how to use Docker is a very valuable skill. To learn more, visit [their website](https://www.docker.com/). You can also find a huge amount of tutorials and resources just through Google. Please spend some time understanding Docker before continuing. For the remainder of this tutorial, it is assumed you understand `docker` and `docker-compose`.

For all of the commands we'll mention, they must be run from within the target repository. 

### Clone the Repository

You should really already know how to do this. An example command to clone the `REPO` repository locally is provided below:

```bash
git clone --recursive https://github.com/WisconsinAutonomous/REPO.git && cd REPO 
```

```{note}
`--recursive` is recommended so that all [git submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules) are automatically pulled along with the main repository. If this isn't done during this step, you may do this after the fact with `git submodule update --init`.
```

### Installing Required Packages

First, you need to install any necessary python packages for `REPO`. There should be a file called `host-requirements.txt`, which contains any python packages needed to run the control stack. At the very least, this file will contain `wa_cli`, which is the command line interface tool created for the Wisconsin Autonomous team.

To install these packages, run the following command:

```bash
pip install -r host-requirements.txt
```

```{note}
It is recommended to use a python environment of some sort (i.e. Anaconda or venv). Activate the environment prior to running the previous command.
```

### Installing Docker

To get started, you will need to install [Docker](https://docs.docker.com/desktop/) and [Docker Compose](https://docs.docker.com/compose/install/). Please refer to the previous links for instructions on how to install.

### Docker Setup

In order to support multiple containers, we'll need to create a [Docker network](https://docs.docker.com/network/). This will allow us to have a container for simulations and then a separate container running our control stack. The scalability of this structure is very important for long term development across multiple platforms.

The `wa_cli` package has a command that will create the network for you. Make sure you have installed `wa_cli` and you may run the following command:

```bash
wa docker network
```

If you'd rather use the docker api, to create the network, you can run the following command:

```shell
docker network create \
	--driver=bridge \
	--subnet=172.20.0.0/25 \
	wa
```

This command created a network titled `wa`.

### Starting the Control Stack

For development, we'll focus on the `REPO-dev` service.

The very first time you start up the control stack, it will need to build the image that the container users. This may take a while, but should only need to be done the first time around.

To start the container (and build it if it isn't built yet), run the following command.

```shell
 docker-compose up -d REPO-dev
```

If it completes successfully, it should say something similar to the following:

```shell
[+] Running 1/1
 ⠿ Container REPO-dev  Started
```

The container is now running in the background.

### Entering the Container

After starting the container, to actually use it and run ROS commands, you'll need to attach to it in your shell. To do this, run the following command.

```shell
docker-compose exec REPO-dev bash
```

Within the container, you should enter the shell in the `/root/` directory. Within `/root/`, the only folder you should see is `REPO` (reminder that `REPO` should be replaced with whatever repo you're actually using). This is a docker [volume](https://docs.docker.com/storage/volumes/), so any changes you make in this folder will be reflected on your system. I will say that again: the only file changes that will persist between your host system and the docker container are the files in `/root/REPO`. If you make any changes from within your docker container, ensure it is done in there. 

For development purposes, you can use whatever tools you'd like for editing the code (`Atom`, `VSCode`, etc.). Because `/root/REPO` in the container is a volume (see previous paragraph), changes from your system will also be copied to your container.

Various tools are installed to aid development, such as `tmux`. Feel free to leverage these. The `docker-compose exec ...` command can also be run from any terminal window (as long as it's run from within this repository); this is the advantage of running `docker-compose up` in the background (detached).

For those interested, you may use other shells. `zsh` and `bash` should both alread  be installed, but you may need to install other shells, if desired.

### Stopping the Container

When you are finished and would like to free up resources on your computer, you may shutdown the container with the following command.

```shell
docker-compose down
```

### Building the ROS Workspace

By default, the ROS distribution should be sourced automatically thanks to the shell configuration file. Further, the workspace setup overlay should also be sourced.

As a reminder, the ROS workspace is located in `/root/REPO/workspace/`. To get started with ROS in the container, you will need to build the workspace. An example of how to do this is provided below, but how to use ROS beyond this example is outside the scope of this guide.

```bash
cd /root/REPO/workspace/ && colcon build
```

## ROS Workspace

The structure of the ROS workspace within `REPO` may be slightly different given the application, but it should generally look like the following.

```
└── src
    ├── common/
    │   └── common_interfaces/
    ├── perception/
    │   ├── perception_py/
    │   └── perception_cpp/
    ├── state_estimation/
    ├── controls/
    ├── drivers/
    └── external
```

### Creating a Package

Packages can be created using the `ros2 pkg create` command. Use `ros2 pkg create --help` to get more information.

For example, to create a new python package, run the following: `ros2 pkg create --built-type ament_python new_package` from the `/root/REPO/workspace/src` directory. This will create a new package at the directory level.

### Creating a Node

<details>
<summary>Click Me</summary>
<p markdown="block">

Most of the time you won't be creating new packages, but just adding nodes to existing packages. We are using a templating tool called [Hygen](https://www.hygen.io/) to help with boiler plate code. Hygen is already installed and configured in the development docker container, so that is the recommended avenue to use it. Otherwise, you are responsible for installing and setting it up (you need to set the `HYGEN_TMPLS` environment variable to the `_templates` directory).

This is a **4** step process

**1.**
To add a node to the `new_package` package, you can run the following:

```
cd src/new_package/new_package/
hygen ros2 python-node
```

You will be prompted to enter some basic information about the node, and a file for a new node will be written in the current directory. You should enter this information exactly as is documented the [WA Software Architecture Diagram](https://drive.google.com/file/d/1nBj6e1DiyWXzSxHhxgPGXwtTzqKDPyTg/view?usp=sharing)

```
✔ What is the name of the node ? · new_node
✔ What topics should be subscribed to (comma seperated)? · sub_topic1, sub_topic2, sub_topic3
✔ What topics should be published (comma seperated)? · pub_topic2, pub_topic2

Loaded templates: /root/REPO/_templates
       added: ./new_node.py
```

Resulting in:

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class NewNode(Node):

    def __init__(self):
        super().__init__('new_node')


        # Create publisher handles
        self.publisher_handles = {}

        self.publisher_handles["pub_topic1"] = self.create_publisher(String, "pub_topic1", 10)

        self.publisher_handles["pub_topic2"] = self.create_publisher(String, "pub_topic2", 10)


        # Create subscriber handles
        self.subscriber_handles = {}

        self.subscriber_handles["sub_topic1"] = self.create_subscription(String, "sub_topic1", self.sub_topic1_callback, 10)

        self.subscriber_handles["sub_topic2"] = self.create_subscription(String, "sub_topic2", self.sub_topic2_callback, 10)

        self.subscriber_handles["sub_topic3"] = self.create_subscription(String, "sub_topic3", self.sub_topic3_callback, 10)


        # Periodic publishing
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0


    def timer_callback(self):
        """
        Basic method for publishing. Get rid of or modify this method,
        but please include a description of the method as is done here.
        """
        msg = String()
        msg.data = 'Hello World: %d' % self.i

        self.publisher_handles["pub_topic1"].publish(msg)

        self.publisher_handles["pub_topic2"].publish(msg)

        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


    def sub_topic1_callback(self, msg):
        """
        Callback for the sub_topic1 topic.
        """
        self.get_logger().info(f"Received {msg} on topic sub_topic1")


    def sub_topic2_callback(self, msg):
        """
        Callback for the sub_topic2 topic.
        """
        self.get_logger().info(f"Received {msg} on topic sub_topic2")


    def sub_topic3_callback(self, msg):
        """
        Callback for the sub_topic3 topic.
        """
        self.get_logger().info(f"Received {msg} on topic sub_topic3")



def main(args=None):
    rclpy.init(args=args)

    new_node = NewNode()

    rclpy.spin(new_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**2.**
Before you can run the node, you need to make sure the build system `colcon` will see it. 

- **2a)** First, make it executable by running: 

```bash
chmod +x new_node.py
```

- **2b)** Next, update the `setup.py` file in `src/my_package/`:

```python
entry_points={
  'console_scripts': [
    'new_node = new_package.new_node:main' # <--- New line
  ],
},
```

**3.**

- **3a)** Now, return to the `workspace` directory,

```bash
cd /root/REPO/workspace
```

- **3b)** and run `colcon build` or `make build`.

- **3c)** Make sure to source your shell so the build changes take effect by running `source ./install/setup.bash` OR `source ./install/setup.bash`

**4.**
Once that has completed, verify that the executable can be seen:

```bash
ros2 pkg executables new_package
```

</p>
</details>

### Visualizing ROS Topics

Each control stack should also have a [submodule](https://git-scm.com/book/en/v2/Git-Tools-Submodules) for [ROSboard](https://github.com/wisconsinautonomous/rosboard). ROSboard allows easy visualization of ROS topics without the need for many external packages. It can also be easily including the ROS framework, meaning it can be incorporated in launch files and other ROS specific tools. To learn more, please reference the [README](https://github.com/wisconsinautonomous/rosboard).

The Docker container for `REPO`  should already be setup with ROSboard and should work out of the box. If you'd like to use ROSboard as a standalone node (outside of launch files), run the following command.

```bash
ros2 run rosboard rosboard_node
```

This will run the rosboard server and start a server on port `8888`. This means that the webpage from rosboard is served to the url `http://localhost:8888`. Navigate there in a browser and you should see ROSboard!

To visualize messages, click the hamburger menu at the top left. There, you can select what messages you'd like to visualize. The ROSboard repository was forked into the Wisconsin Autonomous team, so if there are custom visualizations that would like to be viewed with custom message types, this can be implemented there. Refer to the ROSboard documentation for information on how to do this.

### Visualizing Other GUI Apps

In addition to `rosboard`, you may also want to use `novnc` as a tool for visualizing gui apps. [novnc](https://novnc.com/info.html) is a tool for running [VNC](https://en.wikipedia.org/wiki/Virtual_Network_Computing) in a browser. With the correct setup, this allows you to run gui tools in a docker container and visualize them in your browser. Typically, running gui apps is very difficult in docker containers, but it is made simple with `novnc`.

The [wa\_cli](https://WisconsinAutonomous.github.io/wa_cli) provides a very convenient entrypoint at `wa docker novnc` to spin up a `novnc` container that can run on the same Docker `network` as the ros stack. Then, if the container has set it's `DISPLAY` environment variable to `novnc:0.0` (provided the `novnc` docker container has been named 'novnc'), any gui apps that are displayed can be seen in the browser at [http://localhost:8080/vnc\_auto.html](http://localhost:8080/vnc_auto.html) (also assuming the 8080 port has been exposed by the container.

As you can tell, there quite a few requirements. So, the `wa_cli` provides an entrypoint to do this for you. The documentation command can be found [here](https://wisconsinautonomous.github.io/wa_cli/usage.html#docker-novnc). To spin up the `novnc` container, please run the following command:

```bash
wa docker novnc
```

## Using Simulations

Simulations are powerful in order to quickly set up scenarios with your ROS stack. `wa_simulator` has been made just for this, along with it's ros bridge `wa_simulator_ros_bridge`. The documentation for the simulator can be found [here](https://WisconsinAutonomous.github.io/wa_simulator).

Because we are using Docker as the main tool for running ROS, the easiest way to have simulators interface with our software stack is also through Docker. This can be done via Docker networks, which were discussed earlier.

A more in-depth tutorial was created in the `wa_simulator` docs and that can be found [here](https://wisconsinautonomous.github.io/wa_simulator/tutorials/docker-usage.html).

### Running `wa_simulator` Scripts

To run `wa_simulator` scripts with your ROS software stack in docker, the `wa_cli` provides an entrypoint at `wa docker run` to do this. Please see the usage guide at [here](https://wisconsinautonomous.github.io/wa_cli/usage.html#docker-run).

As the documentation describes, to run a basic simulation script in the `wasim/` subfolder, you can run something similar to the following command:

```bash
wa docker run \
        --wasim \
        --data "wasim/data/" \
        wasim/script.py 
```

Where `script.py` is a `wa_simulator` script and `data/` holds the data files needed for the `wa_simulator` script, where both are inside the `wasim/` folder. 

## Other Tools

### License Script

It is important that files have a license header. Licenses aren't necessarily used to disallow people from stealing your code, but it actually allows them to use it.

The [wa\_cli](https://github.com/WisconsinAutonomous/wa_cli) has an entrypoint at `wa script license` that updates these headers automatically for you.

To do this, run the following command inside the root directory of `REPO`:

```
wa script license .
```

For more information on the `license` command, run the previous command with a `--help` flag.

## Support

Contact [Wisconsin Autonomous](mailto:wisconsinautonomous@studentorg.wisc.edu) for any questions or concerns regarding the contents of this post.

## See Also

Stay up to date with our technical info by following our [blog](https://wa.wisc.edu/blog).

Follow us on [Facebook](https://www.facebook.com/wisconsinautonomous/), [Instagram](https://www.instagram.com/wisconsinautonomous/), and [LinkedIn](https://www.linkedin.com/company/wisconsin-autonomous/about/)!

![WA Logo](/assets/img/logos/wa-white.png){: .left height="100"}
![Wisconsin Crest](/assets/img/logos/uw-crest.png){: .right height="100"}
