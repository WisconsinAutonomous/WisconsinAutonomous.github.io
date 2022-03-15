---
title: ROS Overview
author: Aaron Young
date: 2021-12-16 16:53:00 -0600
categories: [ROS]
tags: [ros, technical]
---

This guide outlines how Wisconsin Autonomous structures their ROS respositories and how ROS development should take place

## Purpose

A major difference between Wisconsin Autonomous and industry is that in WA we have much higher turnover. This is because students graduate or move on. In addition, industry has employees working 40hrs a week and WA has students working 5-10hrs a week. As a result, planning is very important to ensure the time we do spend working on WA is not wasted.

This guide goes into detail about how Wisconsin Autonomous structures their control stacks. We may have multiple control stacks for different applications; for example, one for the evGrand Prix, one for AutoDrive, and one for the rc car.

It is very important the ideas presented on this page are followed _to a T_. 

## Prerequisites

We'll refer to the control stack repository as `REPO` from now on. This simply helps for describing what's going on, replace `REPO` with the repository you're using, i.e. `waGrandPrix`.

## TL;DR

**This section is for quick reference!! If you have not read this document in-full yet, you _must_ do that first.**

### Required Packages

- [Docker](https://www.docker.com/)
- [Docker Compose](https://docs.docker.com/compose/)
- [Python3](https://www.python.org/)
- [Git](https://git-scm.com/)

### 1. Clone the Repository

_Needed only once._

```bash
git clone --recursive https://github.com/WisconsinAutonomous/REPO.git && cd REPO 
```

### 2. Install Host Python Dependencies

_Needed only once._

```bash
pip install -r requirements.txt
```

{% include note.html content="We recommend using a python environment (Anaconda, venv, etc.)." %}

### 3. Use the Docker Development Environment

**Building, Starting, and Entering:**

```bash
atk dev
```

**Tearing Down:**

```bash
atk dev --down
```

### 4. Test the Stack with wa_simulator

```bash
atk dev -r -s wasim --args python baseline_track.py -mv
```

{% include note.html content="This is an example, `baseline_track.py` may not exist in `REPO`." %}

### 5. Visualize GUI Applications

Navigate to [https://localhost:8080/](https://localhost:8080/).

## ROS

The [Robot Operating System (ROS)](https://www.ros.org/) is a major tool in the area of robotics. We use [ROS 2](https://docs.ros.org/en/galactic/) and for the remainder of this document, it is assumed you have a solid background with ROS related topics. This includes, but is not limited to, topics, nodes, messages, publishers, and subscribers. If any of this doesn't make sense, please refer to the [ROS tutorials](https://docs.ros.org/en/galactic/Tutorials.html).

We have also created an easy to setup ROS environment for learning ROS using [Docker](#docker) called [wa\_ros\_tutorial](https://github.com/WisconsinAutonomous/wa_ros_tutorial). There are instructions there on how to set it up.

## Docker

Additionally, [Docker](https://www.docker.com/) is used to mitigate setup headaches. Docker related background is out of the scope of this article, so please research it yourself to get a better understanding of what Docker really is. You _can_, in theory, get through this entire tutorial without understanding what Docker is, but that doesn't help you learn things, right?

## Repository Structure

Each control stack used by Wisconsin Autonomous should be structured as follows:

```
REPO
├── .atk.yml 
├── docker/             # Dockerfiles
├── docs/               # Documentation/tutorials specific to this repo
├── misc/               # Miscellaneous scripts and resources
├── wasim/              # wa_simulator scenarios that can be used to test the control stack
├── workspace/          # ROS 2 workspace
└── README.md
```

A template has been made with the aforementioned structure to streamline the creation process for this type of repo. The template can be found [here](https://github.com/WisconsinAutonomous/wa-control-stack).

### `.atk.yml`

[Docker Compose](https://docs.docker.com/compose/) is a tool for running multi-container Docker applications. For the most part, it is used in this way; however, it comes with an entrypoint called `docker compose` that simplifies building/running/executing on containers and it is cross platform (i.e. it replaces bash scripts with intricate command line arguments).

The `.atk.yml` file defines custom configuration variables for the `docker` containers within it. All `docker-compose.yml` files, which the `.atk.yml` file provides custom edits for, will have atleast two `services`: `dev` and `vnc`. `dev` is meant specifically for local development of the ROS stack. `vnc` is used for visualization purposes. On the actual vehicle, we'll use a more refined docker container or run it outside of docker entirely.

### `docker/`

This folder holds the Dockerfiles that are used to generate the images that we'll run the control stack in. For the most part, these don't need to be changed. 

### `docs/`

This folder holds general documentation or tutorials specific to this repository. Anything that may be similar across each repos should just go in this guide.

### `misc/`

This should house miscellaneous resources and/or scripts. This should again be specific to the repository. Anything common across repos should go in a shared repository.

### `sim/`

This folder contains scripts for interacting with simulators. For example, `wa_simulator` scripts should be placed here. This folder, in conjunction with the `atk`, is simply a way of providing simulation files without the need to create them each time or build the simulators locally.

### `workspace/`

This is where we put the ROS 2 workspace. This is what is specific to each repository. Most of the development will take place here, and we'll go into more detail in future sections of this guide.

## Setup and Installation

**DISCLAIMER**: For developing the ROS 2 workspace, you _must_ use the created docker images. This is a requirement because we don't want to have to support multiple operating systems and have to deal with individual people's systems. The only place docker may not be used with this repository is on the actual vehicle (and with our own hardware).

Docker is a powerful application that is out of the scope of this guide. Beyond Wisconsin Autonomous, knowing what and how to use Docker is a very valuable skill. To learn more, visit [their website](https://www.docker.com/). You can also find a huge amount of tutorials and resources just through Google. Please spend some time understanding Docker before continuing. For the remainder of this tutorial, it is assumed you understand `docker` and `docker compose` at least on the surface.

For all of the commands we'll mention, they must be run from within the target repository (i.e. `cd` into the `REPO` folder). 

### Clone the Repository

An example command to clone the `REPO` repository locally is provided below:

```bash
git clone --recursive https://github.com/WisconsinAutonomous/REPO.git && cd REPO 
```

{% include note.html content="`--recursive` is recommended so that all [git submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules) are automatically pulled along with the main repository. If this isn't done during this step, you may do this after the fact with `git submodule update --init`." %}

### Installing Required Packages

First, you need to install any necessary python packages for `REPO`. There should be a file called `requirements.txt` at the root of `REPO`, which contains any python packages needed to run the control stack. At the very least, this file will contain `autonomy-toolkit`, which is the command line interface tool created for the Wisconsin Autonomous team.

To install these packages, run the following command:

```bash
pip install -r requirements.txt
```

{% include note.html content="It is recommended to use a python environment of some sort (i.e. Anaconda or venv). Activate the environment prior to running the previous command." %}

### Installing Docker

To get started, you will need to install [Docker](https://docs.docker.com/desktop/) and [Docker Compose](https://docs.docker.com/compose/install/). Please refer to the previous links for instructions on how to install.

### Developing in the Control Stack

For development, we'll focus on the `dev` service.

The very first time you start up the control stack, it will need to build the image that the container uses. This may take a while, but should only need to be done the first time around.

It is _required_ that you use the `autonomy-toolkit` to start, exec, build, and tear down the containers. Using the `autonomy-toolkit`, you can start, build, and attach to the container with on command.

```shell
atk dev
```

Within the container, you should enter the shell in the `~/{project}/workspace` directory, where `{project}` is custom to `REPO` and is probably just a lowercase version of the stacks name. `~/{project}/` is a docker [volume](https://docs.docker.com/storage/volumes/), so any changes you make in this folder will be reflected on your system. I will say that again: the only file changes that will persist between your host system and the docker container are the files in `/{project}`. If you make any changes from within your docker container, ensure it is done in there. 

For development purposes, you can use whatever tools you'd like for editing the code (`Atom`, `VSCode`, etc.). Because `~/{project}` in the container is a volume (see previous paragraph), changes from your system will also be copied to your container.

Various tools are or can be installed to aid development, such as `tmux`. Feel free to leverage these. The `atk dev...` command can also be run from any terminal window to attach to a new shell session (as long as it's run from within this repository); this is the advantage of running the container in the background (detached).

For those interested, you may use other shells. `zsh` and `bash` should both already be installed, but you may need to install other shells, if desired.

### Stopping the Container

When you are finished and would like to free up resources on your computer, you may shutdown the container with either of the following commands.

```shell
atk dev --down
```

### Building the ROS Workspace

By default, the ROS distribution should be sourced automatically thanks to the shell configuration file. This will only be the case if you have built the workspace with `colcon` and there is an `setup.<shell>` file present at `~/{project}/workspace/install/`.

As a reminder, the ROS workspace is located in `~/{project}/workspace/`. To get started with ROS in the container, you will need to build the workspace. An example of how to do this is provided below, but how to use ROS beyond this example is outside the scope of this guide. NOTE: `colcon build` should only ever be run at `~/{project}/workspace/`.

```bash
colcon build
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

For example, to create a new python package, run the following: `ros2 pkg create --build-type ament_python new_package` from the `~/{project}/workspace/src` directory. This will create a new package at the directory level.

### Visualizing ROS Topics

Each control stack should also have a [submodule](https://git-scm.com/book/en/v2/Git-Tools-Submodules) for [ROSboard](https://github.com/wisconsinautonomous/rosboard). ROSboard allows easy visualization of ROS topics without the need for many external packages. It can also be easily including the ROS framework, meaning it can be incorporated in launch files and other ROS specific tools. To learn more, please reference the [README](https://github.com/wisconsinautonomous/rosboard).

The Docker container for `REPO`  should already be setup with ROSboard and should work out of the box. If you'd like to use ROSboard as a standalone node (outside of launch files), run the following command.

```bash
ros2 run rosboard rosboard_node
```

This will run the rosboard server and start a server on port `8888`. This means that the webpage from rosboard is served to the url `http://localhost:8888`. Navigate there in a browser and you should see ROSboard!

To visualize messages, click the hamburger menu at the top left. There, you can select what messages you'd like to visualize. The ROSboard repository was forked into the Wisconsin Autonomous team, so if there are custom visualizations that would like to be viewed with custom message types, this can be implemented there. Refer to the ROSboard documentation for information on how to do this.

### Visualizing Other GUI Apps

In addition to `rosboard`, you may also want to use `vnc` as a tool for visualizing gui apps. [novnc](https://novnc.com/info.html) is a tool for running [VNC](https://en.wikipedia.org/wiki/Virtual_Network_Computing) in a browser. With the correct setup, this allows you to run gui tools in a docker container and visualize them in your browser. Typically, running gui apps is very difficult in docker containers, but it is made simple with `novnc`. Furthermore, `vnc` can also be used using vnc viewers.

The [`autonomy-toolkit`](http://projects.sbel.org/autonomy-toolkit) provides a very convenient `vnc` container that can be explicitly invoked with `atk dev -s vnc -u` to spin up a `vnc` container that can run on the same Docker `network` as the ros stack. Then, if the container has set it's `DISPLAY` environment variable to `vnc:0.0` (provided the `vnc` docker container has been named 'vnc'), any gui apps that are displayed can be seen in the browser at [http://localhost:8080/](http://localhost:8080/) (also assuming the 8080 port has been exposed by the container).

{% include note.html content="By default, the `atk dev` command will run spin up the `vnc` container implicitly. You probably don't need to explicitly start up the `vnc` container yourself." %}

## Using Simulations

Simulations are powerful in order to quickly set up scenarios with your ROS stack. `wa_simulator` has been made just for this, along with it's ros bridge `wa_simulator_ros_bridge`. The documentation for the simulator can be found [here](https://WisconsinAutonomous.github.io/wa_simulator).

Because we are using Docker as the main tool for running ROS, the easiest way to have simulators interface with our software stack is also through Docker. This can be done via Docker networks, which were discussed earlier.

A more in-depth tutorial was created in the `wa_simulator` docs and that can be found [here](https://wisconsinautonomous.github.io/wa_simulator/tutorials/docker-usage.html).

### Running `wa_simulator` Scripts

To run `wa_simulator` scripts with your ROS software stack in docker, the `REPO` should provide a `wasim` service that can be invoked with `atk`. 

As the documentation describes, to run a basic simulation script in the `sim/wasim` subfolder, you can run something similar to the following command:

```bash
atk dev -r -s wasim wa_simulator_script.py -mv
```

## Other Tools

{% include note.html content="To update..." %}

## Support

Contact [Wisconsin Autonomous](mailto:wisconsinautonomous@studentorg.wisc.edu) for any questions or concerns regarding the contents of this post.

## See Also

Stay up to date with our technical info by following our [blog](https://wa.wisc.edu/blog).

Follow us on [Facebook](https://www.facebook.com/wisconsinautonomous/), [Instagram](https://www.instagram.com/wisconsinautonomous/), and [LinkedIn](https://www.linkedin.com/company/wisconsin-autonomous/about/)!

![WA Logo](/assets/img/logos/wa-white.png){: .left height="100"}
![Wisconsin Crest](/assets/img/logos/uw-crest.png){: .right height="100"}
