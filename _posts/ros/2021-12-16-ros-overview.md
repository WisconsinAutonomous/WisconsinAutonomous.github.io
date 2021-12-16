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

This guide goes into detail about how Wisconsin Autonomous structures their control stacks.

It is very important the ideas presented on this page are followed. 

## Prerequisites

#### ROS

The [Robot Operating System (ROS)](https://www.ros.org/) is a major tool in the area of robotics. We use [ROS 2](https://docs.ros.org/en/foxy/) and for the remainder of this document, it is assumed you have a solid background with ROS related topics. This includes, but is not limited to, topics, nodes, messages, publishers, and subscribers. If any of this doesn't make sense, plesae refer to the [ROS tutorials](https://docs.ros.org/en/foxy/Tutorials.html).

#### Docker

Additionally, [Docker](https://www.docker.com/) is used to mitigate setup headaches. Please research Docker to understand what it is exactly. Detailed instructions will be provided on how to use it.

## Repository Structure

Each control stack used by Wisocnsin Autonomous should be structured as follows.

```
.
├── docker-compose.yml
├── docker/             # Dockerfiles
├── docs/               # Documentation/tutorials specific to this repo
├── misc/               # Miscellaneous scripts and resources
├── workspace/          # ROS 2 work signals
├── _templates/         # Templates for generating ROS nodes
└── README.md
```

### `docker-compose.yml`

[Docker Compose](https://docs.docker.com/compose/) is a tool for running multi-container Docker applications. For the most part, it is used in this way; however, it comes with an entrypoint called `docker-compose` that simplifies building/running/executing on containers that is cross platform.

The `docker-compose.yml` file defines configuration variables for the `docker` containers within it. 

### `docker`



### `_templates`

## Setup Guide


## Support

Contact [Your Name](mailto:wiscid@wisc.edu) for any questions or concerns regarding the contents of this post.

## See Also

Stay up to date with our technical info by following our [blog](https://www.wisconsinautonomous.org/blog).

Follow us on [Facebook](https://www.facebook.com/wisconsinautonomous/), [Instagram](https://www.instagram.com/wisconsinautonomous/), and [LinkedIn](https://www.linkedin.com/company/wisconsin-autonomous/about/)!

![WA Logo](/assets/img/logos/wa-white.png){: .left height="100"}
![Wisconsin Crest](/assets/img/logos/uw-crest.png){: .right height="100"}
