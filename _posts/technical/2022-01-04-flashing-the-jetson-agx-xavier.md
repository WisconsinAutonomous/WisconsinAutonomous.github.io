---
title: Flashing the Jetson AGX Xavier
author: Wisconsin Autonomous
date: 2022-01-04 15:09:15 -0600
categories: [Technical, Miscellaneous]
tags: [technical]
---

This guide outlines how to flash the [Jetson AGX Xavier](https://developer.nvidia.com/embedded/jetson-agx-xavier-developer-kit). This is the computer we use on our [evGrand Prix](http://evgrandprix.org/autonomous) vehicle. 

The Jetson AGX Xavier is built by [NVIDIA](https://www.nvidia.com/en-us/) and runs an operating system called [JetPack](https://developer.nvidia.com/embedded/jetpack), which is specific for Jetson computers and is built on top of Linux. "Flashing" the Jetson computer is essentially just reinstalling JetPack.

## Setup Guide

### Prerequsites

To begin, you must install [Docker](https://docker.com). Docker is a powerful tool that allows you to run [containers](https://www.docker.com/resources/what-container). The flashing software requires (as of writing) Ubuntu 18.04 to run, so using Docker, we can run Ubuntu 18.04 in a container on Windows, MacOS, or other versions of Ubuntu.

In summary, install Docker through their [official documentation](https://docs.docker.com/get-docker/).

### Flashing

Ensure the jetson is connected to your computer via USB. Then, run the following command:

```bash
sudo docker run -it --rm --privileged --network host -v /dev/bus/usb:/dev/bus/usb sdkmanager:1.7.0.8846 --cli install --logintype devzone --product Jetson --version 4.6 --targetos Linux --host --target JETSON_AGX_XAVIER_TARGETS --flash all
```

Apologies for the small amount of documentation in this guide. If this is done again, this guide will be updated with more information.

{% include note.html content="This command was only tested on Ubuntu 20.04, but the sdkmanager should theoretically work on other operating systems with some minor changes to the command; for instance, the `-v /dev/bus/usb:/dev/bus/usb` shares the usb drivers between the base operating system to the container and this may be different for another OS." %}

## Support

Contact [Wisconsin Autonomous](mailto:wisconsinautonomous@studentorg.wisc.edu) for any questions or concerns regarding the contents of this post.

## See Also

Stay up to date with our technical info by following our [blog](https://wa.wisc.edu/blog).

Follow us on [Facebook](https://www.facebook.com/wisconsinautonomous/), [Instagram](https://www.instagram.com/wisconsinautonomous/), and [LinkedIn](https://www.linkedin.com/company/wisconsin-autonomous/about/)!

![WA Logo](/assets/img/logos/wa-white.png){: .left height="100"}
![Wisconsin Crest](/assets/img/logos/uw-crest.png){: .right height="100"}
    
