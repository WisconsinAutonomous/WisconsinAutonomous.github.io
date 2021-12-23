---
title: Anaconda
author: Aaron Young
date: 2021-01-29 14:10:00 -0600
categories: [Technical, Workstation]
tags: [tutorials, technical, workstation]
---

[Anaconda](https://docs.anaconda.com/) is a super powerful tool for scientific computing and provides a simple package management and deployment system. Although this post is primarily for use on the work station, Anaconda is an industry standard and is a great program to learn!

On our work station, Anaconda is the tool we've chosen for isolating our python environments. Isolation essentially means that each user can install and configure python packages however they please _without_ messing around with everyone elses. You _must_ use Anaconda to install python packages, as it is the only place where python is available. Furthermore, you need Anaconda for other tools, such as JupyterHub and the simulation. Anaconda is not restricted to just python, as it can be used to install other software tools independent from the whole system.

## Setup Guide

Setup is fairly simple and straight forward. Please SSH to the work station. Refer to [this post](/posts/ssh) for an explanation on how to do that.

### Load Anaconda

Anaconda is already installed to the system via [Modules](/posts/modules). To load the anaconda module, run the following command:
```shell
module load anaconda/4.9.2
```

Running `conda env list` should now display two environments installed to the system. 
```console
user@name:~$ conda env list
# conda environments:
#
base                     /usr/local/share/anaconda/4.9.2
jupyter                  /usr/local/share/anaconda/4.9.2/envs/jupyter
```

The `base` environment is the default environment for Anaconda. The `jupyter` env is used for [JupyterHub](/posts/jupyter-hub).

### Creating a User Environment

You will want to create a user environment so that you can install and uninstall packages. By default, the `base` and `jupyter` environments are read only, so you can't actually configure anything in those environments.

As an example, we will create a conda environment with python3.8 installed. To create this environment with the name `wa` and install python3.8, run the following command:
```shell
conda create --name wa python=3.8
```

Running `conda env list` should now display _three_ environments installed to the system. 
```console
user@name:~$ conda env list
# conda environments:
#
wa                       /filespace/a/<username>/.conda/envs/wa
base                     /usr/local/share/anaconda/4.9.2
jupyter                  /usr/local/share/anaconda/4.9.2/envs/jupyter
```

### Activating a Conda Environment

With your conda environment created, you can now activate the environment to be able to install/configure packages locally. To do that, run the following:
```shell
conda activate wa
```

> Note: Replace `wa` with any package name present in `conda env list`, if you'd like.

## Configuring your Shell

For most people, python and anaconda will be used very often. It is recommended to add the previous configuration commands directly to your shell script. This means that when you open up a terminal window, everything will be configured _automatically_.

To do that, run the following (_for non-bash users, make sure you change .bashrc to your shell profile_):
```shell
echo "module load anaconda/4.9.2" >> ~/.bashrc
echo "conda activate wa" >> ~/.bashrc
```

## Troubleshooting



## Support

Contact [Aaron Young](mailto:aryoung5@wisc.edu) for any questions or concerns regarding the contents of this repository.

## See Also

Stay up to date with our technical info by following our [blog](https://wa.wisc.edu/blog).

Follow us on [Facebook](https://www.facebook.com/wisconsinautonomous/), [Instagram](https://www.instagram.com/wisconsinautonomous/), and [LinkedIn](https://www.linkedin.com/company/wisconsin-autonomous/about/)!

![WA Logo](/assets/img/logos/wa-white.png){: .left height="100"}
![Wisconsin Crest](/assets/img/logos/uw-crest.png){: .right height="100"}