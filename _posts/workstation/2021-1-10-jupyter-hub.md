---
title: JupyterHub
author: Aaron Young
date: 2021-01-10 18:34:00 -0600
categories: [Technical, Workstation]
tags: [tutorials, workstation, technical]
---

[JupyterHub](https://jupyter.org/hub) is a powerful program that allows companies, classrooms or research labs to share computational hardware and run _notebook_ based code concurrently. [Jupyter's](https://jupyter.org/index.html) goal is to provide interactive computing software for different programming languages. JupyterHub allows multiple users to take advantage of a single server by separating user processes and programs. 

We foresee much of our development to be done through JupyterHub as it is simple to use, scalable and functions well for multiple use cases.

## Setup Guide

JupyterHub (and Jupyter in general) has a server based design. A server is responsible for running the actual code, and a client (typically visualized in a browser) can write code for the server to run. On our workstation, there is a always running server setup. This is nice, as you will never need to launch the server yourself. So, JupyterHub is essentially setup already.

[JupyterLab](https://jupyterlab.readthedocs.io/en/stable/) is the next-generation user interface for Jupyter and can include Jupyter Notebooks. It is a very intuitive and clean program, so we have opted to use it as opposed to just regular Jupyter Notebooks.

### Initializing an SSH Tunnel

For you to be able to visualize JupyterHub in your browser, your computer needs to know that there is a JupyterHub instance running on the workstation. This can be done through something called an `ssh` Tunnel. Visit [this](/posts/ssh#ssh-tunnel) post for information on how to setup an `ssh` tunnel.

Jupyter is run on port `8000`, so you must forward that port to your own computer. In theory, you can map `8000` to any port you'd like, but we'll leave it at `8000` for simplicity. Your command may look something like this:
```shell
ssh -L 8000:localhost:8000 -J <cae username>@best-tux.cae.wisc.edu <cae username>@carproject-06.engr.wisc.edu
```

The `-L` option will forward port `8000` on the work station to your computer at `8000`. The `-J` command will be a `ProxyJump` and feed you through the cae computers to the work station.

### Opening JupyterHub in your Browser

Your computer should now have access to the JupyterHub server. To open up Jupyter, go to a browser and type in `localhost:8000/jupyter`. This should open a webpage with a login prompt. Login with your cae credentials (same as the work station). It should look similar to this:

![Jupyter Login](/assets/img/workstation/jupyter_login_screen.png)

> Note: Currently, a non-secure `http` connection is used. Your information and activity is safe because of cae's vpn, however a more secure `https` connection will be used in the future.

Once logged in, a JupyterLab instance will load. You can edit, delete and add files via the file explorer on the left side. If you click on a file, JupyterLab will open it similar to a regular IDE. Your browser should look similar to the following image.

![Jupyter Launch](/assets/img/workstation/jupyter_launch_screen.png)

### Creating a Jupyter Kernel

As you can see in the previous image (or in your browser window), within the Launcher tab, there is a Notebook and a Console section. Within those sections, there should be a single square titled "Python3". For JupyterLab to work, there are isolated "kernels" that are used to run various programs. The "Python3" kernel is installed globally on the workstation and is **not** configurable by most users. In order to install custom packages, you will want to create a new kernel.

[Anaconda](https://www.anaconda.com/) is a fantastic tool for scientific computing and package management. It is the tool we use for isolating python environments between users. Please see [this post](/posts/anaconda) for an explanation on how to setup your own `conda` environment. _For the remaining commands in this section, it is assumed you have setup **and activated** your `conda` environment_.

Head back to your terminal window and with a `conda` environment setup **and activated**, you can now create your own kernel. Run the following command to do so:
```shell
conda install ipykernel
ipython kernel install --user --name 'WA' --display-name "WA"
```

This will create a kernel with the name `WA` and will be configured for your user account so you can install and uninstall packages as you please. Reload the JupyterLab window in your browser. It should now look like the image below.

![Jupyter Post Kernel Creation](/assets/img/workstation/jupyter_post_kernel_creation.png)

## Using JupyterLab

Please visit the [JupyterLab documentation](https://jupyterlab.readthedocs.io/en/stable/user/interface.html) for more information about the interface and uses. Video tutorials will be posted in the future documenting specific use cases for Wisconsin Autonomous.

## Management

This JupyterHub instance was configured with simplicity and scalability in mind. Because long term changes were unknown at the time (whether we will get more workstations, number of users, etc.), [The Little JupyterHub](https://tljh.jupyter.org/en/latest/) was not chosen here. Instead, [this tutorial](https://jupyterhub.readthedocs.io/en/stable/installation-guide-hard.html) was followed to configure the system.

For your convenience, a installation guide is described below. Only information regarding configuration is given in here. Some steps vary from the original tutorial, so please refer to this guide if the goal is to configure the system.

### Setting up JupyterHub

The JupyterHub instance was created in a virtualenv located at `/opt/jupyterhub`. As you'll see later, this is the location we will place many configuration based files.

The `jupyterhub_config.py` file, the main file that manages the system configuration for JupyterHub, is located at `/opt/jupyterhub/etc/jupyterhub/` (within the virtualenv we created earlier). By default, the user interface is set to display as JupyterLab, which is essentially a newer release of Jupyter Notebook with extended IDE-like features. To changes this, set the following option in the `jupyterhub_config.py` file (currently it's set to `/lab`):
```yaml
c.Spawner.default_url = '/tree'
```

To have JupyterHub run on startup, a system service used and run by [Systemd](https://man7.org/linux/man-pages/man1/systemd.1.html). The `.service` file is located at `/opt/jupyterhub/etc/systemd/jupyterhub.service`. There shouldn't be much to change here, but this basically just runs JupyterHub and loads in the config file we created.

[Anaconda](https://docs.conda.io/en/latest/) was used for managing different Python environments. Anaconda is managed using modulefiles, of which we wrote a [post about](/posts/modules). The location of the `conda` environments deviates from the original guide because of the `modulefile` management system. A default `conda` environment was made with only a few packages. Since it is a global install, `pip install` or `conda install` isn't allowed. Instead, users should create their own conda environment to have custom installs.

Furthermore, located within the `jupyterhub_config.py`, the `autonomous-sudo` user group has been configured as admins of the JupyterHub instance.

:exclamation: TODO: Details on how to setup conda env for user :exclamation:

:exclamation: TODO: Security details. As of 1/5/2020 our domain (wa.wisc.edu) is owned by Wix.com, so will need to wait at a minimum 60 days to configure custom URL, i.e. still have to use `localhost:8000/jupyter` :exclamation:

## Support

Contact [Aaron Young](mailto:aryoung5@wisc.edu) for any questions or concerns regarding the contents of this repository.

## See Also

Stay up to date with our technical info by following our [blog](https://wa.wisc.edu/blog).

Follow us on [Facebook](https://www.facebook.com/wisconsinautonomous/), [Instagram](https://www.instagram.com/wisconsinautonomous/), and [LinkedIn](https://www.linkedin.com/company/wisconsin-autonomous/about/)!

![WA Logo](/assets/img/logos/wa-white.png){: .left height="100"}
![Wisconsin Crest](/assets/img/logos/uw-crest.png){: .right height="100"}