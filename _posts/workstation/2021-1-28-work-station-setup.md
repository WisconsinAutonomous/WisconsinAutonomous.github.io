---
title: Work Station Setup
author: Aaron Young
date: 2021-01-28 14:09:00 -0600
categories: [Technical, Workstation]
tags: [tutorials, technical, workstation]
---

Please see [this post](/posts/work-station-info) for more background information on the workstation.

## Use Guidelines

The computer was built so that many people could use it at the same time. When using the work station, please be respectful so that everyone has a positive experience.

## User Setup

User accounts are created via CAE's user system. This means your login and user name is configured directly from your CAE account. For those of you already in the College of Engineering, just use your already made account. If you are in a different school (i.e. Letters and Sciences) or don't have a CAE account, [please see below](#cae-account).

Once you have a CAE account, you can register your wisc id with team leaders to gain access. This was done so that only team members can login to the computer. Message your team leader on Slack and we'll give you access!!

### CAE Account

If you have a CAE account, no need to read this section. For those of you without a CAE account and would like access, you can register an account [here](https://newuser.my.cae.wisc.edu/account). Once submitted, it may take a few days to receive approval.

### Account Registration

Login access to the workstation is restricted to approved individuals. To gain access, please message a team leader with your cae account information. We can then get your user setup.

## Using the Work Station

There are multiple ways of using the work station. At the time of writing, I will assume physical access to the lounge is _not_ available due to Covid19.

All subsequent techniques require prior approval by the team leads. Please see [above](#account-registration) for how to do that.

### SSH

To get an understanding of what the Secure Shell (SSH) is, please refer to [this post](/posts/ssh).

Due to the University's firewall, _most_ students must ssh through the cae computers. Your SSH route will look like this: Your Computer -> CAE Computers -> WA Work Station.

On an SSH manager or in a terminal window, you can run the following command:
```bash
ssh -J <cae username>@best-tux.cae.wisc.edu <cae username>@carproject-06.engr.wisc.edu
```

The previous command will simply feed you through the cae computers. To improve productivity, you may want to consider adding a `ProxyCommand` to your ssh config file or using ssh keys. This is described in the [SSH post](/posts/ssh).

### JupyterHub

JupyterHub is a program that allows you to run code on the workstation from your own browser. Some detailed info and full setup instructions are described in [this post](/posts/jupyter-hub).

To allow for communication between your computer and the workstation, you must first setup an SSH tunnel. Although it is explained in [this post](/posts/jupyter-hub), for easy reference, I have placed that command here:
```bash
ssh -L 8000:localhost:8000 -J <cae username>@best-tux.cae.wisc.edu <cae username>@carproject-06.engr.wisc.edu
```

After running the previous command in a terminal or an ssh agent, you may then go to your browser and navigate to `http://localhost:8000/jupyter`.

### VNC

Virtual Network Computing (VNC) is a powerful tool to allow users to use graphical desktops without being physically connected to the computer. This means you can use visual applications, not just the terminal, with VNC. A detailed setup and usage guide is located [here](/posts/vnc).

VNC setup is a little more involved than the previous two methods. Please refer to the [setup guide](/posts/vnc) for instructions how to use it.

## Troubleshooting

### Permission Denied

Some may get an error that looks like the following:
```console
user@name:~$ ssh name@carproject-06.engr.wisc.edu
name@carproject-06.engr.wisc.edu's password:
Permission denied, please try again.
name@carproject-06.engr.wisc.edu's password:
Permission denied, please try again.
name@carproject-06.engr.wisc.edu's password:
Permission denied (publickey,password).
```

As the errors demonstrate, you most likely do not have permission to use the workstation. Please see [this section](/posts/work-station-setup#account-registration) for details on how to gain access.

## Support

Contact [Aaron Young](mailto:aryoung5@wisc.edu) for any questions or concerns regarding the contents of this post.

## See Also

Stay up to date with our technical info by following our [blog](https://wa.wisc.edu/blog).

Follow us on [Facebook](https://www.facebook.com/wisconsinautonomous/), [Instagram](https://www.instagram.com/wisconsinautonomous/), and [LinkedIn](https://www.linkedin.com/company/wisconsin-autonomous/about/)!

![WA Logo](/assets/img/logos/wa-white.png){: .left height="100"}
![Wisconsin Crest](/assets/img/logos/uw-crest.png){: .right height="100"}