---
title: Package Managers
author: Aaron Young
date: 2021-01-10 18:34:00 -0600
categories: [Technical, Miscellaneous]
tags: [technical]
---

A package manager is a software tool that automates installing, upgrading, configuring and removing computer programs consistently and reliably. In most applications, package managers are non-graphical tools.

In this README, multiple package managers are explained, along with their setup and use. Additional package managers may be added in the future!

are explained, along with their setup and use. Additional package managers may be added in the future!

## Table of Contents
- [Homebrew](#homebrew)
	- [Setup Guide](#homebrew-setup-guide)
	- [Usage](#homebrew-usage)
	- [See Also](#homebrew-see-also)
- [Chocolatey](#chocolatey)
	- [Setup Guide](#chocolatey-setup-guide)
	- [Usage](#chocolatey-usage)
	- [See Also](#chocolatey-see-also)
- [Support](#support)
- [See Also](#see-also)


## Homebrew

[Homebrew](https://brew.sh/) is an open-source package manager made to simplify the installation of software on MacOS or Linux operating systems (MacOS will be the focus in this document).

### Homebrew Setup Guide

Homebrew has been made to be really simple to install. To install homebrew, you need to download an installation script from the internet. The following command will download the script using `curl` and then install `brew`.

```shell
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

### Homebrew Usage

`brew` has multiple helpful commands to install, update or configure your machine. A few of the most useful ones will be described below.

- `brew install` is used (as its name suggests) to install packages. For example, to install `git`, which is a version control tool [(we wrote a README on it)](/posts/git), you would run `brew install git`.

- `brew install --cask` is a special install command that allows you to install larger, UI based applications. To install firefox, you would need to run `brew install --cask firefox`.

- `brew uninstall`, similar to `brew install`, is used to uninstall a package. As an example, to uninstall `git`, run `brew uninstall git`.

- `brew search` allows you to search for a package and check if it exists. You can simply run `brew search python` to see all available packages with python in its name.

`brew upgrade` and `brew update` are used for checking for updates and installing those updates. It makes sense to regularly run these commands to keep all download packages up-to-date. _Warning_: When a package is updated, other packages that depend on it (called dependencies) will also update which could affect other, unrelated packages.

### Homebrew See Also

The [homebrew](https://brew.sh/) is really helpful for checking available packages. Direct documenation can be found [here](https://docs.brew.sh/).

## Chocolatey
### Chocolatey Setup Guide
### Chocolatey Usage
### Chocolatey See Also

## Support

Contact [Aaron Young](mailto:aryoung5@wisc.edu) for any questions or concerns regarding the contents of this repository.

## See Also

Stay up to date with our technical info by following our [blog](https://wa.wisc.edu/blog).

Follow us on [Facebook](https://www.facebook.com/wisconsinautonomous/), [Instagram](https://www.instagram.com/wisconsinautonomous/), and [LinkedIn](https://www.linkedin.com/company/wisconsin-autonomous/about/)!

![WA Logo](/assets/img/logos/wa-white.png){: .left height="100"}
![Wisconsin Crest](/assets/img/logos/uw-crest.png){: .right height="100"}