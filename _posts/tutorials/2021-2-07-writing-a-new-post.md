---
title: Writing a New Post
author: Aaron Young
date: 2021-02-07 15:42:00 -0600
categories: [Introduction, Wiki Tutorials]
tags: [tutorials, non-technical]
---

This post outlines how to write a new post and deploy it to the GitHub hosted website.

## Guidelines

The site is for internal use, so no worries if mistakes are made. However, there are a few rules that we recommend you abide by to keep everything organized.

- Only push to github when you have verified that everything runs successfully ([see this section](#local-development-environment))
	- The website is built using [GitHub Actions](https://github.com/features/actions)
	- Actions will build the site automatically, **but only a maximum of 10 times per hour**
	- If you push more than that, the site may not be built and you may get an email
- When you push to github, go to the [actions tab](https://github.com/WisconsinAutonomous/WisconsinAutonomous.github.io/actions) to verify it built successfully.

### Organization

All posts should go in the `_posts` folder. Place the post in any reasonable subfolder or make your own. See [this post](/posts/post-markdown-overview) to understand the syntax for developing the content.

All pictures/files/assets should go in `assets/img/` and place them in any reasonable subfolder.

_Keep it organized_!!

## Setup

In order to write a new post, you will need to clone the [repository](https://github.com/WisconsinAutonomous/WisconsinAutonomous.github.io). This can be done with the following command:
```shell
git clone https://github.com/WisconsinAutonomous/WisconsinAutonomous.github.io.git
cd WisconsinAutonomous.github.io
```

This will basically download the code to your system so you can edit it.

> Note: The fundamentals of `git` are described in [this post](/posts/git).

## Local Development Environment

To create a local development environment, it's definitely recommended to use [Anaconda](https://docs.anaconda.com/anaconda/install/) so you don't have unneeded files persisting on your system.

To do so, [setup](#setup) this repository on your local system and create the conda environment with this command:
```shell
conda create -n wa-web ruby gcc_linux-64 gxx_linux-64
conda activate wa-web
```
> Note: Change wa-web to whatever name you'd like

Next, we need to apply a patch/fix to the anaconda environment to be able to use it for development. Copy and paste the following commands into _the same where you have activated the environment_. **This will take a minute or two**.
```shell
SAVED_DIR=$PWD; cd $(gem environment gemdir); cd ../../$(basename $PWD)/$(gem environment platform | sed -e 's/.*://'); mv rbconfig.rb rbconfig.rb.bu; perl -pe 's/\/\S*?\/_build_env\/bin\///g' rbconfig.rb.bu > rbconfig.rb; gem install bundler jekyll; cd $SAVED_DIR; bundle
```

Once everything is installed, you can create a local server that you can access on your browser to see your changes. Run the following command to do that:
```shell
bundle exec jekyll s
```

Now, navigate to `http://127.0.0.1:4000/` to see the site!

## Deploying

Deployment happens when a commit is pushed to the `master` branch. Feel free to push to any branch (besides `gh-pages`) throughout the process for version control purposes. If you feel hesitant to push, test your [environment locally](#local-development-environment) and ask a team lead to check it for you!

## Support

Contact [Aaron Young](mailto:aryoung5@wisc.edu) for any questions or concerns regarding the contents of this post.

## See Also

Stay up to date with our technical info by following our [blog](https://www.wisconsinautonomous.org/blog).

Follow us on [Facebook](https://www.facebook.com/wisconsinautonomous/), [Instagram](https://www.instagram.com/wisconsinautonomous/), and [LinkedIn](https://www.linkedin.com/company/wisconsin-autonomous/about/)!

![WA Logo](/assets/img/logos/wa-white.png){: .left height="100"}
![Wisconsin Crest](/assets/img/logos/uw-crest.png){: .right height="100"}