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

To simplify the development environment, we'll use [Docker](https://www.docker.com/). Please refer to the official documentation for an installation guide.

We'll be using docker to basically build the wiki in a linux server, and then we'll serve it to a local url at `https://localhost:4000`. To do this, run the following command at the root of the `WisconsinAutonomous.github.io` repo (assumes you have installed docker):

```bash
docker run -it --rm --volume="$PWD:/srv/jekyll" -p 4000:4000 jekyll/jekyll jekyll serve
```

It will take a few minutes to build. Once it is built, you can navigate to `https://localhost:4000` to see the built changes. The docker container will continue to run until you stop the process. Any new changes you make to the repository will then cause a rebuild, which will can be seen at the url.

## Deploying

Deployment happens when a commit is pushed to the `master` branch. Feel free to push to any branch (besides `gh-pages`) throughout the process for version control purposes. If you feel hesitant to push, test your [environment locally](#local-development-environment) and ask a team lead to check it for you!

## Support

Contact [Aaron Young](mailto:aryoung5@wisc.edu) for any questions or concerns regarding the contents of this post.

## See Also

Stay up to date with our technical info by following our [blog](https://wa.wisc.edu/blog).

Follow us on [Facebook](https://www.facebook.com/wisconsinautonomous/), [Instagram](https://www.instagram.com/wisconsinautonomous/), and [LinkedIn](https://www.linkedin.com/company/wisconsin-autonomous/about/)!

![WA Logo](/assets/img/logos/wa-white.png){: .left height="100"}
![Wisconsin Crest](/assets/img/logos/uw-crest.png){: .right height="100"}
