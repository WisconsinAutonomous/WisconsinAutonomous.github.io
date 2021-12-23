---
title: Post Template
author: Aaron Young
date: 2021-01-10 15:19:00 -0600
categories: [Introduction, Wiki Tutorials]
tags: [tutorials, templates, non-technical]
---

This post is supposed to help expedite the post making process. Feel free to copy the [Front Matter](#front-matter) and the [Markdown Content](#markdown-content) directly into your new post.

## Writing a New Post
To understand the syntax behind Markdown and how your posts are displayed on the site, head over to [this post](/posts/post-markdown-overview).

## Creating the Post
Create your file under `_posts/<folder>` where `<folder>` is a folder that best describes the content of the post. If there is no folder that matches your content best, feel free to make a new one. 

The file **must** be titled like `YYYY-MM-DD-TITLE.EXTENSION`. Please note that the `EXTENSION` must be one of `md` and `markdown`.

### Using the `wa_cli`

You can also use the CLI exposed from the [wa\_cli](https://wisconsinautonomous.github.io/wa_cli). See the aforementioned link for installation information.

To make a template post, after installing `wa_cli`, you can use the following command:

```bash
wa wiki post --author <author> --catergory <category 1> --category <category 2> --tag <tag 1> --tag <tag 2> --tag <tag 3> <title>
```

For instance, to make the [Markdown Overview Post](/posts/writing-a-new-post), you'd run the following command:

```bash
wa wiki post --category Introduction --category "Wiki Tutorials" --author "Aaron Young" --email "aryoung5@wisc.edu" --tag tutorials --tag non-technical --title "Writing a New Post"
```

## Front Matter
This should go at the very top of your `.md` file. Make sure you update the values with the correct information. The timezone offset is currently set for Madison time (UTC-0600).

```yaml
---
title: New Post
author: Your Name
date: 2021-01-11 00:00:00 -0600
categories: [Introduction, Wiki Tutorials]
tags: [tutorials, templates, non-technical]
---
```

## Markdown Content
The title of the post will be automatically set by your `Front Matter`.

```markdown
This is a template for a new post for this site. Feel free to copy this markdown directly to your new file.

## Setup Guide


## Support

Contact [Your Name](wiscid@wisc.edu) for any questions or concerns regarding the contents of this post.

## See Also

Stay up to date with our technical info by following our [blog](https://www.wisconsinautonomous.org/blog).

Follow us on [Facebook](https://www.facebook.com/wisconsinautonomous/), [Instagram](https://www.instagram.com/wisconsinautonomous/), and [LinkedIn](https://www.linkedin.com/company/wisconsin-autonomous/about/)!

![WA Logo](/assets/img/logos/wa-white.png){: .left height="100"}
![Wisconsin Crest](/assets/img/logos/uw-crest.png){: .right height="100"}
```

## Full Post Template
```markdown
---
title: New Post
author: Your Name
date: 2021-01-11 00:00:00 -0600
categories: [Introduction, Wiki Tutorials]
tags: [tutorials, templates, non-technical]
---

This is a template for a new post for this site. Feel free to copy this markdown directly to your new file.

## Setup Guide


## Support

Contact [Your Name](mailto:wiscid@wisc.edu) for any questions or concerns regarding the contents of this post.

## See Also

Stay up to date with our technical info by following our [blog](https://www.wisconsinautonomous.org/blog).

Follow us on [Facebook](https://www.facebook.com/wisconsinautonomous/), [Instagram](https://www.instagram.com/wisconsinautonomous/), and [LinkedIn](https://www.linkedin.com/company/wisconsin-autonomous/about/)!

![WA Logo](/assets/img/logos/wa-white.png){: .left height="100"}
![Wisconsin Crest](/assets/img/logos/uw-crest.png){: .right height="100"}
```
