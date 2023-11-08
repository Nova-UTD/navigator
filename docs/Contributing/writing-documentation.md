---
layout: default
title: Writing documentation
parent: Contributing
---

# System overview
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

This documentation site is built off of Navigator's `dev` branch. All files within the `/doc` directory are remapped to `nova-utd.github.io`.

Adding to the site is easy. Here are two good options:

## Testing locally
To test locally, cd into `/docs` and run 

```text
./docs.sh
```
*This build script won't run unless you're exporting a* [``ROS_DOMAIN_ID``](../index.md#installation) *environment variable.*
Here's the original command:
```text
docker run --rm --volume="$PWD:/srv/jekyll:Z" -e JEKYLL_UID=$UID -e JEKYLL_GID=$UID -p $port:$port jekyll/jekyll jekyll serve --port $port
```


 Use a browser to view `localhost:(8000 + ROS_DOMAIN_ID)`. Refresh the page to show the latest updates. See the [official Docker README](https://github.com/envygeeks/jekyll-docker/blob/master/README.md) for more info.

## Editing online
On GitHub, move to the `/docs` directory on the `dev` branch ([here](https://github.com/Nova-UTD/navigator/tree/dev/docs)), then press the period key on your keyboard. This will open GitHub's web editor. See [here](https://code.visualstudio.com/Docs/languages/markdown#_markdown-preview) for more info.