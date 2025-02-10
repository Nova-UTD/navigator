---
layout: default
title: Developer Etiquette
parent: Contributing
---

# System overview
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## Account on the Quad

The Quad is a centralized resource to run graphics and do training of models that might otherwise overly tax your PC.  The Quad has a fair bit of storage, however, docker images add up quickly and datasets for training can be quite large.  

For this reason, if you are recompiling docker files a lot, let Justin know so he can clean up some of the dangling images.  If you ever find that your docker images have been erased, it was probably that we needed to do a clean sweep of the docker images for a fresh start.

Also, we have dedicated separate hard drives to store large datasets for training, etc.  These are located in `/storage` and `/storage2`. Please use these locations to download large datasets and do not keep a copy in your home directory.  Storage will be audited occasionally and you may be asked to clean things up, but it would be helpful if you keep it streamlined.

## NoMachine Use

We use NoMachine as our remote desktop software. This allows you to have a graphical interface on the Quad, which is important while running graphics (like rviz). However, we currently have a 10 seat license (the subscription is not cheap), so it is essential that you *log out*, not simply *disconnect*. Leaving your session running will keep others from using the Quad. If you experience issues logging into the Quad, let Justin know (sometimes the NoMachine server acts up, but he can also boot people off that are not actively using their session).

## Dockerfile

One of the benefits of using the Quad's development environment is that we all get to share the same docker images, so we don't have to keep many copies of the same thing. However, since we all share the same image, when you are editing and changing the docker file for navigator (or any other repository) you should rename that docker image name until you are ready to make a pull request. You should be working off of a new branch, and you should rename the docker image with, e.g., `navigator_featurename` which is the same naming convention for branches.

The `docker-compose.yml` file determines the name of the images, so you should replace all instances of `navigator` with `navigator_featurename`. This would, for example, change `navigator_carla` into `navigator_featurename_carla`. The only instances of `navigator` that should not be changed are the lines that have `target: /navigator`.
