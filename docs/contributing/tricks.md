---
layout: default
title: Tricks
nav_order: 5
parent: Contributing
---

# Tricks

{: .no_toc }

---

## Tools

- You can get the publishing frequency of a topic with `$ ros2 topic hz [topic_name]`
- Show a graph of all current topics and nodes with `$ rqt_graph`
- VS Code's "Remote - SSH" extension is a good way to develop remotely on the Quad. Files, terminals, and more all appear as though you were working locally on the Quad.
- Other useful VS Code extensions include Prettier, GitLens, Doxygen Documenttion Generator (C++), autoDocstring (Python), and the C/C++ entension pack.

## Other

- ROS [Services](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html) are alternatives to publishers/subscribers. They return results only when requested by a Client.
