# pyOpenDRIVE (NOVA Branch)

**pyOpenDRIVE is a Python wrapper of [libOpenDRIVE](https://github.com/pageldev/libOpenDRIVE)** implemented via Cython. 

**This branch is specially modified for use by UTD's Nova project, and is set up as a ROS2 package!**

It's small and can be easily integrated in other projects. A core function is the parsing of OpenDRIVE files and the generation of 3D models. libOpenDRIVE, and by extension pyOpenDRIVE, targets OpenDRIVE version 1.4.

The [src](/src/) directory contains the original C++ libOpenDRIVE source code.
The [pyOpenDRIVE](/pyOpenDRIVE/) directory contains the original header files, as well as the Cython wrapper `.pxd` and `.pyx` files.

## Example
An example of how this wrapper works can be found in [test.py](test.py).