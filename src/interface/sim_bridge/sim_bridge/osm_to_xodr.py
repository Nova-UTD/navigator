#!/usr/bin/python

'''
Nova at UT Dallas, 2022

The Navigator Simulation Bridge for CARLA

The goal is to mimick Hail Bopp as much as possible.

Targetted sensors:
- GNSS (GPS)
✓ IMU ()
- Front and rear Lidar
✓ Front  RGB camera
✓ Front depth camera
- CARLA ground truths for
    - Detected objects
    ✓ Car's odometry (position, orientation, speed)
    ✓ CARLA virtual bird's-eye camera (/carla/birds_eye_rgb)

Todos:
- Specific todos are dispersed in this script. General ones are here.
- Ensure all sensors publish in ROS coordinate system, NOT Unreal Engine's.

'''

import sys
sys.path.append('/home/share/carla/PythonAPI/carla/dist/carla-0.9.12-py3.7-linux-x86_64.egg')

import carla

# Read the .osm data
f = open("/home/share/maps/campus_openstreetmap_cropped.osm", 'r')
osm_data = f.read()
f.close()

# Define the desired settings. In this case, default values.
settings = carla.Osm2OdrSettings()
# Set OSM road types to export to OpenDRIVE
settings.set_osm_way_types(["motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link", "secondary", "secondary_link", "tertiary", "tertiary_link", "unclassified", "service"])
# Convert to .xodr
xodr_data = carla.Osm2Odr.convert(osm_data, settings)

# save opendrive file
f = open("/home/share/maps/campus_openstreetmap.xodr", 'w')
f.write(xodr_data)
f.close()