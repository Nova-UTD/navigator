---
layout: default
title: Maps
---

# Maps

{: .no_toc }

_Maintained by Will Heitman_

## Table of contents

{: .no_toc .text-delta }

1. TOC
   {:toc}

## Grid maps

All grid maps are in the `base_link` (vehicle) reference frame. The semantic grid extends 40 meters ahead, 20 meters behind, and 30 meters to either side of the car.
![Cost map dimensions](assets/res/cost_map_size.jpg)

---

## Semantic map

This is a general-purpose grid map drawn from OpenDRIVE map data, where each cell is tagged with an appropriate class.

| ID   | Description                                                  |
| ---- | ------------------------------------------------------------ |
| 0    | None/unknown. Grass, buildings, and other regions not described in the OpenDRIVE map. |
| 1    | Road                                                         |
| 2    | Shoulder                                                     |
| 3    | Curb                                                         |
| 4    | Sidewalk                                                     |
| 5    | Median                                                       |
| 11   | Stop sign                                                    |
| 12   | Speed limit sign                                             |
| 13   | Traffic light                                                |
| 21   | Crosswalk                                                    |

