---
layout: default
title: Semantic segmentation
nav_order: 3
---

# Semantic segmentation

{: .no_toc }

_Maintained by Will Heitman_

## Table of contents

{: .no_toc .text-delta }

1. TOC
   {:toc}

---

## Class dictionary

Our semantic segmentation model is trained on the [Cityscapes Dataset](https://www.cityscapes-dataset.com/).

| Description                             | Class ID (int) | RGB                                                         |
| --------------------------------------- | -------------- | ----------------------------------------------------------- |
| Road                                    | 0              | <span class="swatch bg-road"></span>(128, 64, 128)          |
| Sidewalk                                | 1              | <span class="swatch bg-sidewalk"></span>(244, 35, 232)      |
| ~~Building~~ (overwritten by wall)      | 2              |                                                             |
| Wall                                    | 3              | <span class="swatch bg-wall"></span>(102,102,156)           |
| ~~Fence~~ (overwritten by wall)         | 4              |                                                             |
| Pole                                    | 5              | <span class="swatch bg-pole"></span>(153, 153, 153)         |
| Traffic light                           | 6              | <span class="swatch bg-traffic-light"></span>(250, 170, 30) |
| Traffic sign                            | 7              | <span class="swatch bg-traffic-sign"></span>(220, 220, 0)   |
| Vegetation                              | 8              | <span class="swatch bg-vegetation"></span>(107, 142, 35)    |
| Terrain                                 | 9              | <span class="swatch bg-terrain"></span>(145, 170, 100)      |
| Sky                                     | 10             | <span class="swatch bg-sky"></span>(70, 130, 180)           |
| Person                                  | 11             | <span class="swatch bg-person"></span>(220, 20, 60)         |
| ~~Rider~~ (overwritten by Car)          | 12             |                                                             |
| Car                                     | 13             | <span class="swatch bg-car"></span>(0, 0, 142)              |
| ~~Truck~~ (overwritten by Car)          | 14             |                                                             |
| ~~Bus~~ (overwritten by Car)            | 15             |                                                             |
| ~~Train~~ (overwritten by Car)          | 16             |                                                             |
| ~~Motorcycle~~ (overwritten by Bicycle) | 17             |                                                             |
| Bicycle                                 | 18             | <span class="swatch bg-bicycle"></span>(119, 11, 32)        |
