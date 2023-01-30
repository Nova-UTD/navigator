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

| Description   | Class ID (int) | RGB                                                         |
| ------------- | -------------- | ----------------------------------------------------------- |
| Road          | 0              | <span class="swatch bg-road"></span>(128, 64, 128)          |
| Sidewalk      | 1              | <span class="swatch bg-sidewalk"></span>(244, 35, 232)      |
| Parking       |                |                                                             |
| Rail track    |                |                                                             |
| Person        | 11             | <span class="swatch bg-person"></span>(220, 20, 60)         |
| Rider         |                |                                                             |
| Car           | 13             | <span class="swatch bg-car"></span>(0, 0, 142)              |
| Truck         |                |                                                             |
| Bus           |                |                                                             |
| On rails      |                |                                                             |
| Motorcycle    |                |                                                             |
| Bicycle       |                |                                                             |
| Caravan       |                |                                                             |
| Trailer       |                |                                                             |
| Building      |                | <span class="swatch bg-building"></span>(70, 70, 70)        |
| Wall          |                |                                                             |
| Fence         | 3              | <span class="swatch bg-fence"></span>(100, 40, 40)          |
| Guard rail    |                |                                                             |
| Bridge        |                |                                                             |
| Tunnel        |                |                                                             |
| Pole          | 5              | <span class="swatch bg-pole"></span>(153, 153, 153)         |
| Traffic sign  |                | <span class="swatch bg-traffic-sign"></span>(220, 220, 0)   |
| Traffic light | 6              | <span class="swatch bg-traffic-light"></span>(250, 170, 30) |
| Vegetation    | 8              | <span class="swatch bg-vegetation"></span>(107, 142, 35)    |
| Terrain       | 9              | <span class="swatch bg-terrain"></span>(145, 170, 100)      |
| Sky           | 10             | <span class="swatch bg-sky"></span>(70, 130, 180)           |
