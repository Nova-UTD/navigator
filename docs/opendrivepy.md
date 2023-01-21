---
layout: default
title: OpenDrivePy
---

# OpenDrivePy

{: .no_toc }

_Maintained by Will Heitman_

## Table of contents

{: .no_toc .text-delta }

1. TOC
   {:toc}

---

## Element tree

```
Map
├── get_route(): Lane[]
├── header: Header (contains north, x0, and other geo ref data)
├── shapes: STRTree
├── shapes: STRTree
├── shapes: STRTree
├── roads: Road[]
│   └── Road 1
│       ├── name: str
│       ├── id: int
│       ├── length: float
│       ├── junction: int
│       ├── speed_limit: float (m/s, should convert otherwise)
│       ├── type: RoadType
│       ├── next: Road | Junction
│       ├── prev: Road | Junction
│       ├── refline: LineString
│       ├── lane_offset: LineString
│       └── sections: [LaneSection]
│           ├── s: float
│           ├── signals: Signal[] (speed limit signs, traffic lights, ...)
│           └── lanes: Lane[]
│               ├── Lane A
│               │   ├── id: int
│               │   ├── lsec: LaneSection (parent reference)
│               │   ├── road: Road (grandparent reference)
│               │   ├── type: LaneType ("shoulder", "sidewalk", "driving", ...)
│               │   ├── predecessors: Lane[]
│               │   ├── successors: Lane[]
│               │   └── shape: Polygon
│               └── Lane B
│                   └── ...
├── controllers: Controller[]
└── junctions: Junction[]
```

![image-20230103235217600](/home/main/.config/Typora/typora-user-images/image-20230103235217600.png)

<small>Above: Lane polygons. Errors with the rendering of curved lanes and lane ocause some imperfection.</small>

![image-20230103235227882](/home/main/.config/Typora/typora-user-images/image-20230103235227882.png)

<small>Above: Drivable map using prepared geometry</small>

![image-20230104001856539](/home/main/.config/Typora/typora-user-images/image-20230104001856539.png)

<small>Above: Drivable map using STRTree</small>