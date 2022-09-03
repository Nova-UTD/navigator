---
layout: post
title: Behavior Planning and Controls - Egan Johnson
date: 2022-01-11
categories: Deep Dives
author: "Dylan Hiu"
---

[![Headshot of Egan](/assets/res/headshots/Egan_Johnson.jpg)](/assets/res/headshots/Egan_Johnson.jpg)

Hello Comets! From all of us on the Nova team, we hope that you had a restful, restorative, and comfortable break. As we head back to our campus in these uncertain times, we are all hoping that you stay safe, take care of one another, and have a productive start to the year.

While all of us are coming back to our campus and preparing for our classes, Nova is continuing as usual. We have been working our hardest in order to successfully demonstrate a functional autonomous vehicle in just a few short months. Our team has worked continuously throughout the winter break to get closer to bringing this project to life, and we are so excited to share our work with you soon! In the meantime, we would like to shine a spotlight this week on one of the members of our Behavior Planning and Controls team, Egan Johnson!

Egan is one of the members of our Behavior Planning & Controls team. Egan is a sophomore studying computer science and hails from the Madison, Wisconsin area. He greatly enjoys running, and was nationally competitive on his high school cross country team.

For Nova, Egan is something of a veteran, being one of the original members of the Nova team.  Last year, Egan helped work on connecting the data taken from Nova’s mounted LIDAR sensors and converted it into workable data for the algorithms held within Nova’s onboard computer, the Jetson. He was responsible for designing the softwares that converted this information into the “point clouds” that frequent nearly all of Nova’s simulations and programs. 

This year, as part of the Behavior Planning & Controls team, Egan is working extensively on Hail Bopp’s route planner. While the route planner already existed on the vehicle, the older version of the system was very rudimentary, and could only navigate a very simple route in an empty parking lot. The existing system did not include the capability to navigate anything other than bidirectional roads and could not work with lane changes (both of which are, of course, very important to get right when we are trying to get this vehicle onto real roads!). This is now Egan’s project: designing a machine that can plot a path to get from one point to another with the “least cost,” or with the least amount of distance or travel time. This time, the process needs to account for every possible route with all lanes on the road considered!


[![Datamap with Raw View](/assets/res/22-01-11_Egan_Johnson-DataMap.png)](/assets/res/22-01-11_Egan_Johnson-DataMap.png)

[![Datamap with Map View](/assets/res/22-01-11_Egan_Johnson-OverviewMap.png)](/assets/res/22-01-11_Egan_Johnson-OverviewMap.png)

This is a complex process, involving the softwares in the machine retrieving information from the onboard LIDAR and depth map sensors, along with the calculations from the Perception team’s algorithms, in order to conceptualize all safe, followable paths for the vehicle. These potential paths are updated constantly many times per second in order to have up-to-date routes as new obstacles or variables present themselves around the vehicle. Out of all of these routes, one specific path is chosen by the software for the vehicle to follow, with the safest and most comfortable path being chosen (the code and decisions made on how which of these paths best fit these characteristics will be covered in a Deep-Dive to come!) After this curve is generated and chosen, the path information is passed on to the Controls team, where that data is converted into commands for the wheel, throttle, and brake motors to actuate in specific patterns in order to follow the given path. 

Egan has spent a great deal of time working on Nova. He has a great deal of experience working on the project, and from this, has a piece of advice for students: get involved! From his point of view, Egan says that there are so many small details that go into designing these systems that just are not seen from the outside. There is so much to learn from getting more deeply invested in projects. For Egan, this experience has been great…and for you, the reader, Egan recommends that you do the same! There is so much to gain from being involved, and it can open so many doors to fantastic projects and experience. 

As we continue to work on our project, expect to hear much more from us soon. We cannot wait to show you what we have done, and we wish you all good luck on the start to the new school year!

